from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.chery.cherycan import CanBus
from opendbc.car.chery.values import CarControllerParams, DBC
from opendbc.car.interfaces import CarStateBase


# Sits above the throttle openpilot's own hold request echoes back through ENGINE_DATA.GAS,
# which ramps in 25.6 steps to at most 205 while the pedal reads untouched. Real driver presses
# in the same logs measured 486..2442.
GAS_PRESSED_THRESHOLD = 300


class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.button_states = {
      structs.CarState.ButtonEvent.Type.accelCruise: False,
      structs.CarState.ButtonEvent.Type.decelCruise: False,
      structs.CarState.ButtonEvent.Type.gapAdjustCruise: False,
    }
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["ENGINE_DATA"]["GEAR"]
    self.lkas_cmd = {}
    self.acc_cmd = {}
    self.buttons_stock_values = {}
    self.brake_pos = 0
    self.front_wheel_speed = 0.0
    self.acc_active = False
    self.cruise_enabled_prev = False
    self.lkas_state = {}
    self.gap_setting = 0
    self.hud_alert = {}
    self.eps_dead_frames = 0
    self.eps_inactive = False
    self.steer_angle_hr_last = 0.0
    self.steer_rate_sign = 1

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    pt_messages = [
      ("STEER_ANGLE_SENSOR", 100), ("WHEEL_SPEED_FRNT", 50),
      ("WHEEL_SPEED_REAR", 50), ("BRAKE_DATA", 50), ("ENGINE_DATA", 100),
      ("STEER_SENSOR_2", 50), ("STEER_BUTTON", 20),
      ("BCM_SIGNAL_1", 50), ("LKAS", 100), ("NEW_MSG_430", 50), ("STEER_SENSOR", 100),
    ]
    if CP.enableBsm:
      pt_messages += [("BSM_LEFT", 10), ("BSM_RIGHT", 10)]
    cam_messages = [
      ("ACC_CMD", 50), ("ACC", 50), ("LKAS_CAM_CMD_345", 50),
      ("LKAS_STATE", 20), ("SETTING", 20), ("LEAD_FRONT", 20), ("HUD_ALERT", 20),
    ]
    loopback_messages = [("LKAS_CAM_CMD_345", 0), ("ACC_CMD", 0)]
    can_bus = CanBus(CP)
    dbc = DBC[CP.carFingerprint][Bus.pt]
    return {
      Bus.pt: CANParser(dbc, pt_messages, can_bus.main),
      Bus.cam: CANParser(dbc, cam_messages, can_bus.camera),
      Bus.loopback: CANParser(dbc, loopback_messages, can_bus.loopback),
    }

  def _button_events(self, buttons):
    button_types = {
      structs.CarState.ButtonEvent.Type.accelCruise: ("RES_PLUS",),
      structs.CarState.ButtonEvent.Type.decelCruise: ("RES_MINUS",),
      # openpilot has one distance button that cycles personality; the carcontroller then taps
      # the stock gap to match, so either direction on the wheel is a cycle press.
      structs.CarState.ButtonEvent.Type.gapAdjustCruise: ("GAP_ADJUST_UP", "GAP_ADJUST_DOWN"),
    }
    events = []
    for event_type, signals in button_types.items():
      pressed = any(buttons[signal] for signal in signals)
      if pressed != self.button_states[event_type]:
        event = structs.CarState.ButtonEvent.new_message()
        event.type = event_type
        event.pressed = pressed
        events.append(event)
      self.button_states[event_type] = pressed
    return events

  def _update_eps_fault(self, ret, cp_loopback, cp) -> bool:
    """Latch a temporary steer fault when the EPS stops acting on a command openpilot is sending.

    LKAS reporting itself inactive (EPS_TORQUE at its 1023 sentinel) while openpilot commands means the servo has gone dead.
    The loopback copy of our own 0x345 is what says openpilot is actually commanding, which is
    the check the working fork made with CC.latActive. The count holds through the carcontroller's
    re-arm gaps, where nothing is commanded, and only clears once the EPS comes back.
    """
    commanding = cp_loopback.vl["LKAS_CAM_CMD_345"]["LKA_ACTIVE"] == 1
    if not ret.cruiseState.enabled or ret.vEgo <= self.CP.minSteerSpeed or not self.eps_inactive:
      self.eps_dead_frames = 0
    elif commanding:
      self.eps_dead_frames += 1
    return self.eps_dead_frames >= CarControllerParams.STEER_TIMEOUT

  def _steering_rate(self, steer_sensor) -> float:
    """STEER_SENSOR.STEER_RATE is unsigned; the direction comes from its high-resolution angle.

    Fitted on 45k frames of a real route: |rate| = 4 deg/s per LSB (r=0.992), and STEER_ANGLE_HR
    matches STEER_ANGLE at 0.0625 deg per LSB (r=1.000). The sign is held while STEER_RATE reads 0:
    a still wheel jitters one LSB either way (339 flips in 9.7k still frames), which would otherwise
    flip the borrowed sign of a driver holding the wheel.
    """
    angle = steer_sensor["STEER_ANGLE_HR"]
    if angle != self.steer_angle_hr_last and steer_sensor["STEER_RATE"] > 0:
      self.steer_rate_sign = 1 if angle > self.steer_angle_hr_last else -1
    self.steer_angle_hr_last = angle
    return self.steer_rate_sign * steer_sensor["STEER_RATE"]

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()
    ret_sp = structs.CarStateSP()
    fl = cp.vl["WHEEL_SPEED_FRNT"]["WHEEL_SPEED_FL"] * CV.KPH_TO_MS
    fr = cp.vl["WHEEL_SPEED_FRNT"]["WHEEL_SPEED_FR"] * CV.KPH_TO_MS
    rl = cp.vl["WHEEL_SPEED_REAR"]["WHEEL_SPEED_RL"] * CV.KPH_TO_MS
    rr = cp.vl["WHEEL_SPEED_REAR"]["WHEEL_SPEED_RR"] * CV.KPH_TO_MS
    self.front_wheel_speed = (fl + fr) / 2
    ret.wheelSpeeds.fl, ret.wheelSpeeds.fr = fl, fr
    ret.wheelSpeeds.rl, ret.wheelSpeeds.rr = rl, rr
    self.parse_wheel_speeds(ret, fl, fr, rl, rr, unit=1.0)
    ret.standstill = ret.vEgoRaw < 1e-3
    # ENGINE_DATA.GAS is the throttle the powertrain is executing, not pedal travel. During an
    # ACC hold it is openpilot's own request echoed back, so the old `> 1` test read that echo as
    # a driver press: openpilot handed off to overriding, the request stopped, GAS fell, it
    # re-engaged, and the request rose again -- a ~0.4s lurch-and-hold loop. While the ACC owns
    # the throttle, trust only the camera's own driver-pedal bit.
    self.acc_active = bool(cp_cam.vl["ACC"]["ACC_ACTIVE"])
    ret.gasPressed = (bool(cp_cam.vl["ACC_CMD"]["GAS_PRESSED"]) if self.acc_active else
                      cp.vl["ENGINE_DATA"]["GAS"] > GAS_PRESSED_THRESHOLD)
    self.brake_pos = cp.vl["BRAKE_DATA"]["BRAKE_POS"]
    ret.brakePressed = cp.vl["ENGINE_DATA"]["BRAKE_PRESS"] != 0
    ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"]
    ret.steeringRateDeg = self._steering_rate(cp.vl["STEER_SENSOR"])
    # TORQUE_DRIVER only ever reads positive, so it is a magnitude. desire_helper needs a sign to
    # confirm a lane change, so borrow it from which way the wheel is turning, as the working fork
    # does. ponytail: a driver pushing against the wheel without moving it keeps the last sign.
    ret.steeringTorque = cp.vl["STEER_SENSOR_2"]["TORQUE_DRIVER"] * self.steer_rate_sign
    ret.steeringTorqueEps = cp.vl["STEER_ANGLE_SENSOR"]["TORQUE"]
    # The threshold is the one the working fork runs with; see KNOWN_GAPS.md.
    ret.steeringPressed = abs(ret.steeringTorque) > CarControllerParams.STEER_THRESHOLD

    # Mirror the panda's chery_pcm_cruise_check. The camera can abort with ACC_STATE=0 and STOPPED=1
    # while ACC_ACTIVE still reads 1 for ~300ms; the panda drops controls on that frame, and if
    # openpilot stayed engaged its blocked sends timed out the loopback parser and raised canError.
    acc_state = cp_cam.vl["ACC_CMD"]["ACC_STATE"]
    gas_override = acc_state == 1 and bool(cp_cam.vl["ACC_CMD"]["GAS_PRESSED"]) and self.acc_active
    acc_engaged = self.acc_active or (bool(cp_cam.vl["ACC_CMD"]["STOPPED"]) and self.cruise_enabled_prev)
    # ACC_AVAILABLE reads 3 while the driver overrides with the accelerator and ACC_ACTIVE stays
    # 1, and again once the standstill hold drops ACC_ACTIVE to wait for RES+. Treating either as
    # unavailable raised wrongCarMode: every gas press dropped lateral, and every hold past ~3s
    # disengaged openpilot before it could tap RES+ (route 494 seg 22), so the car never resumed.
    acc_available = cp_cam.vl["SETTING"]["ACC_AVAILABLE"]
    ret.cruiseState.available = acc_available in (1, 2) or (acc_available == 3 and acc_engaged)
    ret.cruiseState.enabled = acc_engaged and (acc_state in (2, 3) or gas_override)
    self.cruise_enabled_prev = ret.cruiseState.enabled
    ret.cruiseState.speed = cp_cam.vl["SETTING"]["CC_SPEED"] * CV.KPH_TO_MS
    # Stock ACC following distance: five levels, 5 the farthest (owner-identified on the cluster).
    self.gap_setting = int(cp_cam.vl["SETTING"]["GAP"])
    # The stock ACC drops ACC_ACTIVE ~3s into a standstill hold and then ignores ACC_CMD gas
    # until a RES+ press. Report that -- not plain vEgo -- as cruise standstill, so controlsd
    # asks for a resume. It has to clear the moment ACC_ACTIVE returns, otherwise
    # long_control_state_trans keeps starting_condition False and the car stays held after the
    # button lands.
    ret.cruiseState.standstill = not self.acc_active and bool(cp_cam.vl["ACC_CMD"]["STOPPED"])
    # Route 1b6 braked with SETTING.AEB_ACTIVE=3. ACC.AEB_ACTIVE rose for route 488's dash collision
    # warning with AEB switched off, and for a second 1b6 stop that SETTING never flagged.
    ret.stockAeb = cp_cam.vl["SETTING"]["AEB_ACTIVE"] == 3
    ret.stockFcw = cp_cam.vl["ACC"]["AEB_ACTIVE"] == 1 and not ret.stockAeb
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(int(cp.vl["ENGINE_DATA"]["GEAR"])))
    ret.leftBlinker = cp.vl["BCM_SIGNAL_1"]["SIGN_SIGNAL"] == 2
    ret.rightBlinker = cp.vl["BCM_SIGNAL_1"]["SIGN_SIGNAL"] == 1
    self.lkas_cmd = cp_cam.vl["LKAS_CAM_CMD_345"].copy()
    self.lkas_state = cp_cam.vl["LKAS_STATE"].copy()
    self.hud_alert = cp_cam.vl["HUD_ALERT"].copy()
    self.acc_cmd = cp_cam.vl["ACC_CMD"].copy()
    self.eps_inactive = cp.vl["LKAS"]["EPS_TORQUE"] == 1023 and cp.vl["LKAS"]["EPS_INACTIVE"] == 1
    ret.steerFaultTemporary = self._update_eps_fault(ret, can_parsers[Bus.loopback], cp)
    if self.CP.enableBsm:
      ret.leftBlindspot = bool(cp.vl["BSM_LEFT"]["BSM_LEFT_DETECT"])
      ret.rightBlindspot = bool(cp.vl["BSM_RIGHT"]["BSM_RIGHT_DETECT"])
    # Confirmed on one route: the door bits rise only in park or at a crawl as occupants get in
    # and out, and SEATBELT reads 1 in park before buckling and after the doors open at the end,
    # and 0 for the whole drive.
    ret.doorOpen = any(cp.vl["BCM_SIGNAL_1"][door] for door in ("FL_DOOR_OPEN", "FR_DOOR_OPEN", "RL_DOOR_OPEN", "RR_DOOR_OPEN"))
    ret.seatbeltUnlatched = cp.vl["NEW_MSG_430"]["SEATBELT"] == 1
    self.buttons_stock_values = cp.vl["STEER_BUTTON"].copy()
    ret.buttonEvents = self._button_events(cp.vl["STEER_BUTTON"])
    return ret, ret_sp
