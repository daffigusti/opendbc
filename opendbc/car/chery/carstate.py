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
    }
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["ENGINE_DATA"]["GEAR"]
    self.lkas_cmd = {}
    self.acc_cmd = {}
    self.buttons_stock_values = {}
    self.brake_pos = 0
    self.front_wheel_speed = 0.0
    self.acc_active = False
    self.lkas_state = {}
    self.eps_dead_frames = 0

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    pt_messages = [
      ("STEER_ANGLE_SENSOR", 100), ("WHEEL_SPEED_FRNT", 50),
      ("WHEEL_SPEED_REAR", 50), ("BRAKE_DATA", 50), ("ENGINE_DATA", 100),
      ("STEER_SENSOR_2", 50), ("STEER_BUTTON", 20),
      ("BCM_SIGNAL_1", 50), ("LKAS", 100),
    ]
    if CP.enableBsm:
      pt_messages += [("BSM_LEFT", 10), ("BSM_RIGHT", 10)]
    cam_messages = [
      ("ACC_CMD", 50), ("ACC", 50), ("LKAS_CAM_CMD_345", 50),
      ("LKAS_STATE", 20), ("SETTING", 20), ("LEAD_FRONT", 20),
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
      "RES_PLUS": structs.CarState.ButtonEvent.Type.accelCruise,
      "RES_MINUS": structs.CarState.ButtonEvent.Type.decelCruise,
    }
    events = []
    for signal, event_type in button_types.items():
      pressed = bool(buttons[signal])
      if pressed != self.button_states[event_type]:
        event = structs.CarState.ButtonEvent.new_message()
        event.type = event_type
        event.pressed = pressed
        events.append(event)
      self.button_states[event_type] = pressed
    return events

  def _update_eps_fault(self, ret, cp_loopback, cp) -> bool:
    """Latch a temporary steer fault when the EPS stops acting on a command openpilot is sending.

    LKAS.LKAS_CMD pinned at -1 while LKAS reports itself engaged means the servo has gone dead.
    The loopback copy of our own 0x345 is what says openpilot is actually commanding, which is
    the check the working fork made with CC.latActive.
    """
    commanding = cp_loopback.vl["LKAS_CAM_CMD_345"]["LKA_ACTIVE"] == 1
    if ret.cruiseState.enabled and ret.vEgo > self.CP.minSteerSpeed:
      if commanding and cp.vl["LKAS"]["LKAS_CMD"] == -1 and cp.vl["LKAS"]["NEW_SIGNAL_1"] == 1:
        self.eps_dead_frames += 1
      else:
        self.eps_dead_frames = 0
    else:
      self.eps_dead_frames = 0
    return self.eps_dead_frames >= CarControllerParams.STEER_TIMEOUT

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
    ret.steeringTorque = cp.vl["STEER_SENSOR_2"]["TORQUE_DRIVER"]
    ret.steeringTorqueEps = cp.vl["STEER_ANGLE_SENSOR"]["TORQUE"]
    # TORQUE_DRIVER's sign is unverified, so only its magnitude is used. The threshold is the
    # one the working fork runs with; see KNOWN_GAPS.md.
    ret.steeringPressed = abs(ret.steeringTorque) > CarControllerParams.STEER_THRESHOLD

    ret.cruiseState.available = cp_cam.vl["SETTING"]["ACC_AVAILABLE"] in (1, 2)
    ret.cruiseState.enabled = bool(cp_cam.vl["ACC"]["ACC_ACTIVE"] or cp_cam.vl["ACC_CMD"]["STOPPED"])
    ret.cruiseState.speed = cp_cam.vl["SETTING"]["CC_SPEED"] * CV.KPH_TO_MS
    # The stock ACC drops ACC_ACTIVE ~3s into a standstill hold and then ignores ACC_CMD gas
    # until a RES+ press. Report that -- not plain vEgo -- as cruise standstill, so controlsd
    # asks for a resume. It has to clear the moment ACC_ACTIVE returns, otherwise
    # long_control_state_trans keeps starting_condition False and the car stays held after the
    # button lands.
    ret.cruiseState.standstill = not self.acc_active and bool(cp_cam.vl["ACC_CMD"]["STOPPED"])
    ret.stockAeb = cp_cam.vl["ACC"]["AEB_ACTIVE"] == 1
    ret.stockFcw = False
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(int(cp.vl["ENGINE_DATA"]["GEAR"])))
    ret.leftBlinker = cp.vl["BCM_SIGNAL_1"]["SIGN_SIGNAL"] == 2
    ret.rightBlinker = cp.vl["BCM_SIGNAL_1"]["SIGN_SIGNAL"] == 1
    self.lkas_cmd = cp_cam.vl["LKAS_CAM_CMD_345"].copy()
    self.lkas_state = cp_cam.vl["LKAS_STATE"].copy()
    self.acc_cmd = cp_cam.vl["ACC_CMD"].copy()
    ret.steerFaultTemporary = self._update_eps_fault(ret, can_parsers[Bus.loopback], cp)
    if self.CP.enableBsm:
      ret.leftBlindspot = bool(cp.vl["BSM_LEFT"]["BSM_LEFT_DETECT"])
      ret.rightBlindspot = bool(cp.vl["BSM_RIGHT"]["BSM_RIGHT_DETECT"])
    ret.doorOpen = False
    ret.seatbeltUnlatched = False
    self.buttons_stock_values = cp.vl["STEER_BUTTON"].copy()
    ret.buttonEvents = self._button_events(cp.vl["STEER_BUTTON"])
    return ret, ret_sp
