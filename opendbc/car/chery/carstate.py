import copy

from opendbc.car.common.conversions import Conversions as CV
from opendbc.can.can_define import CANDefine
from opendbc.car import Bus, create_button_events, structs
from opendbc.can.parser import CANParser
from opendbc.car.interfaces import CarStateBase
from opendbc.car.chery.values import DBC, CarControllerParams, CanBus
from opendbc.sunnypilot.car.chery.mads import MadsCarState

ButtonType = structs.CarState.ButtonEvent.Type

class CarState(CarStateBase, MadsCarState):
  def __init__(self, CP, CP_SP):
    CarStateBase.__init__(self, CP, CP_SP)
    MadsCarState.__init__(self, CP, CP_SP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.params = CarControllerParams(CP)

    self.shifter_values = can_define.dv["ENGINE_DATA"]["GEAR"]
    self.button_states = {button.event_type: False for button in self.params.BUTTONS}

    self.frame = 0
    self.angleSensorLast = 0
    self.direction= 1
    self.prev_distance_button = 0
    self.distance_button = 0
    self.cruise_decreased = 0
    self.cruise_increased = 0
    self.prev_main_button = 0
    self.lkas_status = 0
    self.main_button = 0
    self.lkas_enabled = False
    self.prev_lkas_enabled = False
    self.mainEnabled = False
    self.mads_enabled = False
    self.last_change_time = 0.0

    # Detect if servo stop responding to steering command.
    self.cruiseState_enabled_prev = False
    self.eps_torque_timer = 0

  def update_button_enable(self, buttonEvents: list[structs.CarState.ButtonEvent]):
    if not self.CP.pcmCruise:
      for b in buttonEvents:
        # Enable OP long on falling edge of enable buttons
        if b.type in (ButtonType.setCruise, ButtonType.resumeCruise) and not b.pressed:
          return True
    return False
  def create_button_events(self, cp, buttons):
    button_events = []

    for button in buttons:
      state = cp.vl[button.can_addr][button.can_msg] in button.values

      if self.button_states[button.event_type] != state:
        event = structs.CarState.ButtonEvent.new_message()
        event.type = button.event_type
        event.pressed = state
        button_events.append(event)
      self.button_states[button.event_type] = state
    return button_events

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    loopback_cp = can_parsers[Bus.loopback]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()
    # car speed
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEED_FRNT"]["WHEEL_SPEED_FR"],
      cp.vl["WHEEL_SPEED_FRNT"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEED_REAR"]["WHEEL_SPEED_RR"],
      cp.vl["WHEEL_SPEED_REAR"]["WHEEL_SPEED_RL"],
    )

    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.vEgoCluster = ret.vEgo
    ret.standstill = ret.vEgoRaw < 1e-3

    self.acc_md = copy.copy(cp_cam.vl["ACC_CMD"])
    self.lkas = copy.copy(cp.vl["LKAS"])
    self.lkas_state = copy.copy(cp_cam.vl["LKAS_STATE"])
    self.setting = copy.copy(cp_cam.vl["SETTING"])
    self.lkas_cmd = copy.copy(cp_cam.vl["LKAS_CAM_CMD_345"])

    # steer_angle = cp.vl["STEER_SENSOR"]["ANGLE"]
    # steer_angle_fraction = cp.vl["STEER_SENSOR"]["FRACTION"]

    # gas pedal
    self.gasPos = cp.vl["ENGINE_DATA"]["GAS"]
    # ret.gas = 0 if self.gasPos >= 2559 or self.gasPos<=0 else self.gasPos
    ret.gas = self.gasPos
    # ret.gasPressed = ret.gas > 1
    ret.gasPressed = (cp_cam.vl["ACC_CMD"]["GAS_PRESSED"]==1) if (cp_cam.vl["ACC"]["ACC_ACTIVE"] != 0) else (ret.gas > 1)

    # brake pedal
    ret.brake = cp.vl["BRAKE_DATA"]["BRAKE_POS"]
    ret.brakePressed = cp.vl["ENGINE_DATA"]["BRAKE_PRESS"] != 0

    # gear
    can_gear = int(cp.vl["ENGINE_DATA"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    # button presses
    ret.leftBlinker = cp.vl["BCM_SIGNAL_1"]["SIGN_SIGNAL"] == 2
    ret.rightBlinker = cp.vl["BCM_SIGNAL_1"]["SIGN_SIGNAL"] == 1

    # steering wheel
    self.agleSensor = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"]

    if  (self.frame  % 10) == 0:
      if(self.agleSensor<self.angleSensorLast):
        self.direction = -1
      else:
        self.direction = 1
      self.angleSensorLast = self.agleSensor

    # ret.steeringAngleDeg = (int(steer_angle_fraction) << 8) + steer_angle - 2048
    ret.steeringAngleDeg = self.agleSensor

    ret.steeringTorque = cp.vl["STEER_SENSOR_2"]["TORQUE_DRIVER"] * self.direction

    ret.steeringTorqueEps = cp.vl["STEER_ANGLE_SENSOR"]['TORQUE']

    ret.steeringPressed = abs(ret.steeringTorque) > CarControllerParams.STEER_THRESHOLD

    self.steerTemporaryUnvailable = False
    self.lkas_status_before = self.lkas_status
    self.lkas_status = cp.vl["LKAS"]['NEW_SIGNAL_1']

    if ret.cruiseState.enabled and ret.vEgo > self.CP.minSteerSpeed:
       # Reset counter on entry
      if self.cruiseState_enabled_prev != ret.cruiseState.enabled:
        self.eps_torque_timer = 0
      # Count up when no torque from servo detected.
      if loopback_cp.vl["LKAS_STATE"]['LKA_ACTIVE'] == 1 and cp.vl["LKAS"]['LKAS_CMD'] == -1 and self.lkas_status == 1:
        self.eps_torque_timer += 1
      else:
        self.eps_torque_timer = 0
      # Set fault if above threshold
      ret.steerFaultTemporary = self.eps_torque_timer >= CarControllerParams.STEER_TIMEOUT

    self.cruiseState_enabled_prev = ret.cruiseState.enabled

    self.button_events = self.create_button_events(cp, self.params.BUTTONS)
    # cruise state
    # ret.cruiseState.available = cp_cam.vl["ACC_CMD"]["ACC_STATE"] != 1 or cp_cam.vl["ACC"]["ACC_ACTIVE"] != 0
    # ret.cruiseState.available =  cp_cam.vl["ACC"]["ACC_ACTIVE"] != 0
    ret.cruiseState.available =  True
    # ret.cruiseState.available = cam_csp.vl["ACC_CMD"]["ACC_STATE"] != 1 or cp_cam.vl["ACC"]["ACC_ACTIVE"] != 0
    ret.cruiseState.enabled = cp_cam.vl["ACC"]["ACC_ACTIVE"] != 0 or cp_cam.vl["ACC_CMD"]["STOPPED"] == 1
    self.lead_front  = (cp_cam.vl["LEAD_FRONT"]["LEAD_DISTANCE"]) if (cp_cam.vl["LEAD_FRONT"]["VALID_SIGNAL"] == 1)  else 0

    self.needResume = cp_cam.vl["ACC"]["ACC_ACTIVE"] == 0 and cp_cam.vl["ACC_CMD"]["STOPPED"] == 1
    ret.cruiseState.speed = cp_cam.vl["SETTING"]["CC_SPEED"] * CV.KPH_TO_MS
    # ret.cruiseState.enabled = cp_cam.vl["LKAS_STATE"]["STATE"] != 0
    ret.cruiseState.standstill = ret.standstill

    self.cruise_decreased_previously = self.cruise_decreased
    self.cruise_decreased = cp.vl["STEER_BUTTON"]["RES_MINUS"]
    self.cruise_increased_previously = self.cruise_increased
    self.cruise_increased = cp.vl["STEER_BUTTON"]["RES_PLUS"]

    self.prev_distance_button = self.distance_button
    self.distance_button = cp.vl["STEER_BUTTON"]["GAP_ADJUST_UP"]
    self.prev_main_button = self.main_button
    self.main_button = cp.vl["STEER_BUTTON"]["ACC"]

    self.buttons_stock_values = cp.vl["STEER_BUTTON"]
    # FrogPilot CarState functions
    self.lkas_previously_enabled = self.lkas_enabled
    self.lkas_enabled = cp_cam.vl["LKAS_STATE"]["LKA_ACTIVE"] != 0
    self.lkas_active =  cp.vl["LKAS"]['LKAS_CMD']
    self.acc_available = cp_cam.vl["SETTING"]["ACC_AVAILABLE"]

    # TODO: get the real value
    ret.stockAeb = False
    ret.stockFcw = False
    # blindspot sensors
    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["BSM_LEFT"]["BSM_LEFT_DETECT"] != 0
      ret.rightBlindspot = cp.vl["BSM_RIGHT"]["BSM_RIGHT_DETECT"] != 0

    # TODO: get the real value
    ret.doorOpen = False
    ret.seatbeltUnlatched = False

    ret.brakeLightsDEPRECATED = bool(ret.brakePressed)

    if self.CP.openpilotLongitudinalControl:
          if self.prev_main_button != 1:
            if self.main_button == 1:
              self.mainEnabled = not self.mainEnabled
          ret.cruiseState.available = ret.cruiseState.available and self.mainEnabled
    self.prev_mads_enabled = self.mads_enabled
    self.prev_lkas_enabled = self.lkas_enabled

    self.mads_enabled = ret.cruiseState.available

    ret.buttonEvents = self.create_button_events(cp, self.params.BUTTONS)
    # print('Steer Fraction: ', steer_angle_fraction)
    # print('retsteeringTorque: ', ret.steeringTorque)
    # print('brakePressed: ', ret.brakePressed)
    # print('agle sensor 1: ', ret.steeringAngleDeg)
    # print('agle sensor 2: ', self.agleSensor)
    # print('Steer Sensor Torque: ', ret.steeringTorque)
    # print('Engine: ', cp.vl["ENGINE_DATA"])
    # print('Button', ret.buttonEvents)

    self.frame += 1
    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    pt_messages = [
       ("STEER_ANGLE_SENSOR", 100),
       ("STEER_SENSOR", 100),
       ("WHEEL_SPEED_FRNT", 50),
       ("WHEEL_SPEED_REAR", 50),
       ("BCM_SIGNAL_1", 50),
       ("BCM_SIGNAL_2", 50),
       ("BRAKE_DATA", 50),
       ("LKAS", 100),
       ("ENGINE_DATA", 100),
       ("STEER_SENSOR_2", 59),
       ("STEER_BUTTON", 20),
    ]

    if CP.enableBsm:
      pt_messages += [
        ("BSM_LEFT", 10),
        ("BSM_RIGHT", 10),
      ]

    cam_messages = [
      ("ACC_CMD", 50),
      ("ACC", 50),
      ("LKAS_CAM_CMD_345", 50),
      ("LKAS_STATE", 20),
      ("SETTING", 20),
      ("LEAD_FRONT", 20),
    ]
    loopback_messages = [
      ("LKAS_STATE", 0),
    ]
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CanBus.main),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, CanBus.camera),
      Bus.loopback: CANParser(DBC[CP.carFingerprint][Bus.pt], loopback_messages, CanBus.loopback),
    }

