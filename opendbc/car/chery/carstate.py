from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.chery.cherycan import CanBus
from opendbc.car.chery.values import DBC
from opendbc.car.interfaces import CarStateBase


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

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    pt_messages = [
      ("STEER_ANGLE_SENSOR", 100), ("WHEEL_SPEED_FRNT", 50),
      ("WHEEL_SPEED_REAR", 50), ("BRAKE_DATA", 50), ("ENGINE_DATA", 100),
      ("STEER_SENSOR_2", 50), ("STEER_BUTTON", 20),
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

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()
    ret_sp = structs.CarStateSP()
    fl = cp.vl["WHEEL_SPEED_FRNT"]["WHEEL_SPEED_FL"] * CV.KPH_TO_MS
    fr = cp.vl["WHEEL_SPEED_FRNT"]["WHEEL_SPEED_FR"] * CV.KPH_TO_MS
    rl = cp.vl["WHEEL_SPEED_REAR"]["WHEEL_SPEED_RL"] * CV.KPH_TO_MS
    rr = cp.vl["WHEEL_SPEED_REAR"]["WHEEL_SPEED_RR"] * CV.KPH_TO_MS
    ret.wheelSpeeds.fl, ret.wheelSpeeds.fr = fl, fr
    ret.wheelSpeeds.rl, ret.wheelSpeeds.rr = rl, rr
    self.parse_wheel_speeds(ret, fl, fr, rl, rr, unit=1.0)
    ret.standstill = ret.vEgoRaw < 1e-3
    ret.gasPressed = (bool(cp_cam.vl["ACC_CMD"]["GAS_PRESSED"]) if cp_cam.vl["ACC"]["ACC_ACTIVE"]
                      else cp.vl["ENGINE_DATA"]["GAS"] > 1)
    self.brake_pos = cp.vl["BRAKE_DATA"]["BRAKE_POS"]
    ret.brakePressed = cp.vl["ENGINE_DATA"]["BRAKE_PRESS"] != 0
    ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"]
    ret.steeringTorque = cp.vl["STEER_SENSOR_2"]["TORQUE_DRIVER"]
    ret.steeringTorqueEps = cp.vl["STEER_ANGLE_SENSOR"]["TORQUE"]
    ret.steeringPressed = abs(ret.steeringTorque) > 1.0

    ret.cruiseState.available = cp_cam.vl["SETTING"]["ACC_AVAILABLE"] in (1, 2)
    ret.cruiseState.enabled = bool(cp_cam.vl["ACC"]["ACC_ACTIVE"] or cp_cam.vl["ACC_CMD"]["STOPPED"])
    ret.cruiseState.speed = cp_cam.vl["SETTING"]["CC_SPEED"] * CV.KPH_TO_MS
    ret.cruiseState.standstill = ret.standstill
    ret.stockAeb = cp_cam.vl["ACC"]["AEB_ACTIVE"] == 1
    ret.stockFcw = False
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(int(cp.vl["ENGINE_DATA"]["GEAR"])))
    self.lkas_cmd = cp_cam.vl["LKAS_CAM_CMD_345"].copy()
    self.acc_cmd = cp_cam.vl["ACC_CMD"].copy()
    if self.CP.enableBsm:
      ret.leftBlindspot = bool(cp.vl["BSM_LEFT"]["BSM_LEFT_DETECT"])
      ret.rightBlindspot = bool(cp.vl["BSM_RIGHT"]["BSM_RIGHT_DETECT"])
    ret.doorOpen = False
    ret.seatbeltUnlatched = False
    self.buttons_stock_values = cp.vl["STEER_BUTTON"].copy()
    ret.buttonEvents = self._button_events(cp.vl["STEER_BUTTON"])
    return ret, ret_sp
