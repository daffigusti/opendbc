from opendbc.car import get_safety_config, structs
from opendbc.car.chery.cherycan import CanBus
from opendbc.car.chery.carstate import CarState
from opendbc.car.chery.values import CarControllerParams, CherySafetyFlags
from opendbc.car.interfaces import CarControllerBase, CarInterfaceBase


class CarController(CarControllerBase):
  def update(self, CC, CC_SP, CS, now_nanos):
    return structs.CarControl.Actuators(), []


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "chery"
    CAN = CanBus(fingerprint=fingerprint)
    safety_configs = [get_safety_config(structs.CarParams.SafetyModel.cheryCanFd)]
    if CAN.main >= 4:
      safety_configs.insert(0, get_safety_config(structs.CarParams.SafetyModel.noOutput))
    ret.safetyConfigs = safety_configs
    ret.radarUnavailable = True
    ret.alphaLongitudinalAvailable = True
    ret.openpilotLongitudinalControl = alpha_long
    if alpha_long:
      ret.safetyConfigs[-1].safetyParam |= CherySafetyFlags.LONG_CONTROL.value
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.transmissionType = structs.CarParams.TransmissionType.direct
    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 1.0
    ret.longitudinalActuatorDelay = 0.05
    ret.stopAccel = CarControllerParams.ACCEL_MIN
    ret.minEnableSpeed = -1.
    ret.minSteerSpeed = -1.
    ret.autoResumeSng = True
    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    CAN = CanBus(fingerprint=fingerprint)
    stock_cp.enableBsm = 0x4B1 in fingerprint[CAN.main] and 0x4B3 in fingerprint[CAN.main]
    return ret
