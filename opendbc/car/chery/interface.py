from opendbc.car import get_safety_config, structs
from opendbc.car.chery.values import CarControllerParams, CherySafetyFlags
from opendbc.car.interfaces import CarInterfaceBase


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "chery"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.cheryCanFd)]
    ret.radarUnavailable = True
    ret.alphaLongitudinalAvailable = True
    ret.openpilotLongitudinalControl = alpha_long
    if alpha_long:
      ret.safetyConfigs[-1].safetyParam |= CherySafetyFlags.LONG_CONTROL
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.transmissionType = structs.CarParams.TransmissionType.direct
    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 1.0
    ret.longitudinalActuatorDelay = 0.05
    ret.stopAccel = CarControllerParams.ACCEL_MIN
    ret.vEgoStarting = 0.1
    ret.vEgoStopping = 0.1
    ret.minEnableSpeed = -1.
    ret.minSteerSpeed = -1.
    ret.autoResumeSng = True
    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    stock_cp.enableBsm = 0x3A7 in fingerprint[2]
    return ret
