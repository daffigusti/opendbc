from opendbc.car import get_safety_config, structs
from opendbc.car.chery.cherycan import CanBus
from opendbc.car.chery.carcontroller import CarController
from opendbc.car.chery.carstate import CarState
from opendbc.car.chery.values import CarControllerParams, CherySafetyFlags
from opendbc.car.interfaces import CarInterfaceBase


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
    # Command to measured wheel angle lags 130ms on real routes.
    ret.steerActuatorDelay = 0.15
    ret.steerLimitTimer = 1.0
    # ACC_CMD to aEgo correlates best at a 0.4 s lag on route 00000494; 0.05 let the planner overshoot.
    ret.longitudinalActuatorDelay = 0.4
    # Floor longcontrol ramps the request down to while stopping. The car has never braked past
    # -2.81 m/s^2 (CMD saturated at -511), and at ACCEL_MIN 1 of 8 stops on route 000004ae bottomed
    # out asking for -3.5 and got -1.33. This still stops harder than openpilot's -2.0 default.
    ret.stopAccel = -2.5
    ret.minEnableSpeed = -1.
    ret.minSteerSpeed = -1.
    ret.autoResumeSng = True
    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    CAN = CanBus(fingerprint=fingerprint)
    stock_cp.enableBsm = 0x4B1 in fingerprint[CAN.main] and 0x4B3 in fingerprint[CAN.main]
    ret.intelligentCruiseButtonManagementAvailable = True
    return ret
