#!/usr/bin/env python3
from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.chery.carcontroller import CarController
from opendbc.car.chery.carstate import CarState
from opendbc.car.chery.values import CarControllerParams, CheryFlags, CherySafetyFlags, DBC
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.chery.cherycan import CanBus

ButtonType = structs.CarState.ButtonEvent.Type
TransmissionType = structs.CarParams.TransmissionType
GearShifter = structs.CarState.GearShifter

CRUISE_OVERRIDE_SPEED_MIN = 5 * CV.KPH_TO_MS

class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "chery"

    CAN = CanBus(fingerprint=fingerprint)
    cfgs = [get_safety_config(structs.CarParams.SafetyModel.cheryCanFd)]
    if CAN.main >= 4:
      cfgs.insert(0, get_safety_config(structs.CarParams.SafetyModel.elm327))
    ret.safetyConfigs = cfgs

    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.safetyConfigs[-1].safetyParam |= CherySafetyFlags.LONG_CONTROL.value
      ret.openpilotLongitudinalControl = True

    # ret.wheelbase = 2.63
    # ret.tireStiffnessFactor = 0.8
    ret.centerToFront = ret.wheelbase * 0.44

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.25
    ret.steerControlType = structs.CarParams.SteerControlType.angle

    ret.transmissionType = TransmissionType.automatic

    ret.stopAccel = CarControllerParams.ACCEL_MIN
    ret.stoppingDecelRate = 0.3
    ret.vEgoStarting = 0.1
    ret.vEgoStopping = 0.1
    ret.longitudinalActuatorDelay = 0.05 # s
    # ret.startAccel = 1.0

    # ret.longitudinalTuning.kpV = [0.0]
    # ret.longitudinalTuning.kiV = [0.5]
    ret.longitudinalTuning.kiBP = [0., 5., 35.]
    # ret.longitudinalTuning.kiV = [0.6, 0.5, 0.3]
    ret.longitudinalTuning.kiV = [0.5, 0.4, 0.2]
    # ret.longitudinalTuning.kiV = [0.5, 0.4, 0.15]

    ret.enableBsm = 0x4B1 in fingerprint[CAN.main] and 0x4B3 in fingerprint[CAN.main]

    ret.minEnableSpeed = -1
    ret.minSteerSpeed = -1
    ret.autoResumeSng = ret.minEnableSpeed == -1.

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, docs: bool) -> structs.CarParamsSP:

    stock_cp.enableBsm = True

    return ret
