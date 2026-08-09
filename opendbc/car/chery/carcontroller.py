from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.chery.cherycan import CanBus, create_steering_control
from opendbc.car.chery.values import CarControllerParams
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.vehicle_model import VehicleModel


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.CAN = CanBus(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.VM = VehicleModel(CP)
    self.apply_angle_last = None

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    lat_active = CC.latActive and abs(CS.out.steeringAngleDeg) <= CarControllerParams.ANGLE_LIMITS.STEER_ANGLE_MAX

    if self.frame % CarControllerParams.STEER_STEP == 0:
      if self.apply_angle_last is None:
        self.apply_angle_last = CS.out.steeringAngleDeg
      if lat_active:
        apply_angle = apply_steer_angle_limits_vm(
          actuators.steeringAngleDeg,
          self.apply_angle_last,
          CS.front_wheel_speed,
          CS.out.steeringAngleDeg,
          True,
          CarControllerParams,
          self.VM,
        )
      else:
        apply_angle = CS.out.steeringAngleDeg
      self.apply_angle_last = apply_angle
      if abs(apply_angle) <= 370.4:
        can_sends.append(create_steering_control(self.packer, self.CAN.main, apply_angle, lat_active, CS.lkas_cmd))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last if self.apply_angle_last is not None else CS.out.steeringAngleDeg
    self.frame += 1
    return new_actuators, can_sends
