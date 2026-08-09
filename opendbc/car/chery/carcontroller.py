from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.chery.cherycan import (CanBus, create_acc_control, create_steering_control,
                                        limit_active_steering_angle, quantize_steering_angle)
from opendbc.car.chery.values import CarControllerParams
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car import structs
from opendbc.car.vehicle_model import VehicleModel


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.CAN = CanBus(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.VM = VehicleModel(CP)
    self.apply_angle_last = None
    self.angle_command_skipped = False

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    lat_active = CC.latActive and abs(CS.out.steeringAngleDeg) <= CarControllerParams.ANGLE_LIMITS.STEER_ANGLE_MAX

    if self.frame % CarControllerParams.STEER_STEP == 0:
      if self.apply_angle_last is None:
        self.apply_angle_last = CS.out.steeringAngleDeg
      recovering = self.angle_command_skipped and abs(CS.out.steeringAngleDeg) <= 370.4
      if recovering:
        # Panda's desired-angle history must be reset by an actual inactive frame.
        apply_angle = CS.out.steeringAngleDeg
        command_active = False
      elif lat_active:
        apply_angle = apply_steer_angle_limits_vm(
          actuators.steeringAngleDeg,
          self.apply_angle_last,
          CS.front_wheel_speed,
          CS.out.steeringAngleDeg,
          True,
          CarControllerParams,
          self.VM,
        )
        command_active = True
      else:
        apply_angle = CS.out.steeringAngleDeg
        command_active = False
      wire_angle = (limit_active_steering_angle(apply_angle, self.apply_angle_last)
                    if command_active else quantize_steering_angle(apply_angle, False))
      if abs(wire_angle) <= 370.4:
        can_sends.append(create_steering_control(self.packer, self.CAN.main, wire_angle, command_active, CS.lkas_cmd))
        self.apply_angle_last = wire_angle
        self.angle_command_skipped = False
      else:
        self.angle_command_skipped = True

    if self.CP.openpilotLongitudinalControl and self.frame % CarControllerParams.ACC_CONTROL_STEP == 0:
      long_state = structs.CarControl.Actuators.LongControlState
      full_stop = actuators.longControlState == long_state.stopping
      can_sends.append(create_acc_control(
        self.packer, self.CAN.main, CS.acc_cmd, self.frame, CC.longActive,
        actuators.accel, full_stop, CC.cruiseControl.resume,
      ))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last if self.apply_angle_last is not None else CS.out.steeringAngleDeg
    self.frame += 1
    return new_actuators, can_sends
