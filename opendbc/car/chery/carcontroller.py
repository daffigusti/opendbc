from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL
from opendbc.car.chery.cherycan import (CanBus, create_acc_control, create_button_control,
                                        create_lkas_state_hud, create_steering_control,
                                        limit_active_steering_angle, quantize_steering_angle)
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
    self.angle_command_skipped = False
    self.lkas_active_last = False
    self.resume_counter = 0
    self.steer_pressed_frames = 0
    self.steer_released_frames = 0
    self.steer_override = False

  def _update_steer_override(self, CS):
    """Hand steering back to the driver while they hold the wheel, and take it back once they let go.

    The EPS has no torque-override path of its own here, so the only way a driver wins an
    argument with LKAS is for openpilot to stop commanding.
    """
    if abs(CS.out.steeringTorque) >= CarControllerParams.STEER_THRESHOLD:
      self.steer_pressed_frames += 1
      self.steer_released_frames = 0
    else:
      self.steer_pressed_frames = 0
      self.steer_released_frames += 1

    if self.steer_pressed_frames * DT_CTRL > CarControllerParams.STEER_OVERRIDE_TIME:
      self.steer_override = True
    elif self.steer_released_frames * DT_CTRL > CarControllerParams.STEER_OVERRIDE_TIME:
      self.steer_override = False

  def _update_resume(self, CC, CS, can_sends):
    """Tap RES+ to get out of the stock ACC's standstill hold.

    Once the hold drops ACC_ACTIVE the ACC ignores ACC_CMD gas entirely and only a button press
    brings it back. RES+ also means "raise set speed" while ACC_ACTIVE is 1, so this re-checks
    the freshest CarState rather than trusting cruiseControl.resume, which was computed from the
    previous one: a single stale frame would land the tap in the raise-set-speed window.
    """
    if not CC.cruiseControl.resume:
      self.resume_counter = 0
      return
    if self.frame % CarControllerParams.BUTTONS_STEP != 0:
      return
    if CS.acc_active:
      self.resume_counter = 0
      return
    if self.resume_counter % CarControllerParams.RESUME_TAP_PERIOD < CarControllerParams.RESUME_TAP_FRAMES:
      can_sends.append(create_button_control(self.packer, self.CAN.camera, self.frame,
                                             CS.buttons_stock_values, resume=True))
    self.resume_counter += 1

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    self._update_steer_override(CS)
    lat_active = (CC.latActive and not self.steer_override and
                  abs(CS.out.steeringAngleDeg) <= CarControllerParams.ANGLE_LIMITS.STEER_ANGLE_MAX)

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
        self.lkas_active_last = command_active
      else:
        self.angle_command_skipped = True
        self.lkas_active_last = False

    # Stock 0x307 is blocked from forwarding, so the cluster only sees lane-keep state if this
    # relays it -- openpilot's while steering, the camera's verbatim otherwise.
    if self.frame % CarControllerParams.LKAS_HUD_STEP == 0:
      can_sends.append(create_lkas_state_hud(self.packer, self.CAN.main, CS.lkas_state, self.lkas_active_last))

    self._update_resume(CC, CS, can_sends)

    if self.CP.openpilotLongitudinalControl and self.frame % CarControllerParams.ACC_CONTROL_STEP == 0:
      # CMD=400 is the OEM's standstill brake, so it may only go out once the car has actually
      # stopped and the plan still wants deceleration. Sent while rolling it is a full-force
      # brake application; sent while the plan wants to launch it fights the pull-away.
      full_stop = CC.longActive and CS.out.standstill and actuators.accel <= 0
      can_sends.append(create_acc_control(
        self.packer, self.CAN.main, CS.acc_cmd, CC.longActive,
        actuators.accel, full_stop, CC.cruiseControl.resume,
      ))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last if self.apply_angle_last is not None else CS.out.steeringAngleDeg
    self.frame += 1
    return new_actuators, can_sends
