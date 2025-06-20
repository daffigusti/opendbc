import math
import numpy as np
from opendbc.car.carlog import carlog
from opendbc.can.packer import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, apply_std_steer_angle_limits, structs, AngleSteeringLimits, rate_limit
from opendbc.car.chery import cherycan
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.chery.values import DBC, CarControllerParams
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.interfaces import CarControllerBase, ISO_LATERAL_ACCEL

VisualAlert = structs.CarControl.HUDControl.VisualAlert
NetworkLocation = structs.CarParams.NetworkLocation
LongCtrlState = structs.CarControl.Actuators.LongControlState
BUTTONS_STATES = ["accelCruise", "decelCruise", "cancel", "resumeCruise"]

# Camera cancels up to 0.1s after brake is pressed, ECM allows 0.5s
CAMERA_CANCEL_DELAY_FRAMES = 10
# Enforce a minimum interval between steering messages to avoid a fault
MIN_STEER_MSG_INTERVAL_MS = 15

MAX_ANGLE_RATE = 5
# Add extra tolerance for average banked road since safety doesn't have the roll
AVERAGE_ROAD_ROLL = 0.06  # ~3.4 degrees, 6% superelevation. higher actual roll lowers lateral acceleration
MAX_LATERAL_ACCEL = ISO_LATERAL_ACCEL + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL)  # ~3.6 m/s^2
MAX_LATERAL_JERK = 3.0 + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL)  # ~3.6 m/s^3

def get_max_angle_delta(v_ego_raw: float, VM: VehicleModel):
  max_curvature_rate_sec = MAX_LATERAL_JERK / (v_ego_raw ** 2)  # (1/m)/s
  max_angle_rate_sec = math.degrees(VM.get_steer_from_curvature(max_curvature_rate_sec, v_ego_raw, 0))  # deg/s
  return max_angle_rate_sec * (DT_CTRL * CarControllerParams.STEER_STEP)


def get_max_angle(v_ego_raw: float, VM: VehicleModel):
  max_curvature = MAX_LATERAL_ACCEL / (v_ego_raw ** 2)  # 1/m
  return math.degrees(VM.get_steer_from_curvature(max_curvature, v_ego_raw, 0))  # deg

def apply_chery_steer_angle_limits(apply_angle: float, apply_angle_last: float, v_ego_raw: float, steering_angle: float,
                                     lat_active: bool, limits: AngleSteeringLimits, VM: VehicleModel, smoothing_factor, recently_overridden) -> float:
  v_ego_raw = max(v_ego_raw, 1)

  # *** max lateral jerk limit ***
  max_angle_delta = get_max_angle_delta(v_ego_raw, VM)

  # prevent fault
  max_angle_delta = min(max_angle_delta, MAX_ANGLE_RATE)
  new_apply_angle = rate_limit(apply_angle, apply_angle_last, -max_angle_delta, max_angle_delta)

  # *** max lateral accel limit ***
  max_angle = get_max_angle(v_ego_raw, VM)
  new_apply_angle = np.clip(new_apply_angle, -max_angle, max_angle)

  # angle is current angle when inactive
  if not lat_active:
    new_apply_angle = steering_angle

  # prevent fault
  return float(np.clip(new_apply_angle, -limits.STEER_ANGLE_MAX, limits.STEER_ANGLE_MAX))


def apply_chery_steer_angle_limits2(apply_angle: float, apply_angle_last: float, v_ego_raw: float, steering_angle: float,
                                     lat_active: bool, limits: AngleSteeringLimits, VM: VehicleModel, smoothing_factor, recently_overridden) -> float:
  apply_angle_last = steering_angle if recently_overridden else apply_angle_last  # Reset last angle if recently overridden
  new_angle = np.clip(apply_angle, -819.2, 819.1)
  v_ego_raw = max(v_ego_raw, 1)

  if abs(new_angle - apply_angle_last) > 0.1:  # If there's a significant difference between the new angle and the last applied angle, apply smoothing
    adjusted_alpha = np.interp(v_ego_raw, CarControllerParams.SMOOTHING_ANGLE_VEGO_MATRIX, CarControllerParams.SMOOTHING_ANGLE_ALPHA_MATRIX) + smoothing_factor
    adjusted_alpha_limited = float(min(float(adjusted_alpha), 1.))  # Limit the smoothing factor to 1 if adjusted_alpha is greater than 1
    new_angle = (new_angle * adjusted_alpha_limited) + (apply_angle_last * (1 - adjusted_alpha_limited))

  apply_angle = new_angle

  # *** max lateral jerk limit ***
  max_angle_delta = get_max_angle_delta(v_ego_raw, VM)

  # prevent fault
  max_angle_delta = min(max_angle_delta, MAX_ANGLE_RATE)
  new_apply_angle = rate_limit(apply_angle, apply_angle_last, -max_angle_delta, max_angle_delta)

  # *** max lateral accel limit ***
  max_angle = get_max_angle(v_ego_raw, VM)
  new_apply_angle = np.clip(new_apply_angle, -max_angle, max_angle)

  # angle is current angle when inactive
  if not lat_active or recently_overridden:
    new_apply_angle = steering_angle

  # prevent fault
  return float(np.clip(new_apply_angle, -limits.STEER_ANGLE_MAX, limits.STEER_ANGLE_MAX))

def get_safety_CP():
  from opendbc.car.hyundai.interface import CarInterface
  return CarInterface.get_non_essential_params("CHERY_OMODA_E5")


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.CP = CP
    self.CAN = cherycan.CanBus(CP)
    self.car_fingerprint = CP.carFingerprint
    self.packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])
    self.params = CarControllerParams(self.CP)
    self.frame = 0

      # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())

    self.start_time = 0.
    self.apply_steer_last = 0
    self.apply_angle_last = 0
    self.last_steer_frame = 0
    self.last_button_frame = 0
    self.brake_counter = 0
    self.cancel_counter = 0
    self.accel = 0.0

    self.angle_limit_counter = 0
    self.smoothing_factor = 0.6
    self.last_override_frame = 0

    self.lka_steering_cmd_counter = 0
    self.lka_steering_cmd_counter_last = -1

    self.lka_icon_status_last = (False, False)

    self.steering_pressed_counter = 0
    self.steering_unpressed_counter = 0
    self.steerDisableTemp = False

    self.prev_gas = 0
    self.prev_accel = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel
    experimentalMode = True
    # hud_control = CC.hudControl
    # hud_alert = hud_control.visualAlert
    # hud_v_cruise = hud_control.setSpeed
    recently_overridden = self.frame - self.last_override_frame < 50
    ### STEER ###
    steer_hud_alert = 1 if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw) else 0

    if CC.cruiseControl.cancel and (self.frame % self.params.BUTTONS_STEP) == 0:
      # can_sends.append(cherycan.create_button_msg(self.packer, self.CAN.camera,self.frame, CS.buttons_stock_values, cancel=True))
      print('Send Cancel')

    elif (CC.cruiseControl.resume) and (self.frame % self.params.BUTTONS_STEP) == 0:
      # can_sends.append(cherycan.create_button_msg(self.packer, self.CAN.camera, self.frame, CS.buttons_stock_values, resume=True))
      print('Send Resume')
    else:
      self.brake_counter = 0

    self.steering_pressed_counter = self.steering_pressed_counter + 1 if abs(CS.out.steeringTorque) >= 50 else 0
    # Make LKA Temporary disable when driver try to override
    if self.steering_pressed_counter * DT_CTRL > 1:
      self.steerDisableTemp = True
      self.steering_unpressed_counter = 0
    else:
      self.steering_unpressed_counter += 1
      if self.steering_unpressed_counter * DT_CTRL > 1:
        self.steerDisableTemp = False

    if CS.out.steeringPressed:  # User is overriding
        # Let's try to consider that the override is not a true or false but a progressive depending on how much torque is being applied to the col
        self.last_override_frame = self.frame

    ### lateral control ###
    # send steer msg at 50Hz
    apply_steer_req = False
    lat_active = CC.latActive and not self.steerDisableTemp
    if (self.frame  % self.params.STEER_STEP) == 0:
      apply_angle = apply_chery_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                                               CS.out.steeringAngleDeg, lat_active,
                                                               CarControllerParams.ANGLE_LIMITS, self.VM, self.smoothing_factor, recently_overridden)

      # apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg, CC.latActive, CarControllerParams.ANGLE_LIMITS)
      if lat_active:
        print(f"apply_angle: {apply_angle}")
      # apply_steer_req = CC.latActive and not CS.out.standstill
      # apply_steer_req = CC.latActive

      self.apply_angle_last = apply_angle
      self.last_steer_frame = self.frame

      can_sends.append(cherycan.create_steering_control_lkas(self.packer, self.CAN.main, apply_angle, self.frame, lat_active, CS.lkas_cmd))

    # if  (self.frame  % self.params.LKAS_HUD_STEP) == 0:
    #   can_sends.append(cherycan.create_lkas_state(self.packer, 0, self.frame, CC.latActive, CS.lkas_state))

    ### longitudinal control ###
    # send acc msg at 50Hz
    if self.CP.openpilotLongitudinalControl and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0:
      full_stop = CC.longActive and CS.out.standstill
      self.accel = int(round(np.interp(actuators.accel, self.params.ACCEL_LOOKUP_BP, self.params.ACCEL_LOOKUP_V)))
      gas = self.accel

      if gas > 0 and CS.out.standstill:
        full_stop = 0

      self.prev_gas = gas

      # full_stop = 0

      if not CC.longActive:
        gas = CarControllerParams.INACTIVE_GAS
      stopping = CC.actuators.longControlState == LongCtrlState.stopping
      if experimentalMode:
        # print(f'actuator accell {actuators.accel}, accel {self.accel}, gas {gas}, full_stop {full_stop}, CC.longActive {CC.longActive}, CS.out.standstill {CS.out.standstill}' )
        can_sends.append(cherycan.create_longitudinal_control(self.packer, self.CAN.main, CS.acc_md, self.frame, CC.longActive, gas, self.accel, stopping, full_stop))
      else:
        can_sends.append(cherycan.create_longitudinal_controlBypass(self.packer, self.CAN.main, CS.acc_md, self.frame))

    if self.frame % 20 == 0:
      # ldw = CC.hudControl.visualAlert == VisualAlert.ldw
      # steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
      can_sends.append(cherycan.create_lkas_state_hud(self.packer, self.CAN.main, self.frame, CS.lkas_state, lat_active))

    new_actuators = CC.actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    new_actuators.accel = self.accel

    self.frame += 1
    return new_actuators, can_sends
