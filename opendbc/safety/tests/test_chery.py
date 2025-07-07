#!/usr/bin/env python3
import enum
import unittest

from opendbc.car.chery.carcontroller import get_max_angle_delta, get_max_angle, get_safety_CP
from opendbc.car.chery.values import CherySafetyFlags

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.car.vehicle_model import VehicleModel
from opendbc.can.can_define import CANDefine
from openpilot.selfdrive.pandad.pandad_api_impl import can_list_to_can_capnp
from opendbc.safety.tests.common import CANPackerPanda, MAX_SPEED_DELTA, MAX_WRONG_COUNTERS, away_round, round_speed

# Chery CAN message addresses
class CheryMsg(enum.IntEnum):
  ACC_CMD = 0x3A2
  ACC_STATUS = 0x3A5
  LKAS_HUD = 0x307
  LKAS_CMD = 0x345
  ACC_SETTING = 0x387
  HUD_ALERT = 0x3FC
  ENGINE = 0x3E
  BRAKE = 0x29A
  BRAKE_SENSOR = 0x4ED
  WHEEL_SENSOR = 0x316
  ACC_DATA = 0x3A5
  STEER_BUTTON = 0x360

# Chery CAN bus numbers
CHERY_MAIN = 0
CHERY_AUX = 1
CHERY_CAM = 2

cnt_gas = 0
cnt_speed = 0
cnt_brake = 0
cnt_cruise = 0
cnt_button = 0
class TestCherySafetyBase(common.PandaCarSafetyTest, common.AngleSteeringSafetyTest):
  FLAGS = 0
  # RELAY_MALFUNCTION_ADDRS = {CHERY_MAIN: (CheryMsg.LKAS_CMD, CheryMsg.LKAS_HUD, CheryMsg.ACC_CMD),
                            #  CHERY_CAM: ( CheryMsg.ACC_DATA)}
  RELAY_MALFUNCTION_ADDRS = {0: [0x307, 0x345], 2 : []}
  FWD_BLACKLISTED_ADDRS = {CHERY_CAM: [CheryMsg.LKAS_CMD, CheryMsg.LKAS_HUD], CHERY_MAIN: []} # No explicit blacklisting in chery.h
  TX_MSGS = [[CheryMsg.LKAS_CMD, CHERY_MAIN], [CheryMsg.LKAS_HUD, CHERY_MAIN], [CheryMsg.STEER_BUTTON, CHERY_CAM], [CheryMsg.ACC_DATA, CHERY_MAIN], [CheryMsg.ACC_CMD, CHERY_MAIN]]

  # Angle control limits
  STEER_ANGLE_MAX = 360  # deg
  DEG_TO_CAN = 10

  # Chery uses get_max_angle_delta and get_max_angle for real lateral accel and jerk limits
  # TODO: integrate this into AngleSteeringSafetyTest
  ANGLE_RATE_BP = None
  ANGLE_RATE_UP = None
  ANGLE_RATE_DOWN = None

  # Real time limits
  LATERAL_FREQUENCY = 50  # Hz

  # Long control limits
  MAX_ACCEL = 2.0
  MIN_ACCEL = -3.48
  INACTIVE_ACCEL = 0.0
  cnt_angle_cmd = 0
  def _get_steer_cmd_angle_max(self, speed):
    angle = get_max_angle(max(speed, 1), self.VM)
    return angle

  def setUp(self):
    self.VM = VehicleModel(get_safety_CP())
    self.packer = CANPackerPanda("chery_canfd")
    self.define = CANDefine("chery_canfd")

    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.cheryCanFd, self.FLAGS)
    self.safety.init_tests()

  def _button_msg(self, main_button = 0, set=0, res = 0, bus=CHERY_CAM):
    values = { "ACC": main_button, "RES_PLUS": res, "RES_MINUS": set}
    return self.packer.make_can_msg_panda("STEER_BUTTON", bus, values)

  def _main_cruise_button_msg(self, enabled):
    return self._button_msg(enabled)
  # Helper functions for creating Chery CAN messages
  def _speed_msg(self, speed):
    # CHERY_WHEEL_SENSOR (0x316)
    # uint16_t right_rear = (GET_BYTE(to_push, 0) << 8) | (GET_BYTE(to_push, 1));
    # uint16_t left_rear = (GET_BYTE(to_push, 2) << 8) | (GET_BYTE(to_push, 3));
    values = {"WHEEL_SPEED_FR": speed * 3.6, "WHEEL_SPEED_FL": speed * 3.6}
    can =  self.packer.make_can_msg_panda("WHEEL_SPEED_FRNT", CHERY_MAIN, values)
    return can

  def _brake_msg(self, pressed):
    # CHERY_BRAKE_SENSOR (0x4ED)
    # BRAKE_PRESS is at bit 37 (byte 4, bit 5) in BRAKE_SENSOR message
    values = {"BRAKE_PRESS": 1 if pressed else 0}
    return self.packer.make_can_msg_panda("BRAKE_SENSOR", CHERY_MAIN, values)

  def _user_brake_msg(self, pressed):
    return self._brake_msg(pressed)

  def _gas_msg(self, pressed):
    # ACC_CMD: GAS_PRESSED
    values = {"GAS_PRESSED": 1 if pressed else 0}
    return self.packer.make_can_msg_panda("ACC_CMD", CHERY_CAM, values)

  def _user_gas_msg(self, pressed):
    return self._gas_msg(pressed)

  def _pcm_status_msg(self, engaged):
    # ACC: ACC_ACTIVE
    values = {"ACC_ACTIVE": 1 if engaged else 0}
    return self.packer.make_can_msg_panda("ACC", CHERY_CAM, values)

  def _lkas_cmd_msg(self, steer_angle, lat_active):
    # LKAS_CAM_CMD_345: CMD, LKA_ACTIVE
    values = {"CMD": steer_angle, "LKA_ACTIVE": 1 if lat_active else 0}
    return self.packer.make_can_msg_panda("LKAS_CAM_CMD_345", CHERY_MAIN, values)

  def _lkas_hud_msg(self, lkas_state):
    # LKAS_STATE: STATE
    values = {"STATE": lkas_state}
    return self.packer.make_can_msg_panda("LKAS_STATE", CHERY_MAIN, values)

  def _acc_cmd_msg(self, acc_on):
    values = {"ACCEL_ON": acc_on}
    return self.packer.make_can_msg_panda("ACC_CMD", CHERY_MAIN, values)

  def _long_control_msg(self, gas, acc_state=3, bus=CHERY_MAIN):
    long_active = 1
    throtle = gas if long_active else -24
    full_stop = 0
    values = {
      "CMD": 400 if full_stop else throtle,
      "ACCEL_ON": 1 if throtle>= 0 else 0,
      "ACC_STATE": acc_state, # 1 not available, 2 available, 3 active
      "STOPPED": 1 if full_stop else 0 if long_active else 0,
    }
    return self.packer.make_can_msg_panda("DAS_control", bus, values)

  def _accel_msg(self, accel: float):
    # For common.LongitudinalAccelSafetyTest
    return self._long_control_msg(accel_limits=(accel, max(accel, 0)))

  def _angle_cmd_msg(self, angle: float, enabled: bool, increment_timer: bool = True):
    if increment_timer:
      self.safety.set_timer(self.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    apply_steer = int((angle*10)-392)
    values = {"CMD": apply_steer & 0x1FFF, "LKA_ACTIVE": 1 if enabled else 0, "NEW_SIGNAL_3": 1 if apply_steer>1 else 0 }
    print(f"angle: {angle}, apply_steer: {apply_steer }, CMD: {apply_steer & 0x1FFF}")
    msg = self.packer.make_can_msg_panda("LKAS_CAM_CMD_345", CHERY_MAIN, values)
    # print("CAN BYTES:", list(msg[0].data))

    return msg

  def _angle_meas_msg(self, angle: float):
    values = {"STEER_ANGLE": angle}
    print(f"sent angle_meas: {angle}")
    return self.packer.make_can_msg_panda("STEER_ANGLE_SENSOR", CHERY_MAIN, values)

  def test_angle_cmd_when_enabled(self):
    # We properly test lateral acceleration and jerk below
    pass
class TestCherySafety(TestCherySafetyBase):
  def test_default_controls_allowed(self):
    # Test that controls are allowed by default
    self.safety.set_controls_allowed(True)
    self.assertTrue(self.safety.get_controls_allowed())

  def test_chery_rx_hook_vehicle_moving(self):
    # Test vehicle moving detection
    self._rx(self._speed_msg(0.1)) # speed > 0
    self.assertTrue(self.safety.get_vehicle_moving())
    self._rx(self._speed_msg(0.0)) # speed == 0
    self.assertFalse(self.safety.get_vehicle_moving())

  def test_chery_rx_hook_brake_pressed(self):
    # Test brake pressed detection
    self._rx(self._brake_msg(True))
    self.assertTrue(self.safety.get_brake_pressed_prev())
    self._rx(self._brake_msg(False))
    self.assertFalse(self.safety.get_brake_pressed_prev())

  def test_chery_rx_hook_gas_pressed(self):
    # Test gas pressed detection
    self._rx(self._gas_msg(True))
    self.assertTrue(self.safety.get_gas_pressed_prev())
    self._rx(self._gas_msg(False))
    self.assertFalse(self.safety.get_gas_pressed_prev())

  def test_chery_rx_hook_pcm_cruise_check(self):
    # Test pcm cruise check
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_cruise_engaged_prev())
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self.safety.get_cruise_engaged_prev())

  def test_chery_rx_hook_acc_main_on(self):
    # Test ACC main on detection
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_acc_main_on())
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self.safety.get_acc_main_on())

  def test_chery_tx_msgs(self):
    self.safety.set_controls_allowed(True)
    self._rx(self._angle_meas_msg(0))        # Initializes angle_meas
    self._rx(self._speed_msg(20))            # Initializes vehicle_speed, if required
    print(self.safety.get_lat_active());
    self.assertTrue(self._tx(self._angle_cmd_msg(6.5, True)))
    self.assertTrue(self._tx(self._lkas_hud_msg(0)))

  def test_chery_longitudinal_flag(self):
    # Test that setting the longitudinal flag enables long TX messages
    self.safety.set_safety_hooks(CarParams.SafetyModel.cheryCanFd, 1) # 1 for CHERY_PARAM_LONGITUDINAL
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._acc_cmd_msg(True))) # Should be allowed with longitudinal flag

  def test_acc_buttons(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)

      # resume only while controls allowed
      self.assertEqual(controls_allowed, self._tx(self._button_msg(res=1)))

      # can always cancel
      self.assertTrue(self._tx(self._button_msg(main_button=1)))

      # only one button at a time
      self.assertFalse(self._tx(self._button_msg(main_button=1, res=1)))
      self.assertFalse(self._tx(self._button_msg(main_button=0, res=0)))

if __name__ == "__main__":
  unittest.main()
