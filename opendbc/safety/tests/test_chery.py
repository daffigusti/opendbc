import math

from opendbc.safety.tests.common import CANPackerSafety, MAX_WRONG_COUNTERS, SafetyTest, make_msg
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.can import CANPacker
from opendbc.car.chery.cherycan import calculate_crc
from opendbc.car.chery.values import CAR, CarControllerParams
from opendbc.car.chery.interface import CarInterface
from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.vehicle_model import VehicleModel


SAFETY_CHERY = 35
RX_LAYOUT = {
  0x03E: (0, 48, 100),
  0x1D3: (0, 8, 100),
  0x316: (0, 8, 50),
  0x394: (0, 8, 50),
  0x3A2: (2, 8, 50),
  0x3A5: (2, 8, 50),
}

GOLDEN_FRAMES = {
  (0x03E, 0): bytes.fromhex("bc0680007ccd80006c0680647869741d0d067cf215040000ab0644d000002000ab067fd9400000000000000000000000"),
  (0x1D3, 0): bytes.fromhex("78c0010000000523"),
  (0x316, 0): bytes.fromhex("057e057e8c115efe"),
  (0x394, 0): bytes.fromhex("17b000000800038a"),
  (0x3A2, 2): bytes.fromhex("7d1102027f710f57"),
  (0x3A5, 2): bytes.fromhex("0000000000000fb1"),
}

def _j1850(data):
  crc = 0xFF
  for byte in data:
    crc ^= byte
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc ^ 0xFF


def _checksum(address, data):
  if address == 0x316:
    return (0xFF - sum(data[:7])) & 0xFF
  return _j1850(data[:7])


class TestCherySafety(SafetyTest):
  TX_MSGS = [[0x345, 0]]
  FWD_BUS_LOOKUP = {0: 2, 2: 0}
  FWD_BLACKLISTED_ADDRS = {2: [0x345]}
  RELAY_MALFUNCTION_ADDRS = {0: (0x345,)}

  @classmethod
  def setUpClass(cls):
    cls.safety = libsafety_py.libsafety

  def setUp(self):
    self.assertEqual(self.safety.set_safety_hooks(SAFETY_CHERY, 0), 0)
    self.safety.init_tests()
    self._counters = {address: 0 for address in RX_LAYOUT}
    self._timer = 0
    self.packer = CANPackerSafety("chery_canfd")

  def _angle_cmd_msg(self, angle, enabled, bus=0, length=8, timer=True):
    # LKAS_CAM_CMD_345 CMD is signed 13-bit in raw CAN units. Keeping this
    # conversion here makes boundary tests independent of controller code.
    address, data, _ = self.packer.make_can_msg("LKAS_CAM_CMD_345", bus, {
      "CMD": round(angle * 10 - 392), "LKA_ACTIVE": int(enabled),
    })
    if timer:
      self._timer += 20_000
      self.safety.set_timer(self._timer)
    payload = data[:length] if length <= 8 else data + bytes(length - 8)
    return libsafety_py.make_CANPacket(address, bus, payload)

  def _angle_meas_msg(self, angle, bus=0, length=8):
    return self._packet(0x1D3, bus, angle_raw=round((angle + 780) * 10)) if length == 8 else \
      libsafety_py.make_CANPacket(0x1D3, bus, GOLDEN_FRAMES[(0x1D3, 0)][:length])

  def _acc_cmd_msg(self, command, aeb_req_stop=0, accel_on=None, gas_pressed=0):
    packer = CANPacker("chery_canfd")
    if accel_on is None:
      accel_on = command >= 0
    address, data, bus = packer.make_can_msg("ACC_CMD", 0, {
      "CMD": command, "ACCEL_ON": accel_on, "GAS_PRESSED": gas_pressed,
      "AEB_REQ_STOP": aeb_req_stop,
    })
    data = bytearray(data)
    data[-1] = calculate_crc(bytes(data[:-1]))
    return libsafety_py.make_CANPacket(address, bus, data)

  def _enable_longitudinal(self):
    self.assertEqual(self.safety.set_safety_hooks(SAFETY_CHERY, 1), 0)
    self.safety.init_tests()

  def _reset_angle_samples(self, angle):
    for _ in range(6):
      self._rx(self._angle_meas_msg(angle))

  def _reset_speed_samples(self, speed):
    raw = round(speed * 3.6 / 0.00829)
    for _ in range(6):
      self._rx(self._packet(0x316, 0, fr=raw, fl=raw))

  def _golden(self, address, bus):
    return libsafety_py.make_CANPacket(address, bus, GOLDEN_FRAMES[(address, bus)])

  def _seed_all(self):
    for address, (bus, _dlc, _frequency) in RX_LAYOUT.items():
      self._rx(self._packet(address, bus))

  def _seed_non_acc(self):
    for address, (bus, _dlc, _frequency) in RX_LAYOUT.items():
      if address not in (0x3A2, 0x3A5):
        self._rx(self._packet(address, bus))

  def _validate_config(self):
    self.safety.safety_tick_current_safety_config()
    self.assertTrue(self.safety.safety_config_valid())

  def _packet(self, address, bus, counter=None, **fields):
    """Clone golden route data, mutate decoded raw fields, and repair integrity."""
    data = bytearray(GOLDEN_FRAMES[(address, bus)])
    if counter is None:
      counter = self._counters[address]
    self._counters[address] = (counter + 1) & 0xF
    if address == 0x03E:
      for offset in range(0, 40, 8):
        data[offset + 1] = (data[offset + 1] & 0xF0) | counter
      data[27] = (data[27] & ~(1 << 4)) | (int(fields.get("brake", 0)) << 4)
      data[22:24] = int(fields.get("engine_gas", 0)).to_bytes(2, "big")
      for offset in range(0, 40, 8):
        data[offset] = _j1850(data[offset + 1:offset + 8])
    elif address == 0x316:
      for key, index in (("fr", 0), ("fl", 2)):
        if key in fields:
          data[index:index + 2] = (int(fields[key]) & 0xFFFF).to_bytes(2, "big")
      data[6] = (data[6] & 0xF0) | counter
      data[7] = _checksum(address, data)
    elif address == 0x1D3:
      if "angle_raw" in fields:
        raw = int(fields["angle_raw"]) & 0x3FFF
        data[0] = raw >> 6
        data[1] = (data[1] & 0x03) | ((raw & 0x3F) << 2)
      data[6] = (data[6] & 0xF0) | counter
      data[7] = _checksum(address, data)
    elif address == 0x394:
      if "torque_raw" in fields:
        raw = int(fields["torque_raw"]) & 0xFFF
        data[0] = raw >> 4
        data[1] = (data[1] & 0x0F) | ((raw & 0xF) << 4)
      data[6] = (data[6] & 0xF0) | counter
      data[7] = _checksum(address, data)
    elif address == 0x3A2:
      if "state" in fields:
        data[1] = (data[1] & 0xFC) | int(fields["state"])
      if "acc_gas" in fields:
        data[5] = (data[5] & 0x7F) | (int(fields["acc_gas"]) << 7)
      data[6] = (data[6] & 0xF0) | counter
      data[7] = _checksum(address, data)
    elif address == 0x3A5:
      if "active" in fields:
        data[2] = (data[2] & ~(1 << 4)) | (int(fields["active"]) << 4)
      if "aeb" in fields:
        data[5] = (data[5] & ~(1 << 6)) | (int(fields["aeb"]) << 6)
      data[6] = (data[6] & 0xF0) | counter
      data[7] = _checksum(address, data)
    return libsafety_py.make_CANPacket(address, bus, data)

  def _rx_field(self, address, **fields):
    return self._rx(self._packet(address, RX_LAYOUT[address][0], **fields))

  def _engage(self):
    self._seed_all()
    self._validate_config()
    self._rx_field(0x3A2, state=2)
    self._rx_field(0x3A5, active=1)
    self.assertTrue(self.safety.get_controls_allowed())

  def _disengage(self):
    self._rx_field(0x3A5, active=0)
    self.assertFalse(self.safety.get_controls_allowed())

  def _recover_rx_without_acc_off(self):
    self._rx_field(0x3A2, state=2)
    self._rx_field(0x3A5, active=1)
    self._seed_non_acc()
    self._validate_config()

  def test_registration_and_exact_rx_layout(self):
    self.assertEqual(self.safety.get_current_safety_mode(), SAFETY_CHERY)
    self.assertEqual(self.safety.get_current_safety_rx_checks_len(), 6)
    for index, (address, (bus, dlc, frequency)) in enumerate(RX_LAYOUT.items()):
      self.assertEqual(self.safety.get_rx_check_addr(index), address)
      self.assertEqual(self.safety.get_rx_check_bus(index), bus)
      self.assertEqual(self.safety.get_rx_check_len(index), dlc)
      self.assertEqual(self.safety.get_rx_check_frequency(index), frequency)
      self.assertEqual(self.safety.get_rx_check_max_counter(index), 15)
      self.assertEqual(self.safety.get_rx_check_ignore_quality(index), address != 0x03E)
      self.assertFalse(self.safety.get_rx_check_ignore_checksum(index))
      self.assertFalse(self.safety.get_rx_check_ignore_counter(index))

  def test_golden_frames_validate_and_config_fails_closed_on_timeout(self):
    self._seed_all()
    self.assertTrue(self.safety.safety_config_valid())
    self.safety.set_timer(2_000_001)
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.safety_config_valid())

  def test_omitted_message_fails_while_other_messages_refresh(self):
    self._seed_all()
    self.assertTrue(self.safety.safety_config_valid())
    omitted = 0x3A5
    self.safety.set_timer(1_000_001)
    for address, (bus, _dlc, _frequency) in RX_LAYOUT.items():
      if address != omitted:
        self._rx(self._golden(address, bus))
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.safety_config_valid())

  def test_checksum_corruption_invalidates_each_message(self):
    for address, (bus, _dlc, _frequency) in RX_LAYOUT.items():
      self.setUp()
      packet = self._golden(address, bus)
      packet.data[24 if address == 0x03E else 7] ^= 1
      self.assertFalse(self._rx(packet))

  def test_invalid_integrity_revokes_longitudinal_and_mads_lateral_controls(self):
    self._seed_all()
    self.safety.set_controls_allowed(True)
    self.safety.set_controls_allowed_lateral(True)
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_controls_allowed_lateral())

    packet = self._golden(0x3A5, 2)
    packet.data[7] ^= 1
    self.assertFalse(self._rx(packet))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_each_engine_data_integrity_island_is_required(self):
    for island in range(5):
      self.setUp()
      packet = self._golden(0x03E, 0)
      packet.data[island * 8 + 1] ^= 1
      packet.data[island * 8] = _j1850(packet.data[island * 8 + 1:island * 8 + 8])
      self.assertFalse(self._rx(packet))

  def test_wrong_bus_or_dlc_does_not_match(self):
    for address, (bus, dlc, _frequency) in RX_LAYOUT.items():
      self.setUp()
      for other_address, (other_bus, _other_dlc, _other_frequency) in RX_LAYOUT.items():
        if other_address != address:
          self._rx(self._golden(other_address, other_bus))
      wrong_bus = 2 if bus == 0 else 0
      self._rx(libsafety_py.make_CANPacket(address, wrong_bus, GOLDEN_FRAMES[(address, bus)]))
      self.assertFalse(self.safety.safety_config_valid())

      self.setUp()
      for other_address, (other_bus, _other_dlc, _other_frequency) in RX_LAYOUT.items():
        if other_address != address:
          self._rx(self._golden(other_address, other_bus))
      wrong_dlc = 64 if dlc == 48 else 12
      malformed = GOLDEN_FRAMES[(address, bus)] + bytes(wrong_dlc - dlc)
      self._rx(libsafety_py.make_CANPacket(address, bus, malformed))
      self.assertFalse(self.safety.safety_config_valid())

      self.setUp()
      for other_address, (other_bus, _other_dlc, _other_frequency) in RX_LAYOUT.items():
        if other_address != address:
          self._rx(self._golden(other_address, other_bus))
      self._rx(self._golden(address, bus))
      self.assertTrue(self.safety.safety_config_valid())

  def test_rx_metadata_getters_are_bounds_safe(self):
    invalid = (-1, self.safety.get_current_safety_rx_checks_len())
    for index in invalid:
      for getter in (self.safety.get_rx_check_addr, self.safety.get_rx_check_bus,
                     self.safety.get_rx_check_len, self.safety.get_rx_check_frequency,
                     self.safety.get_rx_check_max_counter):
        self.assertEqual(getter(index), -1)
      for getter in (self.safety.get_rx_check_ignore_quality, self.safety.get_rx_check_ignore_checksum,
                     self.safety.get_rx_check_ignore_counter):
        self.assertFalse(getter(index))

  def test_counter_progression_tolerance_and_eventual_invalidation(self):
    for address, (bus, _dlc, _frequency) in RX_LAYOUT.items():
      self.setUp()
      for other_address, (other_bus, _other_dlc, _other_frequency) in RX_LAYOUT.items():
        if other_address != address:
          self._rx(self._golden(other_address, other_bus))

      # Counters 14 -> 15 -> 0 are valid after preceding 1..13 frames.
      for counter in list(range(1, 16)) + [0]:
        data = bytearray(GOLDEN_FRAMES[(address, bus)])
        if address == 0x03E:
          for offset in range(0, 40, 8):
            data[offset + 1] = (data[offset + 1] & 0xF0) | counter
            data[offset] = _j1850(data[offset + 1:offset + 8])
        else:
          data[6] = (data[6] & 0xF0) | counter
          data[7] = _checksum(address, data)
        self.assertTrue(self._rx(libsafety_py.make_CANPacket(address, bus, data)))

      # One repeated and one skipped counter are tolerated; repeated values eventually fail.
      self.setUp()
      for other_address, (other_bus, _other_dlc, _other_frequency) in RX_LAYOUT.items():
        if other_address != address:
          self._rx(self._golden(other_address, other_bus))
      for counter in range(1, 7):
        data = bytearray(GOLDEN_FRAMES[(address, bus)])
        if address == 0x03E:
          for offset in range(0, 40, 8):
            data[offset + 1] = (data[offset + 1] & 0xF0) | counter
            data[offset] = _j1850(data[offset + 1:offset + 8])
        else:
          data[6] = (data[6] & 0xF0) | counter
          data[7] = _checksum(address, data)
        self.assertTrue(self._rx(libsafety_py.make_CANPacket(address, bus, data)))
      repeated = bytearray(GOLDEN_FRAMES[(address, bus)])
      if address == 0x03E:
        for offset in range(0, 40, 8):
          repeated[offset + 1] = (repeated[offset + 1] & 0xF0) | 8
          repeated[offset] = _j1850(repeated[offset + 1:offset + 8])
      else:
        repeated[6] = (repeated[6] & 0xF0) | 8
        repeated[7] = _checksum(address, repeated)
      repeated_msg = libsafety_py.make_CANPacket(address, bus, repeated)
      self.assertTrue(self._rx(repeated_msg), hex(address))
      for _ in range(MAX_WRONG_COUNTERS - 2):
        self.assertTrue(self._rx(repeated_msg), hex(address))
      self.assertFalse(self._rx(repeated_msg), hex(address))

  def test_tx_denied_and_forwarding_disabled(self):
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      for bus in range(4):
        self.assertFalse(self.safety.safety_tx_hook(make_msg(bus, 0x360, 6)))
        self.assertFalse(self.safety.safety_tx_hook(make_msg(bus, 0x360, 8)))
    self.assertEqual(self.safety.safety_fwd_hook(0, 0x345), 2)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x345), -1)
    self.assertEqual(self.safety.safety_fwd_hook(0, 0x360), 2)
    self.assertEqual(self.safety.safety_fwd_hook(0, 0x3A2), 2)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x3A2), 0)
    self.assertEqual(self.safety.safety_fwd_hook(1, 0x345), -1)
    self.assertEqual(self.safety.safety_fwd_hook(3, 0x345), -1)

  def test_longitudinal_base_flag_and_raw_command_contract(self):
    self.assertFalse(self._tx(self._acc_cmd_msg(-24)))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x3A2), 0)
    self._enable_longitudinal()
    self.safety.set_controls_allowed(True)
    for command in (-511, 0, 511):
      self.assertTrue(self._tx(self._acc_cmd_msg(command)), command)
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._acc_cmd_msg(-24)))
    self.assertFalse(self._tx(self._acc_cmd_msg(-511)))
    self.assertFalse(self._tx(self._acc_cmd_msg(511)))
    self.assertFalse(self._tx(self._acc_cmd_msg(-512)))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x3A2), -1)

  def test_longitudinal_accel_on_and_gas_pressed_contract(self):
    self._enable_longitudinal()
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._acc_cmd_msg(511, accel_on=0)))
    self.assertFalse(self._tx(self._acc_cmd_msg(-24, accel_on=1)))
    self.assertTrue(self._tx(self._acc_cmd_msg(-24, gas_pressed=1)))
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._acc_cmd_msg(-24, gas_pressed=1)))

  def test_longitudinal_aeb_and_rx_inhibitors_allow_only_inactive_command(self):
    self._enable_longitudinal()
    for field, address in (("brake", 0x03E), ("engine_gas", 0x03E),
                           ("acc_gas", 0x3A2), ("aeb", 0x3A5)):
      self.setUp()
      self._enable_longitudinal()
      value = 11 if field == "engine_gas" else 1
      self._rx_field(address, **{field: value})
      self.safety.set_controls_allowed(True)
      self.assertFalse(self._tx(self._acc_cmd_msg(511)), field)
      inactive_allowed = field != "aeb"
      self.assertEqual(inactive_allowed, self._tx(self._acc_cmd_msg(-24)), field)
    self.setUp()
    self._enable_longitudinal()
    self.safety.set_controls_allowed(True)
    for aeb_req_stop in range(1, 16):
      self.assertFalse(self._tx(self._acc_cmd_msg(-24, aeb_req_stop)))

  def test_stock_aeb_rejects_all_host_acc_and_restores_oem_forwarding(self):
    self._enable_longitudinal()
    self._rx_field(0x3A5, aeb=1, active=1)
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._acc_cmd_msg(-24)))
    self.assertFalse(self._tx(self._acc_cmd_msg(511)))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x3A2), 0)
    self._rx_field(0x3A5, aeb=0, active=0)
    self._rx_field(0x3A2, state=2)
    self.assertTrue(self._tx(self._acc_cmd_msg(-24)))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x3A2), -1)

  def test_angle_command_signed13_boundaries_and_message_shape(self):
    self.safety.set_controls_allowed(True)
    for angle in (-150.1, -150.0, 150.0, 150.1):
      raw = round(angle * 10 - 392)
      self.safety.set_desired_angle_last(round(angle * 100))
      allowed = abs(angle) <= 150.0
      self.assertEqual(allowed, self._tx(self._angle_cmd_msg(angle, True)))

    # Sign bit and positive/negative edge encodings must not alias.
    for raw in (0x0FFF, 0x1000, 0x1FFF):
      self.setUp()
      for _ in range(6):
        self._rx(self._packet(0x1D3, 0, angle_raw=raw))
      expected = raw * 10 - 78000
      self.assertEqual(self.safety.get_angle_meas_min(), expected)
      self.assertEqual(self.safety.get_angle_meas_max(), expected)

    self.assertFalse(self._tx(self._angle_cmd_msg(0, False, bus=2)))
    self.assertFalse(self._tx(self._angle_cmd_msg(0, False, length=7)))

  def test_active_desired_angle_delta_accepts_five_rejects_five_point_one(self):
    self._reset_speed_samples(1)
    self.safety.set_controls_allowed(True)
    for angle, allowed in ((5.0, True), (5.1, False), (-5.0, True), (-5.1, False)):
      self.safety.set_desired_angle_last(0)
      self.assertEqual(allowed, self._tx(self._angle_cmd_msg(angle, True)))

  def test_active_measured_angle_accepts_150_rejects_150_point_one(self):
    self.safety.set_controls_allowed(True)
    for angle, allowed in ((150.0, True), (150.1, False), (-150.0, True), (-150.1, False)):
      self._reset_angle_samples(angle)
      self.safety.set_desired_angle_last(round(angle * 100))
      self.assertEqual(allowed, self._tx(self._angle_cmd_msg(angle, True)))

  def test_inactive_angle_requires_exact_measured_command_and_range(self):
    self.safety.set_controls_allowed(False)
    for measured in (39.2, 39.3, 150.1, 370.0):
      self._reset_angle_samples(measured)
      self.assertTrue(self._tx(self._angle_cmd_msg(measured, False)))
      self.assertFalse(self._tx(self._angle_cmd_msg(measured + 0.1, False)))
    self._reset_angle_samples(370.4)
    self.assertTrue(self._tx(self._angle_cmd_msg(370.4, False)))
    self.assertFalse(self._tx(self._angle_cmd_msg(370.5, False)))

  def test_speed_measurement_representable_range(self):
    for speed in (0, 1, 5, 10, 15, 30, 50, 75):
      self._reset_speed_samples(speed)
      self.assertGreaterEqual(self.safety.get_vehicle_speed_max(), 0)
      self.assertLessEqual(self.safety.get_vehicle_speed_max(), 75.1 if speed == 75 else speed + 1)

  def test_active_commands_require_controls_and_safe_boundary_is_accepted(self):
    for enabled, controls in ((True, False), (True, True), (False, False), (False, True)):
      self.safety.set_controls_allowed(controls)
      self.safety.set_desired_angle_last(0)
      result = self._tx(self._angle_cmd_msg(0, enabled))
      self.assertEqual(result, controls or not enabled)

  # Chery VM limits replace static AngleSteeringSafetyTest rate tables: the
  # Panda hook and production controller must share vehicle-model boundaries.
  def _vm(self):
    return VehicleModel(CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5))

  def _vm_angle(self, speed):
    return min(150.0, get_max_angle_vm(max(speed, 1.0), self._vm(), CarControllerParams))

  def test_vm_lateral_accel_boundaries(self):
    for speed in (0, 1, 5, 10, 15, 30, 50):
      model_speed = max(speed, 1)
      self._reset_speed_samples(model_speed + 1)
      limit = self._vm_angle(speed)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        boundary = sign * math.floor(limit * 10) / 10
        self.safety.set_desired_angle_last(round(boundary * 100))
        self.assertTrue(self._tx(self._angle_cmd_msg(boundary, True)), (speed, sign, limit, boundary))
        self.safety.set_desired_angle_last(round(boundary * 100))
        outside = round(sign * (limit + 0.1), 1)
        self.assertFalse(self._tx(self._angle_cmd_msg(outside, True)))

  def test_vm_lateral_jerk_boundaries(self):
    for speed in (0, 1, 5, 10, 15, 30, 50):
      model_speed = max(speed, 1)
      self._reset_speed_samples(model_speed + 1)
      limit = min(150.0, 5.0, get_max_angle_delta_vm(model_speed, self._vm(), CarControllerParams))
      self.safety.set_controls_allowed(True)
      self.safety.set_desired_angle_last(0)
      boundary = math.floor(limit * 10) / 10
      self.assertTrue(self._tx(self._angle_cmd_msg(boundary, True)), (speed, limit))
      self.safety.set_desired_angle_last(0)
      if limit < 150.0:
        outside = math.ceil(limit * 10) / 10 + 0.1
        self.assertFalse(self._tx(self._angle_cmd_msg(outside, True)), (speed, limit, boundary, outside))

  def test_vm_real_time_window_accepts_50hz_commands(self):
    self._reset_speed_samples(11)
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(0)
    angle = 0.1
    for _ in range(10):
      self.assertTrue(self._tx(self._angle_cmd_msg(angle, True)))

  def test_front_speed_vm_setup_matches_production_wire_command(self):
    # Front/rear disagreement must not make Panda validate controller output
    # against rear speed; production uses the front wheel mean.
    self._reset_speed_samples(18)
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(0)
    self.assertTrue(self._tx(self._angle_cmd_msg(0.1, True)))

  def test_every_non_steering_tx_and_wrong_dlc_is_rejected(self):
    self.safety.set_controls_allowed(True)
    for addr, length in ((0x03E, 48), (0x1D3, 8), (0x316, 8), (0x394, 8),
                         (0x3A2, 8), (0x3A5, 8), (0x360, 6)):
      self.assertFalse(self._tx(make_msg(0, addr, length)))
    for length in (0, 1, 2, 4, 7):
      self.assertFalse(self._tx(self._angle_cmd_msg(0, False, length=length)))

  def test_forwarding_routes_and_relay_protection(self):
    for addr, source, destination in (
      (0x345, 0, 2), (0x3A2, 2, 0), (0x360, 0, 2), (0x3A2, 0, 2),
    ):
      self.assertEqual(destination, self.safety.safety_fwd_hook(source, addr))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x345))
    for bus in (1, 3):
      self.assertEqual(-1, self.safety.safety_fwd_hook(bus, 0x345))

    for angle, destination in ((370.4, -1), (-370.4, -1), (370.5, 0), (-370.5, 0)):
      self._reset_angle_samples(angle)
      self.assertEqual(destination, self.safety.safety_fwd_hook(2, 0x345))

    self.safety.set_relay_malfunction(True)
    for bus in range(4):
      self.assertFalse(self._tx(make_msg(bus, 0x345, 8)))
      self.assertEqual(-1, self.safety.safety_fwd_hook(bus, 0x345))

  def test_acc_mads_authorization_stays_lateral_until_unavailable(self):
    self._engage()
    self.safety.set_controls_allowed_lateral(True)
    for address, (bus, _dlc, _frequency) in RX_LAYOUT.items():
      if address not in (0x3A2, 0x3A5):
        self._rx(self._packet(address, bus))
        self.assertTrue(self.safety.get_controls_allowed_lateral())

    self._rx_field(0x3A2, state=0)
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_acc_authorization_arrival_orders_and_states(self):
    for first, second in ((0x3A2, 0x3A5), (0x3A5, 0x3A2)):
      self.setUp()
      self._seed_all()
      self._rx_field(first, state=2) if first == 0x3A2 else self._rx_field(first, active=1)
      self.assertFalse(self.safety.get_controls_allowed())
      self._rx_field(second, state=2) if second == 0x3A2 else self._rx_field(second, active=1)
      self.assertFalse(self.safety.get_controls_allowed())
      self._validate_config()
      self._rx_field(second, state=2) if second == 0x3A2 else self._rx_field(second, active=1)
      self.assertFalse(self.safety.get_controls_allowed())
      self._rx_field(0x3A5, active=0)
      self._rx_field(0x3A2, state=2)
      self._rx_field(0x3A5, active=1)
      self.assertTrue(self.safety.get_controls_allowed())
    for state in (0, 1):
      self.setUp()
      self._seed_all()
      self._validate_config()
      self._rx_field(0x3A2, state=state)
      self._rx_field(0x3A5, active=1)
      self.assertFalse(self.safety.get_controls_allowed())
    self.setUp()
    self._seed_all()
    self._validate_config()
    self._rx_field(0x3A2, state=1)
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx_field(0x3A5, active=1)
    self.assertFalse(self.safety.get_controls_allowed())
    self.setUp()
    self._seed_all()
    self._validate_config()
    self._rx_field(0x3A5, active=1)
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx_field(0x3A2, state=3)
    self.assertTrue(self.safety.get_controls_allowed())

  def test_reinit_clears_chery_authorization_and_inhibitor_state(self):
    self._engage()
    self._rx_field(0x03E, brake=1, engine_gas=11)
    self._rx_field(0x3A5, aeb=1, active=1)
    self.assertFalse(self.safety.get_controls_allowed())
    self.setUp()
    self._rx_field(0x3A5, active=1)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_acc_off_requires_new_authorization_cycle(self):
    self._engage()
    self._disengage()
    self._rx_field(0x3A2, state=2)
    self._rx_field(0x3A5, active=1)
    self.assertTrue(self.safety.get_controls_allowed())

  def test_each_inhibitor_revokes_and_stays_revoked(self):
    for field, address, value in (("brake", 0x03E, 1), ("engine_gas", 0x03E, 11),
                                   ("acc_gas", 0x3A2, 1), ("aeb", 0x3A5, 1)):
      for speed in (0, 100) if field == "brake" else (100,):
        self.setUp()
        self._seed_all()
        self._validate_config()
        self._rx_field(0x316, fr=speed, fl=speed)
        self._rx_field(address, **{field: value})
        self._rx_field(0x3A2, state=2, acc_gas=value if field == "acc_gas" else 0)
        self._rx_field(0x3A5, active=1, aeb=value if field == "aeb" else 0)
        self.assertFalse(self.safety.get_controls_allowed())
        release = {field: 0}
        if field == "aeb":
          release["active"] = 1
        self._rx_field(address, **release)
        self._rx_field(0x3A5, active=1, aeb=0)
        self.assertFalse(self.safety.get_controls_allowed())
        self._rx_field(0x03E, brake=0, engine_gas=0)
        self._rx_field(0x3A2, acc_gas=0)
        self._rx_field(0x3A5, active=0)
        self._rx_field(0x3A2, state=2)
        self._rx_field(0x3A5, active=1)
        self.assertTrue(self.safety.get_controls_allowed())

  def test_gas_sources_are_or_interleaved(self):
    self._rx_field(0x03E, engine_gas=11)
    self.assertTrue(self.safety.get_gas_pressed_prev())
    self._rx_field(0x3A2, acc_gas=0)
    self.assertTrue(self.safety.get_gas_pressed_prev())
    self._rx_field(0x3A2, acc_gas=1)
    self._rx_field(0x03E, engine_gas=0)
    self.assertTrue(self.safety.get_gas_pressed_prev())
    self._rx_field(0x3A2, acc_gas=0)
    self.assertFalse(self.safety.get_gas_pressed_prev())

  def test_invalid_integrity_and_counter_revoke_engaged_controls(self):
    self._engage()
    bad = self._packet(0x3A5, 2)
    bad.data[7] ^= 1
    self.assertFalse(self._rx(bad))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._rx_field(0x3A5, active=1)
    self.assertFalse(self.safety.get_controls_allowed())

    self.setUp()
    self._engage()
    for _ in range(MAX_WRONG_COUNTERS - 1):
      self.assertTrue(self._rx(self._packet(0x3A5, 2, counter=1)))
    self.assertFalse(self._rx(self._packet(0x3A5, 2, counter=1)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_engagement_revoked_when_rx_config_times_out(self):
    self._engage()
    self._seed_all()
    self.assertTrue(self.safety.safety_config_valid())
    self.safety.set_timer(2_000_001)
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.safety_config_valid())
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_pre_engagement_checksum_fault_requires_acc_off(self):
    self._seed_all()
    self._validate_config()
    bad = self._packet(0x3A5, 2, active=1)
    bad.data[7] ^= 1
    self.assertFalse(self._rx(bad))
    self._rx_field(0x3A2, state=2)
    self._rx_field(0x3A5, active=1)
    self.assertFalse(self.safety.get_controls_allowed())

    self._recover_rx_without_acc_off()
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx_field(0x3A2, state=0)
    self._rx_field(0x3A2, state=2)
    self._rx_field(0x3A5, active=1)
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx_field(0x3A5, active=0)
    self._rx_field(0x3A2, state=2)
    self._rx_field(0x3A5, active=1)
    self.assertTrue(self.safety.get_controls_allowed())

  def test_pre_engagement_timeout_fault_requires_acc_off(self):
    self._seed_all()
    self._validate_config()
    self.safety.set_timer(2_000_001)
    self.safety.safety_tick_current_safety_config()
    self._rx_field(0x3A2, state=2)
    self._rx_field(0x3A5, active=1)
    self.assertFalse(self.safety.get_controls_allowed())

    self._recover_rx_without_acc_off()
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx_field(0x3A2, state=0)
    self._rx_field(0x3A2, state=2)
    self._rx_field(0x3A5, active=1)
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx_field(0x3A5, active=0)
    self._rx_field(0x3A2, state=2)
    self._rx_field(0x3A5, active=1)
    self.assertTrue(self.safety.get_controls_allowed())

  def test_pre_engagement_negative_wheel_fault_requires_acc_off(self):
    self._seed_all()
    self._validate_config()
    self._rx_field(0x316, fr=-1, fl=100)
    self._rx_field(0x3A2, state=2)
    self._rx_field(0x3A5, active=1)
    self.assertFalse(self.safety.get_controls_allowed())

    self._recover_rx_without_acc_off()
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx_field(0x3A2, state=0)
    self._rx_field(0x316, fr=100, fl=100)
    self._rx_field(0x3A2, state=2)
    self._rx_field(0x3A5, active=1)
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx_field(0x3A5, active=0)
    self._rx_field(0x3A2, state=2)
    self._rx_field(0x3A5, active=1)
    self.assertTrue(self.safety.get_controls_allowed())

  def test_signal_extraction_boundaries(self):
    for raw, expected in ((0, -78000), (16383, 85830)):
      for _ in range(6):
        self._rx_field(0x1D3, angle_raw=raw)
      self.assertEqual(self.safety.get_angle_meas_min(), expected)
      self.assertEqual(self.safety.get_angle_meas_max(), expected)
    for raw, expected in ((0x800, -2048), (0xFFF, -1), (0, 0), (0x7FF, 2047)):
      for _ in range(6):
        self._rx_field(0x394, torque_raw=raw)
      self.assertEqual(self.safety.get_torque_driver_min(), expected)
      self.assertEqual(self.safety.get_torque_driver_max(), expected)
    self._rx_field(0x316, fr=0, fl=0)
    self.assertFalse(self.safety.get_vehicle_moving())
    self._rx_field(0x316, fr=100, fl=100)
    self.assertTrue(self.safety.get_vehicle_moving())
    self.assertGreater(self.safety.get_vehicle_speed_max(), 0)
    self._rx_field(0x316, fr=-1, fl=100)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._rx_field(0x316, fr=0x316, fl=0x8000)
    self.assertFalse(self.safety.get_controls_allowed())
    for gas, expected in ((10, False), (11, True)):
      self.setUp()
      self._rx_field(0x03E, engine_gas=gas)
      self.assertEqual(self.safety.get_gas_pressed_prev(), expected)
    self.setUp()
    self._rx_field(0x03E, brake=0)
    self._rx_field(0x03E, brake=0)
    self.assertFalse(self.safety.get_brake_pressed_prev())
    self._rx_field(0x03E, brake=1)
    self._rx_field(0x03E, brake=1)
    self.assertTrue(self.safety.get_brake_pressed_prev())
