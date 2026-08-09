from opendbc.safety.tests.common import MAX_WRONG_COUNTERS, SafetyTest, make_msg
from opendbc.safety.tests.libsafety import libsafety_py


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
  TX_MSGS = []
  FWD_BUS_LOOKUP = {}
  FWD_BLACKLISTED_ADDRS = {}

  @classmethod
  def setUpClass(cls):
    cls.safety = libsafety_py.libsafety

  def setUp(self):
    self.assertEqual(self.safety.set_safety_hooks(SAFETY_CHERY, 0), 0)
    self.safety.init_tests()

  def _golden(self, address, bus):
    return libsafety_py.make_CANPacket(address, bus, GOLDEN_FRAMES[(address, bus)])

  def _seed_all(self):
    for address, (bus, _dlc, _frequency) in RX_LAYOUT.items():
      self._rx(self._golden(address, bus))

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

  def test_each_engine_data_integrity_island_is_required(self):
    for island in range(5):
      self.setUp()
      packet = self._golden(0x03E, 0)
      packet.data[island * 8] ^= 1
      self.assertFalse(self._rx(packet))

      self.setUp()
      packet = self._golden(0x03E, 0)
      packet.data[island * 8 + 1] ^= 1
      self.assertFalse(self._rx(packet))

  def test_wrong_bus_or_dlc_does_not_match(self):
    for address, (bus, dlc, _frequency) in RX_LAYOUT.items():
      self.setUp()
      for other_address, (other_bus, _other_dlc, _other_frequency) in RX_LAYOUT.items():
        if other_address != address:
          self._rx(self._golden(other_address, other_bus))
      wrong_bus = 2 if bus == 0 else 0
      wrong_dlc = 64 if dlc == 48 else 4
      self._rx(make_msg(wrong_bus, address, dlc if dlc != 48 else 64))
      self._rx(make_msg(bus, address, wrong_dlc))
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
    self.assertFalse(self.safety.safety_tx_hook(make_msg(0, 0x345)))
    self.assertEqual(self.safety.safety_fwd_hook(0, 0x123), -1)
