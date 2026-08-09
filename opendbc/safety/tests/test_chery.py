from opendbc.safety.tests.common import SafetyTestBase, make_msg
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


class TestCherySafety(SafetyTestBase):
  @classmethod
  def setUpClass(cls):
    cls.safety = libsafety_py.libsafety

  def setUp(self):
    self.safety.set_safety_hooks(SAFETY_CHERY, 0)

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
      self.assertTrue(self.safety.get_rx_check_ignore_quality(index))
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

  def test_wrong_bus_or_dlc_does_not_match(self):
    for address, (bus, dlc, _frequency) in RX_LAYOUT.items():
      self.setUp()
      wrong_bus = 2 if bus == 0 else 0
      self._rx(make_msg(wrong_bus, address, dlc))
      self._rx(make_msg(bus, address, 8 if dlc == 48 else 48))
      self.assertFalse(self.safety.safety_config_valid())

  def test_tx_denied_and_forwarding_disabled(self):
    self.assertFalse(self.safety.safety_tx_hook(make_msg(0, 0x345)))
    self.assertEqual(self.safety.safety_fwd_hook(0, 0x123), -1)
