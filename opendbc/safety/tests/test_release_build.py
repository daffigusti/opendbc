#!/usr/bin/env python3
import unittest

from opendbc.safety.tests.libsafety.libsafety_py import _build_libsafety, ffi


class TestBuild(unittest.TestCase):
  def test_development_build(self):
    _build_libsafety(release=False)

  def test_release_build(self):
    path = _build_libsafety(release=True)
    safety = ffi.dlopen(path)
    self.assertEqual(safety.set_safety_hooks(35, 0), 0)
    self.assertEqual(safety.get_current_safety_mode(), 35)
    self.assertEqual(safety.get_current_safety_rx_checks_len(), 6)
    packet = ffi.new('CANPacket_t *')
    packet[0].addr = 0
    packet[0].bus = 0
    packet[0].data_len_code = 8
    packet[0].data = b'\0' * 8
    self.assertFalse(safety.safety_tx_hook(packet))
    self.assertEqual(safety.safety_fwd_hook(0, 0x123), -1)


if __name__ == "__main__":
  unittest.main()
