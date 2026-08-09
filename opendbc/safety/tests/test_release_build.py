#!/usr/bin/env python3
import unittest

from opendbc.safety.tests.libsafety.libsafety_py import _build_libsafety, ffi


class TestBuild(unittest.TestCase):
  @staticmethod
  def _acc_packet(ffi, command, aeb_req_stop=0):
    packet = ffi.new('CANPacket_t *')
    packet[0].addr = 0x3A2
    packet[0].bus = 0
    packet[0].data_len_code = 8
    raw = command & 0x3FF
    packet[0].data[0] = (raw >> 3) & 0x7F
    packet[0].data[1] = (raw & 0x7) << 5
    packet[0].data[6] = (aeb_req_stop & 0xF) << 4
    return packet

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
    self.assertEqual(safety.safety_fwd_hook(0, 0x123), 2)

  def test_release_chery_longitudinal_flag_matches_development_contract(self):
    path = _build_libsafety(release=True)
    safety = ffi.dlopen(path)
    safety.set_safety_hooks(35, 1)
    safety.set_controls_allowed(True)
    self.assertTrue(safety.safety_tx_hook(self._acc_packet(ffi, -511)))
    self.assertTrue(safety.safety_tx_hook(self._acc_packet(ffi, 511)))
    safety.set_controls_allowed(False)
    self.assertTrue(safety.safety_tx_hook(self._acc_packet(ffi, -24)))
    self.assertFalse(safety.safety_tx_hook(self._acc_packet(ffi, -511)))
    self.assertFalse(safety.safety_tx_hook(self._acc_packet(ffi, -24, 1)))
    self.assertEqual(safety.safety_fwd_hook(2, 0x3A2), -1)

    safety.set_safety_hooks(35, 0)
    self.assertFalse(safety.safety_tx_hook(self._acc_packet(ffi, -24)))
    self.assertEqual(safety.safety_fwd_hook(2, 0x3A2), 0)


if __name__ == "__main__":
  unittest.main()
