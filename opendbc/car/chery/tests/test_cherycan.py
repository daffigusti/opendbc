from opendbc.can import CANPacker, CANParser
from opendbc.car.chery.cherycan import calculate_crc, create_acc_control, create_button_control, create_steering_control

from .fixtures import GOLDEN_FRAMES


def test_crc_known_vector():
  assert calculate_crc(bytes.fromhex("00000000000000")) == 0x0A


def test_inactive_steering_tracks_stock_frame():
  packer = CANPacker("chery_canfd")
  stock = {
    "CMD": -392,
    "NEW_SIGNAL_3": 0,
    "LKA_ACTIVE": 0,
    "SET_X0": 0,
    "NEW_SIGNAL_5": 0,
    "NEW_SIGNAL_6": 0,
    "NEW_SIGNAL_7": 0,
    "NEW_SIGNAL_1": 0,
  }
  _addr, dat, bus = create_steering_control(packer, 0, 0.0, False, stock)
  assert bus == 0
  assert len(dat) == 8
  assert dat[-1] == calculate_crc(dat[:-1])


def test_steering_captured_stock_fields_and_checksum():
  packer = CANPacker("chery_canfd")
  stock_parser = CANParser("chery_canfd", [("LKAS_CAM_CMD_345", 2)], 2)
  captured = GOLDEN_FRAMES[(0x345, 2)][0]
  stock_parser.update([[0, [(0x345, captured, 2)]]])
  stock = stock_parser.vl["LKAS_CAM_CMD_345"]
  _addr, dat, bus = create_steering_control(packer, 2, -7.1, False, stock)
  assert (dat, bus) == (captured, 2)


def test_button_counter_and_checksum():
  packer = CANPacker("chery_canfd")
  stock = {name: 0 for name in (
    "ACC", "CC_BTN", "RES_PLUS", "RES_MINUS", "NEW_SIGNAL_1",
    "GAP_ADJUST_UP", "GAP_ADJUST_DOWN", "COUNTER", "CHECKSUM",
  )}
  _, first, _ = create_button_control(packer, 2, 3, stock, cancel=True)
  _, second, _ = create_button_control(packer, 2, 4, stock, resume=True)
  assert first[1] != second[1]
  assert first[0] == calculate_crc(first[1:])
  assert second[0] == calculate_crc(second[1:])


def test_acc_counter_and_checksum():
  packer = CANPacker("chery_canfd")
  stock = {name: 0 for name in (
    "ACC_STATE", "STOPPED", "ACC_STATE_2", "NEW_SIGNAL_12", "NEW_SIGNAL_9",
    "NEW_SIGNAL_2", "STOPPING", "NEW_SIGNAL_13", "NEW_SIGNAL_8", "NEW_SIGNAL_5",
    "NEW_SIGNAL_6", "NEW_SIGNAL_10", "NEW_SIGNAL_3", "NEW_SIGNAL_4", "AEB_REQ_STOP",
  )}
  _, dat, bus = create_acc_control(packer, 2, stock, 7, True, 0, 0, False, False, False)
  assert bus == 2
  assert dat[-1] == calculate_crc(dat[:-1])
