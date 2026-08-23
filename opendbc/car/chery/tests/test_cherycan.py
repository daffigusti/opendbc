import pytest

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


@pytest.mark.parametrize("angle", [39.2, 39.3, 150.1, 370.0, -370.0])
def test_inactive_steering_preserves_exact_command(angle):
  packer = CANPacker("chery_canfd")
  stock = {name: 0 for name in (
    "CMD", "NEW_SIGNAL_3", "LKA_ACTIVE", "SET_X0", "NEW_SIGNAL_5", "NEW_SIGNAL_6",
    "NEW_SIGNAL_7", "NEW_SIGNAL_1", "CHECKSUM",
  )}
  address, dat, bus = create_steering_control(packer, 0, angle, False, stock)
  parser = CANParser("chery_canfd", [("LKAS_CAM_CMD_345", 0)], 0)
  parser.update([[0, [(address, dat, bus)]]])
  assert parser.vl["LKAS_CAM_CMD_345"]["CMD"] == round(angle * 10 - 392)


@pytest.mark.parametrize("apply_steer, fixture", [(-7.1, 0), (-7.0, 1)])
def test_steering_captured_stock_fields_and_checksum(apply_steer, fixture):
  packer = CANPacker("chery_canfd")
  stock_parser = CANParser("chery_canfd", [("LKAS_CAM_CMD_345", 2)], 2)
  captured = GOLDEN_FRAMES[(0x345, 2)][fixture]
  stock_parser.update([[0, [(0x345, captured, 2)]]])
  stock = stock_parser.vl["LKAS_CAM_CMD_345"]
  _addr, dat, bus = create_steering_control(packer, 2, apply_steer, False, stock)
  assert (dat, bus) == (captured, 2)


def test_active_steering_signals_and_checksum():
  packer = CANPacker("chery_canfd")
  stock = {name: 0 for name in (
    "CMD", "NEW_SIGNAL_3", "LKA_ACTIVE", "SET_X0", "NEW_SIGNAL_5", "NEW_SIGNAL_6",
    "NEW_SIGNAL_7", "NEW_SIGNAL_1", "CHECKSUM",
  )}
  address, dat, bus = create_steering_control(packer, 2, 39.4, True, stock)
  parser = CANParser("chery_canfd", [("LKAS_CAM_CMD_345", 2)], 2)
  parser.update([[0, [(address, dat, bus)]]])
  values = parser.vl["LKAS_CAM_CMD_345"]
  assert values["CMD"] == 2
  assert values["LKA_ACTIVE"] == 1
  assert values["NEW_SIGNAL_3"] == 1
  assert values["CHECKSUM"] == calculate_crc(dat[:-1])


@pytest.mark.parametrize("fixture, frame", [(0, 8), (1, 9)])
def test_button_captured_frames(fixture, frame):
  packer = CANPacker("chery_canfd")
  captured = GOLDEN_FRAMES[(0x360, 0)][fixture]
  parser = CANParser("chery_canfd", [("STEER_BUTTON", 2)], 2)
  parser.update([[0, [(0x360, captured, 2)]]])
  stock = parser.vl["STEER_BUTTON"]
  _, dat, bus = create_button_control(packer, 2, frame, stock)
  assert (dat, bus) == (captured, 2)
  assert dat[0] == (0x33, 0xDD)[fixture]
  assert dat[0] == calculate_crc(dat[1:])


def test_counter_wraps_at_four_bits():
  packer = CANPacker("chery_canfd")
  stock = {name: 0 for name in (
    "ACC", "CC_BTN", "RES_PLUS", "RES_MINUS", "NEW_SIGNAL_1",
    "GAP_ADJUST_UP", "GAP_ADJUST_DOWN",
  )}
  parser = CANParser("chery_canfd", [("STEER_BUTTON", 2)], 2)
  counters = []
  for frame in (14, 15, 16):
    address, dat, bus = create_button_control(packer, 2, frame, stock)
    parser.update([[0, [(address, dat, bus)]]])
    counters.append(parser.vl["STEER_BUTTON"]["COUNTER"])
  assert counters == [14, 15, 0]


def test_acc_counter_and_checksum():
  packer = CANPacker("chery_canfd")
  stock = {name: 0 for name in (
    "ACC_STATE", "STOPPED", "ACC_STATE_2", "NEW_SIGNAL_12", "NEW_SIGNAL_9",
    "NEW_SIGNAL_2", "STOPPING", "NEW_SIGNAL_13", "NEW_SIGNAL_8", "NEW_SIGNAL_5",
    "NEW_SIGNAL_6", "NEW_SIGNAL_10", "NEW_SIGNAL_3", "NEW_SIGNAL_4", "AEB_REQ_STOP",
    "COUNTER",
  )}
  _, dat, bus = create_acc_control(packer, 2, stock, True, 0, False, False)
  assert bus == 2
  assert dat[-1] == calculate_crc(dat[:-1])


@pytest.mark.parametrize("gas, command", [(-3.5, -511), (0.0, -24), (2.0, 511), (-10.0, -511), (10.0, 511)])
def test_acc_command_maps_clamped_piecewise_accel(gas, command):
  packer = CANPacker("chery_canfd")
  stock = {name: 0 for name in (
    "ACC_STATE", "STOPPED", "ACC_STATE_2", "NEW_SIGNAL_12", "NEW_SIGNAL_9",
    "NEW_SIGNAL_2", "STOPPING", "NEW_SIGNAL_13", "NEW_SIGNAL_8", "NEW_SIGNAL_5",
    "NEW_SIGNAL_6", "NEW_SIGNAL_10", "NEW_SIGNAL_3", "NEW_SIGNAL_4", "AEB_REQ_STOP",
    "COUNTER",
  )}
  address, dat, bus = create_acc_control(packer, 0, stock, True, gas, False, False)
  parser = CANParser("chery_canfd", [("ACC_CMD", 0)], 0)
  parser.update([[0, [(address, dat, bus)]]])
  assert parser.vl["ACC_CMD"]["CMD"] == command


@pytest.mark.parametrize("full_stop", [False, True])
def test_acc_full_stop_holds_with_stock_command_and_inactive_preserves_stock_state(full_stop):
  packer = CANPacker("chery_canfd")
  stock = {name: 0 for name in (
    "ACC_STATE", "STOPPED", "ACC_STATE_2", "NEW_SIGNAL_12", "NEW_SIGNAL_9",
    "NEW_SIGNAL_2", "STOPPING", "NEW_SIGNAL_13", "NEW_SIGNAL_8", "NEW_SIGNAL_5",
    "NEW_SIGNAL_6", "NEW_SIGNAL_10", "NEW_SIGNAL_3", "NEW_SIGNAL_4", "AEB_REQ_STOP",
    "COUNTER",
  )}
  stock.update({"ACC_STATE": 1, "STOPPED": 1})
  parser = CANParser("chery_canfd", [("ACC_CMD", 0)], 0)
  for long_active in (False, True):
    address, dat, bus = create_acc_control(packer, 0, stock, long_active, 2.0, full_stop, False)
    parser.update([[0, [(address, dat, bus)]]])
    values = parser.vl["ACC_CMD"]
    if not long_active:
      assert values["CMD"] == -24
      assert values["ACCEL_ON"] == 0
      assert values["ACC_STATE"] == 1
      assert values["STOPPED"] == 1
    elif full_stop:
      # CMD is a magnitude and ACCEL_ON its direction: 400 with ACCEL_ON clear is the stock
      # maximum brake request that holds a stopped car.
      assert values["CMD"] == 400
      assert values["ACCEL_ON"] == 0
      assert values["ACC_STATE"] == 2
      assert values["STOPPED"] == 1
    else:
      assert values["CMD"] == 511
      assert values["ACCEL_ON"] == 1
      assert values["ACC_STATE"] == 3
      assert values["STOPPED"] == 0


@pytest.mark.parametrize("fixture", [0, 1])
def test_acc_captured_frames(fixture):
  packer = CANPacker("chery_canfd")
  captured = GOLDEN_FRAMES[(0x3A2, 2)][fixture]
  parser = CANParser("chery_canfd", [("ACC_CMD", 2)], 2)
  parser.update([[0, [(0x3A2, captured, 2)]]])
  stock = parser.vl["ACC_CMD"]
  _, dat, bus = create_acc_control(packer, 2, stock, False, 0, False, False)
  assert (dat, bus) == (captured, 2)
  assert stock["CMD"] == -24
  assert stock["ACC_STATE"] == 1


def test_acc_counter_copies_arbitrary_stock_and_tracks_stock_sequence():
  packer = CANPacker("chery_canfd")
  parser = CANParser("chery_canfd", [("ACC_CMD", 2)], 2)
  stock = {name: 0 for name in (
    "ACC_STATE", "STOPPED", "ACC_STATE_2", "NEW_SIGNAL_12", "NEW_SIGNAL_9",
    "NEW_SIGNAL_2", "STOPPING", "NEW_SIGNAL_13", "NEW_SIGNAL_8", "NEW_SIGNAL_5",
    "NEW_SIGNAL_6", "NEW_SIGNAL_10", "NEW_SIGNAL_3", "NEW_SIGNAL_4", "AEB_REQ_STOP",
    "COUNTER",
  )}
  counters = []
  for counter in (7, 8, 9, 10):
    stock["COUNTER"] = counter
    address, dat, bus = create_acc_control(packer, 2, stock, False, 0, False, False)
    parser.update([[0, [(address, dat, bus)]]])
    counters.append(parser.vl["ACC_CMD"]["COUNTER"])
  assert counters == [7, 8, 9, 10]


def test_acc_request_stop_is_not_copied_from_stock():
  packer = CANPacker("chery_canfd")
  stock = {name: 0 for name in (
    "ACC_STATE", "STOPPED", "ACC_STATE_2", "NEW_SIGNAL_12", "NEW_SIGNAL_9",
    "NEW_SIGNAL_2", "STOPPING", "NEW_SIGNAL_13", "NEW_SIGNAL_8", "NEW_SIGNAL_5",
    "NEW_SIGNAL_6", "NEW_SIGNAL_10", "NEW_SIGNAL_3", "NEW_SIGNAL_4", "AEB_REQ_STOP",
    "COUNTER",
  )}
  stock["AEB_REQ_STOP"] = 7
  _, dat, _ = create_acc_control(packer, 2, stock, False, 0, False, False)
  parser = CANParser("chery_canfd", [("ACC_CMD", 2)], 2)
  parser.update([[0, [(0x3A2, dat, 2)]]])
  assert parser.vl["ACC_CMD"]["AEB_REQ_STOP"] == 0


def j1850(data):
  crc = 0xFF
  for byte in data:
    crc ^= byte
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc ^ 0xFF


def test_integrity_fields_decode_from_golden_frames():
  parser = CANParser("chery_canfd", [("ENGINE_DATA", 0), ("STEER_ANGLE_SENSOR", 0),
                                     ("WHEEL_SPEED_FRNT", 0), ("STEER_SENSOR_2", 0)], 0)
  for address, message in ((0x03E, "ENGINE_DATA"), (0x1D3, "STEER_ANGLE_SENSOR"),
                           (0x316, "WHEEL_SPEED_FRNT"), (0x394, "STEER_SENSOR_2")):
    for frame in GOLDEN_FRAMES[(address, 0)]:
      parser.update([[0, [(address, frame, 0)]]])
      values = parser.vl[message]
      if address == 0x03E:
        for island in range(5):
          assert values[f"ENGINE_DATA_CHECKSUM_{island}"] == frame[island * 8]
          assert values[f"ENGINE_DATA_COUNTER_{island}"] == frame[island * 8 + 1] & 0xF
      else:
        assert values["COUNTER"] == frame[6] & 0xF
        assert values["CHECKSUM"] == frame[7]


def test_j1850_integrity_on_committed_golden_frames():
  for address, bus in ((0x1D3, 0), (0x394, 0), (0x3A2, 2), (0x3A5, 2)):
    for frame in GOLDEN_FRAMES[(address, bus)]:
      assert frame[-1] == j1850(frame[:-1])
  for frame in GOLDEN_FRAMES[(0x03E, 0)]:
    for offset in range(0, 40, 8):
      assert frame[offset] == j1850(frame[offset + 1:offset + 8])


def test_additive_complement_and_observed_counter_progression():
  for frame in GOLDEN_FRAMES[(0x316, 0)]:
    assert frame[-1] == (0xFF - sum(frame[:-1])) & 0xFF
  for address, bus in ((0x03E, 0), (0x1D3, 0), (0x394, 0), (0x3A2, 2), (0x3A5, 2)):
    frames = GOLDEN_FRAMES[(address, bus)]
    if address == 0x03E:
      sequences = [[frame[offset + 1] & 0xF for frame in frames] for offset in range(0, 40, 8)]
    else:
      sequences = [[frame[6] & 0xF for frame in frames]]
    for sequence in sequences:
      assert all((next_value - value) % 16 == 1 for value, next_value in zip(sequence, sequence[1:], strict=False))
