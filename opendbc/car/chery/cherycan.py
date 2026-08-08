from opendbc.car import CanBusBase

STEER_ANGLE_OFFSET = -392
STEER_ANGLE_SCALE = 10
CRC_POLY = 0x1D
CRC_INIT = 0xFF
CRC_XOR = 0xFF


def calculate_crc(data: bytes) -> int:
  crc = CRC_INIT
  for byte in data:
    crc ^= byte
    for _ in range(8):
      crc = ((crc << 1) ^ CRC_POLY) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc ^ CRC_XOR


def create_steering_control(packer, bus: int, apply_steer: float, lkas_enable: bool, stock_values: dict):
  command = int(apply_steer * STEER_ANGLE_SCALE + STEER_ANGLE_OFFSET)
  if 0 <= command <= 2:
    command = 2
  values = {
    "CMD": command,
    "NEW_SIGNAL_3": 1 if command > 1 else 0,
    "LKA_ACTIVE": 1 if lkas_enable else 0,
    "SET_X0": 0,
    "NEW_SIGNAL_5": stock_values["NEW_SIGNAL_5"],
    "NEW_SIGNAL_6": stock_values["NEW_SIGNAL_6"],
    "NEW_SIGNAL_7": stock_values["NEW_SIGNAL_7"],
    "NEW_SIGNAL_1": stock_values["NEW_SIGNAL_1"],
  }
  _, dat, _ = packer.make_can_msg("LKAS_CAM_CMD_345", bus, values)
  values["CHECKSUM"] = calculate_crc(dat[:-1])
  return packer.make_can_msg("LKAS_CAM_CMD_345", bus, values)


def create_button_control(packer, bus: int, frame: int, stock_values: dict, cancel: bool = False, resume: bool = False):
  values = {name: stock_values[name] for name in (
    "ACC", "CC_BTN", "RES_PLUS", "RES_MINUS", "NEW_SIGNAL_1",
    "GAP_ADJUST_UP", "GAP_ADJUST_DOWN",
  )}
  values.update({
    "ACC": 1 if cancel else 0,
    "RES_PLUS": 1 if resume else 0,
    "COUNTER": frame % 0x10,
  })
  _, dat, _ = packer.make_can_msg("STEER_BUTTON", bus, values)
  values["CHECKSUM"] = calculate_crc(dat[1:])
  return packer.make_can_msg("STEER_BUTTON", bus, values)


def create_acc_control(packer, bus: int, stock_values: dict, frame: int, long_active: bool,
                       gas: float, full_stop: bool, resume: bool):
  throttle = gas if long_active else -24
  values = {name: stock_values[name] for name in (
    "ACC_STATE_2", "NEW_SIGNAL_12", "NEW_SIGNAL_9", "NEW_SIGNAL_2", "STOPPING",
    "NEW_SIGNAL_13", "NEW_SIGNAL_8", "NEW_SIGNAL_5", "NEW_SIGNAL_6", "NEW_SIGNAL_10",
    "NEW_SIGNAL_3", "NEW_SIGNAL_4", "AEB_REQ_STOP",
  )}
  values.update({
    "CMD": 400 if full_stop else throttle,
    "ACCEL_ON": 1 if throttle >= 0 else 0,
    "ACC_STATE": 2 if full_stop else 3 if long_active else stock_values["ACC_STATE"],
    "STOPPED": 1 if full_stop else 0 if long_active else stock_values["STOPPED"],
    "STOPPING": stock_values["STOPPING"],
    "GAS_PRESSED": 1 if resume else 0,
    "AEB_REQ_STOP": 0,
    "COUNTER": frame % 0x10,
  })
  _, dat, _ = packer.make_can_msg("ACC_CMD", bus, values)
  values["CHECKSUM"] = calculate_crc(dat[:-1])
  return packer.make_can_msg("ACC_CMD", bus, values)


class CanBus(CanBusBase):
  def __init__(self, CP=None, fingerprint=None) -> None:
    super().__init__(CP, fingerprint)

  @property
  def main(self) -> int:
    return self.offset

  @property
  def radar(self) -> int:
    return self.offset + 1

  @property
  def camera(self) -> int:
    return self.offset + 2

  @property
  def loopback(self) -> int:
    return 128
