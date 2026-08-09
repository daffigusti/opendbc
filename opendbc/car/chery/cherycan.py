from opendbc.car import CanBusBase

STEER_ANGLE_OFFSET = -392
STEER_ANGLE_SCALE = 10
CRC_POLY = 0x1D
CRC_INIT = 0xFF
CRC_XOR = 0xFF
ACCEL_MIN = -3.5
ACCEL_ZERO = 0.0
ACCEL_MAX = 2.0
CMD_MIN = -511
CMD_ZERO = -24
CMD_MAX = 511


def calculate_crc(data: bytes) -> int:
  crc = CRC_INIT
  for byte in data:
    crc ^= byte
    for _ in range(8):
      crc = ((crc << 1) ^ CRC_POLY) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc ^ CRC_XOR


def steering_command(angle: float, lkas_enable: bool) -> int:
  command = int(round(angle * STEER_ANGLE_SCALE + STEER_ANGLE_OFFSET))
  return 2 if lkas_enable and 0 <= command <= 2 else command


def steering_angle(command: int) -> float:
  return (command - STEER_ANGLE_OFFSET) / STEER_ANGLE_SCALE


def quantize_steering_angle(angle: float, lkas_enable: bool) -> float:
  return steering_angle(steering_command(angle, lkas_enable))


def limit_active_steering_angle(angle: float, apply_angle_last: float) -> float:
  """Rate-limit active steering in encoded command space, avoiding reserved commands."""
  previous_raw = steering_command(apply_angle_last, False)
  target_raw = int(round(angle * STEER_ANGLE_SCALE + STEER_ANGLE_OFFSET))
  lower_raw, upper_raw = previous_raw - 50, previous_raw + 50
  command = max(lower_raw, min(target_raw, upper_raw))

  if command in (0, 1):
    candidates = [raw for raw in (-1, 2) if lower_raw <= raw <= upper_raw]
    command = min(candidates, key=lambda raw: (abs(raw - target_raw), raw > target_raw))

  return steering_angle(command)


def create_steering_control(packer, bus: int, apply_steer: float, lkas_enable: bool, stock_values: dict):
  command = steering_command(apply_steer, lkas_enable)
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


def create_acc_control(packer, bus: int, stock_values: dict, long_active: bool,
                       gas: float, full_stop: bool, resume: bool):
  if long_active:
    gas = max(ACCEL_MIN, min(gas, ACCEL_MAX))
    if gas <= ACCEL_ZERO:
      throttle = CMD_MIN + (gas - ACCEL_MIN) * (CMD_ZERO - CMD_MIN) / (ACCEL_ZERO - ACCEL_MIN)
    else:
      throttle = CMD_ZERO + gas * (CMD_MAX - CMD_ZERO) / ACCEL_MAX
    throttle = int(round(throttle))
  else:
    throttle = CMD_ZERO
  values = {name: stock_values[name] for name in (
    "ACC_STATE_2", "NEW_SIGNAL_12", "NEW_SIGNAL_9", "NEW_SIGNAL_2", "STOPPING",
    "NEW_SIGNAL_13", "NEW_SIGNAL_8", "NEW_SIGNAL_5", "NEW_SIGNAL_6", "NEW_SIGNAL_10",
    "NEW_SIGNAL_3", "NEW_SIGNAL_4", "AEB_REQ_STOP",
  )}
  values.update({
    "CMD": throttle,
    "ACCEL_ON": 1 if throttle >= 0 else 0,
    "ACC_STATE": 2 if long_active and full_stop else 3 if long_active else stock_values["ACC_STATE"],
    "STOPPED": 1 if long_active and full_stop else 0 if long_active else stock_values["STOPPED"],
    "STOPPING": stock_values["STOPPING"],
    # Keep OEM AEB request path authoritative; host never requests AEB stop.
    "AEB_REQ_STOP": 0,
    "GAS_PRESSED": 1 if long_active and resume else 0,
    "COUNTER": stock_values["COUNTER"],
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
