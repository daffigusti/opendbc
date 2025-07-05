from opendbc.car import CanBusBase, structs
HUDControl = structs.CarControl.HUDControl

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

def calculate_crc(data, poly, xor_output):
  crc = 0
  for byte in data:
    crc ^= byte
    for _ in range(8):
      if crc & 0x80:
        crc = (crc << 1) ^ poly
      else:
        crc <<= 1
      crc &= 0xFF
  return (crc ^ xor_output)

def create_longitudinal_control(packer, bus, acc, frame, long_active: bool, gas: float, accel: float, stopping: bool, full_stop : bool, resume : bool):
  throtle = gas if long_active else -24
  # if full stop cmd = 400, acc_state = 2, and stopped = 1
  acc_state = 2 if full_stop else 3 if long_active else acc['ACC_STATE']
  values = {
      "CMD": 400 if full_stop else throtle,
      "ACCEL_ON": 1 if throtle>= 0 else 0,
      "ACC_STATE": acc_state, # 1 not available, 2 available, 3 active
      "STOPPED": 1 if full_stop else 0 if long_active else  acc['STOPPED'],
      "ACC_STATE_2": acc['ACC_STATE_2'],
      "NEW_SIGNAL_12": acc['NEW_SIGNAL_12'],
      "NEW_SIGNAL_9": acc['NEW_SIGNAL_9'],
      "NEW_SIGNAL_2": acc['NEW_SIGNAL_2'],
      # "STOPPING": 1 if throtle>= 280 else 0,
      "STOPPING": acc['STOPPING'],
      "NEW_SIGNAL_13": acc['NEW_SIGNAL_13'],
      "NEW_SIGNAL_8": acc['NEW_SIGNAL_8'],
      "NEW_SIGNAL_5": acc['NEW_SIGNAL_5'],
      "NEW_SIGNAL_6": acc['NEW_SIGNAL_6'],
      "NEW_SIGNAL_10": acc['NEW_SIGNAL_10'],
      "NEW_SIGNAL_3": acc['NEW_SIGNAL_3'],
      "NEW_SIGNAL_4": acc['NEW_SIGNAL_4'],
      "NEW_SIGNAL_11": acc['NEW_SIGNAL_11'],
      "GAS_PRESSED": 1 if resume else 0,  # gas pressed
      "COUNTER": (frame) % 0x0f,
      # "STEER_REQUEST": steer_req,
  }
  # values["COUNTER"] = (values["COUNTER"] + 1) % 0x0f

  dat = packer.make_can_msg("ACC_CMD", bus, values)[1]

  crc = calculate_crc(dat[:-1], 0x1D, 0xA)
  values["CHECKSUM"] = crc

  if long_active:
    print("Accel:",gas)
  # print("Acc Ori:",acc)
    # print("Send valud:",values)

  return packer.make_can_msg("ACC_CMD", bus, values)

def create_longitudinal_controlBypass(packer, bus, acc, frame):

  return packer.make_can_msg("ACC_CMD", bus, acc)


def create_steering_control_lkas(packer, bus: int, apply_steer, frame, lkas_enable, lkas):
  # idx = (apply_steer) % 1000
  apply_steer = int((apply_steer*10)-392)
  # apply_steer = int((apply_steer+780)*10)
  if apply_steer>= 0 and apply_steer <=2 :
    apply_steer = 2
  values = {
      "CMD": apply_steer,
      "NEW_SIGNAL_3": 1 if (apply_steer)>1 else 0,
      # "LKA_ACTIVE":  1 if (apply_steer) else 0,
      "LKA_ACTIVE": 1 if lkas_enable else 0,
      # "LKA_ACTIVE":  0,
      "SET_X0": 0,
      "NEW_SIGNAL_5": lkas["NEW_SIGNAL_5"],
      "NEW_SIGNAL_6": lkas["NEW_SIGNAL_6"],
      "NEW_SIGNAL_7": lkas["NEW_SIGNAL_7"],
      "NEW_SIGNAL_1": lkas["NEW_SIGNAL_1"],
      "CHECKSUM": lkas["CHECKSUM"],
      # "COUNTER": (frame) % 0x0f,
      # "STEER_REQUEST": steer_req,
  }
  # values["COUNTER"] = (values["COUNTER"] + 1) % 0x0f

  dat = packer.make_can_msg("LKAS_CAM_CMD_345", bus, values)[1]

  crc = calculate_crc(dat[:-1], 0x1D, 0xA)
  values["CHECKSUM"] = crc

  # print("Lkas:",lkas)
  # print("Send valud:",values)
  # if lkas_enable:
  #   print("Applly Steer:",values['CMD'])

  # return to stock values if not enable
  if not lkas_enable:
    values = lkas

  return packer.make_can_msg("LKAS_CAM_CMD_345", bus, values)


def create_lkas_state_hud(packer, bus: int, frame, lkas, lkas_active):

  values = {
      "NEW_SIGNAL_1": 1,
      "NEW_SIGNAL_2": 2 if lkas_active else 0,
      "NEW_SIGNAL_3": 2 if lkas_active else 0,
      "NEW_SIGNAL_4": 1,
      "STATE": 0 if lkas_active else 1,
      "LKA_ACTIVE": 1 if lkas_active else 0,
      "COUNTER": (frame) % 0x0f,
  }

  dat = packer.make_can_msg("LKAS_STATE", bus, values)[1]

  crc = calculate_crc(dat[:-1], 0x1D, 0xA)
  values["CHECKSUM"] = crc

  return packer.make_can_msg("LKAS_STATE", bus, values if lkas_active else lkas)

def create_button_msg(packer, bus: int,frame, stock_values: dict, cancel=False, resume=False):
  """
  Creates a CAN message for buttons/switches.

  Includes cruise control buttons.

  Frequency is 20Hz.
  """
  # print(bus)
  values = {s: stock_values[s] for s in [
    "ACC",
    "CC_BTN",
    "RES_PLUS",
    "RES_MINUS",
    "NEW_SIGNAL_1",
    "GAP_ADJUST_UP",
    "GAP_ADJUST_DOWN",
    "COUNTER",
    "CHECKSUM",
  ]}

  values.update({
    "ACC": 1 if cancel else 0,      # CC cancel button
    "RES_PLUS": 1 if resume else 0,      # CC resume button
    # "COUNTER": (frame) % 0x0f,
  })

  dat = packer.make_can_msg("STEER_BUTTON", bus, values)[1]

  crc = calculate_crc(dat[:-1], 0x1D, 0xA)
  values["CHECKSUM"] = crc

  # print(values)

  return packer.make_can_msg("STEER_BUTTON", bus, values)
