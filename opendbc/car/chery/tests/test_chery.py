from pathlib import Path

import pytest

from opendbc.car import Bus
from opendbc.car import structs
from opendbc.can import CANPacker, CANParser
from opendbc.car.chery.cherycan import CanBus
from opendbc.car.chery.fingerprints import FINGERPRINTS, FW_VERSIONS
from opendbc.car.chery.interface import CarInterface
from opendbc.car.chery.values import CAR, CherySafetyFlags, DBC
from opendbc.car.fingerprints import _FINGERPRINTS
from opendbc.car.structs import CarParams
from opendbc.car.values import PLATFORMS


def fingerprint():
  return {bus: {} for bus in range(8)}


def test_interface_lateral_and_alpha_long():
  lateral = CarInterface.get_params(CAR.CHERY_OMODA_E5, fingerprint(), [], alpha_long=False, is_release=False, docs=False)
  assert lateral.brand == "chery"
  assert lateral.steerControlType == structs.CarParams.SteerControlType.angle
  assert lateral.transmissionType == structs.CarParams.TransmissionType.direct
  assert lateral.alphaLongitudinalAvailable
  assert not lateral.openpilotLongitudinalControl
  assert lateral.safetyConfigs[-1].safetyParam == 0

  long = CarInterface.get_params(CAR.CHERY_OMODA_E5, fingerprint(), [], alpha_long=True, is_release=False, docs=False)
  assert long.openpilotLongitudinalControl
  assert long.safetyConfigs[-1].safetyParam & CherySafetyFlags.LONG_CONTROL


def test_can_bus_offsets():
  assert (CanBus(fingerprint=fingerprint()).main, CanBus(fingerprint=fingerprint()).camera) == (0, 2)


def test_chery_platform_registered():
  assert CAR.CHERY_OMODA_E5 in PLATFORMS.values()
  assert DBC[CAR.CHERY_OMODA_E5][Bus.pt] == "chery_canfd"
  assert int(CarParams.SafetyModel.cheryCanFd) == 35
  assert FINGERPRINTS[CAR.CHERY_OMODA_E5]
  assert _FINGERPRINTS[str(CAR.CHERY_OMODA_E5)] == FINGERPRINTS[CAR.CHERY_OMODA_E5]
  assert FW_VERSIONS[CAR.CHERY_OMODA_E5]
  assert b"00.02.12" in FW_VERSIONS[CAR.CHERY_OMODA_E5][(CarParams.Ecu.engine, 0x7e0, None)]


def test_chery_signed_signal_boundaries():
  packer = CANPacker("chery_canfd")
  dbc = packer.dbc
  signals = (
    ("STEER_ANGLE_SENSOR", "STEER_ANGLE", (-780, 858.3), False),
    ("STEER_ANGLE_SENSOR", "TORQUE", (-128, 127), True),
    ("STEER_SENSOR_2", "TORQUE_DRIVER", (-491.52, 491.28), True),
    ("LKAS_CAM_CMD_345", "CMD", (-4096, 4095), True),
    ("ACC_CMD", "CMD", (-511, 511), True),
  )

  for message, signal, boundaries, is_signed in signals:
    sig = dbc.name_to_msg[message].sigs[signal]
    assert sig.is_signed is is_signed
    raw_boundaries = (-(1 << (sig.size - 1)), (1 << (sig.size - 1)) - 1) if is_signed else (0, (1 << sig.size) - 1)
    assert all(raw_boundaries[0] <= round((value - sig.offset) / sig.factor) <= raw_boundaries[1] for value in boundaries)
    parser = CANParser("chery_canfd", [(message, 0)], 0)
    for value in boundaries:
      address, data, bus = packer.make_can_msg(message, 0, {signal: value})
      parser.update([0, [(address, data, bus)]])
      assert parser.vl[message][signal] == pytest.approx(value)


def test_chery_signal_ranges_declared_in_dbc():
  dbc_lines = set((Path(__file__).parents[3] / "dbc/chery_canfd.dbc").read_text().splitlines())
  assert {
    " SG_ STEER_ANGLE : 7|14@0+ (0.1,-780) [-780|858.3] \"\" XXX",
    " SG_ TORQUE : 16|8@1- (1,0) [-128|127] \"\" XXX",
    " SG_ TORQUE_DRIVER : 7|12@0- (0.24,0) [-491.52|491.28] \"\" XXX",
    " SG_ CMD : 6|13@0- (1,0) [-4096|4095] \"\" XXX",
    " SG_ CMD : 6|10@0- (1,0) [-511|511] \"\" XXX",
    " SG_ ACC_STATE : 9|2@0+ (1,0) [0|3] \"\" XXX",
  } <= dbc_lines
