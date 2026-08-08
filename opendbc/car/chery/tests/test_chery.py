from opendbc.car import Bus
from opendbc.can import CANPacker
from opendbc.car.chery.fingerprints import FINGERPRINTS, FW_VERSIONS
from opendbc.car.chery.values import CAR, DBC
from opendbc.car.fingerprints import _FINGERPRINTS
from opendbc.car.structs import CarParams
from opendbc.car.values import PLATFORMS


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
  signals = (
    ("STEER_ANGLE_SENSOR", "STEER_ANGLE", (-780, 858.3)),
    ("STEER_ANGLE_SENSOR", "TORQUE", (-128, 127)),
    ("STEER_SENSOR_2", "TORQUE_DRIVER", (-491.52, 491.28)),
    ("LKAS_CAM_CMD_345", "CMD", (-4096, 4095)),
    ("ACC_CMD", "CMD", (-511, 511)),
  )

  for message, signal, boundaries in signals:
    for value in boundaries:
      _, data, _ = packer.make_can_msg(message, 0, {signal: value})
      assert len(data) == 8
