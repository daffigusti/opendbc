from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, DT_CTRL, PlatformConfig, Platforms
from opendbc.car.lateral import AngleSteeringLimitsVM
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts, SupportType
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries


Ecu = CarParams.Ecu


class CarControllerParams:
  STEER_STEP = 2
  ACC_CONTROL_STEP = 2
  BUTTONS_STEP = 5
  LKAS_HUD_STEP = 5
  # MAX_ANGLE_RATE only binds below ~32 kph, where the VM jerk limit is looser. At 5 deg/frame
  # (250 deg/s) an engagement onto a 22 deg request stepped the wheel 23 deg in 80ms. The sharpest
  # turn-in openpilot asked for in real routes needed ~75 deg/s, so 100 deg/s leaves it untouched.
  # STEER_ANGLE_MAX: the working fork drove at 300 for months; the 13-bit CMD encoding tops out at
  # 370.4. Above ~25 kph the VM lateral-accel limit binds well before this cap does. Panda must match.
  ANGLE_LIMITS = AngleSteeringLimitsVM(STEER_ANGLE_MAX=300., MAX_ANGLE_RATE=2.)
  ACCEL_MIN = -3.5
  ACCEL_MAX = 2.0
  RAW_ACCEL_MIN = -511
  RAW_ACCEL_MAX = 511
  RAW_ACCEL_INACTIVE = -24

  # RES+ doubles as "raise set speed": held while ACC_ACTIVE is 1 it adds +1..+14 kph, while the
  # 0.19-0.24s taps the driver makes at ACC_ACTIVE 0 only resume. Tap it the way the driver does.
  RESUME_TAP_FRAMES = 4   # 4 button frames at 20Hz = 200ms
  RESUME_TAP_PERIOD = 14  # 700ms cycle, leaving a clear gap between taps
  # Cancel reuses this cadence. STEER_BUTTON.ACC toggles the ACC, cancelling while active and engaging
  # while not; on a real route ACC_ACTIVE dropped 0.1-0.25s after a press, well inside one cycle, so a
  # second tap never lands on an already-cancelled ACC.

  # STEER_SENSOR_2.TORQUE_DRIVER is 0.24 units; 70 is the threshold the working fork ran with.
  # Unverified against a labelled stationary sweep -- see KNOWN_GAPS.md.
  STEER_THRESHOLD = 70.
  # Driver has to hold past this before lateral drops out, and hold off it that long to get it back.
  STEER_OVERRIDE_TIME = 1.0
  # Maximum time the EPS may report a dead LKAS_CMD while openpilot is steering.
  STEER_TIMEOUT = int(30 / DT_CTRL)

  def __init__(self, CP):
    pass


class CherySafetyFlags(IntFlag):
  LONG_CONTROL = 1


@dataclass
class CheryCarDocs(CarDocs):
  package: str = "All"
  support_type: SupportType = SupportType.REVIEW
  support_link: str | None = "#under-review"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


@dataclass
class CheryPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: "chery_canfd"})


class CAR(Platforms):
  CHERY_OMODA_E5 = CheryPlatformConfig(
    [CheryCarDocs("Chery Omoda E5 2024", video="https://youtu.be/9kGGh8sLcHc")],
    # steerRatio fitted from locationd yaw rate against measured wheel angle (r=0.98, 4100 samples,
    # 16.8-17.2 across 11-32 kph). At 14, cars achieved 92% of requested curvature in turns and
    # paramsd was still crawling upward at 15.6. Panda's steer_ratio must match.
    CarSpecs(mass=1785., wheelbase=2.63, steerRatio=17., centerToFrontRatio=0.44)
  )


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    )
  ],
  # The three shapes seen on the E5 engine ECU so far: plain version, masked, and UDS-prefixed part number
  fw_version_regex=br"(?:\d{2}\.\d{2}\.\d{2}|\?{10}|\xf1\x87[0-9A-Z]{11} {5}\xf1\x82\?{10})",
)


DBC = CAR.create_dbc_map()
