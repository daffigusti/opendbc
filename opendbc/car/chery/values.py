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
  # STEER_ANGLE_MAX: 300 capped 90 deg turns at ~8.3 m radius; 360 reaches ~7 m. The 13-bit CMD
  # encoding tops out at 370.4. Above ~25 kph the VM lateral-accel limit binds well before this cap does. Panda must match.
  ANGLE_LIMITS = AngleSteeringLimitsVM(STEER_ANGLE_MAX=360., MAX_ANGLE_RATE=2.)
  # The model's desired angle carries a ~2.7 Hz oscillation at low speed and the angle EPS reproduces
  # it as a wobbly wheel. Smoothing it costs turn-in, one against the other: replayed over route
  # 0000049e (36 min lateral) for jitter and the 7 tight turns of route 000004ad seg 5 (6-15 kph) for
  # turn-in, jitter RMS below 20 kph and the share of the requested peak angle that survives run
  # 1.22 deg / 100% raw, 0.61 / 83% at tau 0.2, 0.46 / 78% here, 0.37 / 73% at 0.4, and 0.54 / 81%
  # for the 0.25 s faded by 30 kph this replaced. 0.4 was driven and read as calm but late in tight
  # turns. Neither a median prefilter (an oscillation is not a spike: 1.20 RMS) nor letting a large
  # angle error through unfiltered (jitter is that large: 0.67 RMS) separates the two.
  # Lag is tau below 10 kph, but bounded by ANGLE_FILTER_MAX_LAG wherever the model outruns it.
  # ponytail: still wobbly below 20 kph -> tau 0.6; still late in tight turns -> max lag 5.
  ANGLE_FILTER_SPEED_BP = [10 / 3.6, 40 / 3.6]
  ANGLE_FILTER_TAU = [0.5, 0.]
  # Route 000004ad seg 10 turned a hairpin at 22 kph: the model swung 165 deg in 3.5 s and unwound
  # at ~130 deg/s, which tau alone trailed by 29 deg p95 and read to openpilot as the car failing to
  # turn (steerSaturated, "Turn Exceeds Steering Limit"). Capping the lag at 6 deg -- twice the
  # jitter this filter exists to remove -- holds it to 7.7 deg p95 there while low-speed jitter stays
  # where tau 0.4 put it (0.37 deg RMS on route 0000049e) and tight-turn peaks improve on it.
  ANGLE_FILTER_MAX_LAG = 6.
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

  # SETTING.GAP runs 1..5, 5 the farthest; GAP_ADJUST_UP moves it farther. openpilot's following
  # distance is narrower than stock's, so its three personalities sit on the middle levels.
  # Measured only for GAP 5: stock ACC held ~41 m (LEAD_FRONT) / ~42 m (model) at 64 kph, a ~2.35 s gap,
  # already farther than relaxed's 1.75 s * v + 6 m (~37 m there). Route 00000488 ran at GAP 5 throughout.
  # ponytail: levels 1-4 still mapped by eye; refit from LEAD_FRONT distance / vEgo on a route that uses them.
  GAP_FOR_DISTANCE_BARS = {1: 2, 2: 3, 3: 4}  # aggressive, standard, relaxed

  # Every STEER_BUTTON frame openpilot injects is its own press: the panda keeps forwarding the
  # wheel's frame (buttons 0) between them, so the camera sees a rising edge per injected frame.
  # Route 000004ae ran the 4-frame resume cadence here, the cluster stepped GAP once per frame, and
  # the 10Hz SETTING.GAP readback -- still stale when the next frame went out -- kept flipping the
  # direction: GAP ping-ponged 1<->3 for 7s off a single driver tap. One frame per press, and wait
  # for the readback before deciding again.
  GAP_TAP_PERIOD = 14  # 700ms between presses; SETTING.GAP settled within ~250ms on that route
  GAP_MAX_TAPS = 6     # 1..5 is 4 levels at most, so stop hunting rather than press forever

  # STEER_SENSOR_2.TORQUE_DRIVER is 0.24 units; 70 is the threshold the working fork ran with.
  # Unverified against a labelled stationary sweep -- see KNOWN_GAPS.md.
  STEER_THRESHOLD = 70.
  # Driver has to hold past this before lateral drops out, and hold off it that long to get it back.
  STEER_OVERRIDE_TIME = 1.0
  # The EPS latches itself off when the driver pushes past ~300 TORQUE_DRIVER and only listens again
  # after LKA_ACTIVE drops and rises (route 0000049e: 3 of 3 dropouts, 39 of 39 recoveries, back
  # 30-40 ms after the rising edge). Engagement and one-frame blips stay under 40 ms.
  EPS_LATCH_TIME = 0.2
  # Shortest LKA_ACTIVE gap the route proved re-arms the EPS; nothing shorter was ever sent.
  # ponytail: try shorter on the car, the gap is lateral the driver goes without.
  EPS_REARM_TIME = 1.0
  # Commanded frames the EPS may stay inactive before a steer fault is raised: ~3 failed re-arms.
  STEER_TIMEOUT = int(0.6 / DT_CTRL)

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
