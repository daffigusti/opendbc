from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.lateral import AngleSteeringLimitsVM
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts, SupportType
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries


Ecu = CarParams.Ecu


class CarControllerParams:
  STEER_STEP = 2
  ACC_CONTROL_STEP = 2
  BUTTONS_STEP = 5
  ANGLE_LIMITS = AngleSteeringLimitsVM(STEER_ANGLE_MAX=150., MAX_ANGLE_RATE=5.)
  ACCEL_MIN = -3.5
  ACCEL_MAX = 2.0
  RAW_ACCEL_MIN = -511
  RAW_ACCEL_MAX = 511
  RAW_ACCEL_INACTIVE = -24

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
    CarSpecs(mass=1785., wheelbase=2.63, steerRatio=17.5)
  )


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    )
  ],
)


DBC = CAR.create_dbc_map()
