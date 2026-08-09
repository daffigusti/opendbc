from pathlib import Path
from types import SimpleNamespace

import pytest

from opendbc.car import Bus
from opendbc.car import structs
from opendbc.can import CANPacker, CANParser
from opendbc.car.chery.cherycan import CanBus
from opendbc.car.chery.carcontroller import CarController
from opendbc.car.chery.carstate import CarState
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

  offset_fingerprint = fingerprint()
  offset_fingerprint[4] = {0x4B1: 8, 0x4B3: 8}
  assert (CanBus(fingerprint=offset_fingerprint).main, CanBus(fingerprint=offset_fingerprint).camera) == (4, 6)

  params = CarInterface.get_params(CAR.CHERY_OMODA_E5, offset_fingerprint, [], alpha_long=False, is_release=False, docs=False)
  assert [config.safetyModel for config in params.safetyConfigs] == [
    structs.CarParams.SafetyModel.noOutput,
    structs.CarParams.SafetyModel.cheryCanFd,
  ]
  CarInterface.get_params_sp(params, CAR.CHERY_OMODA_E5, offset_fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
  assert params.enableBsm
  offset_parsers = CarState.get_can_parsers(params, structs.CarParamsSP())
  assert offset_parsers[Bus.pt].bus == 4
  assert offset_parsers[Bus.cam].bus == 6
  assert offset_parsers[Bus.loopback].bus == 128


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


def make_control(lat_active: bool, angle: float):
  control = structs.CarControl()
  control.latActive = lat_active
  control.actuators.steeringAngleDeg = angle
  return control.as_reader()


def make_state(measured_angle: float, speed: float = 1.0):
  state = structs.CarState()
  state.vEgo = speed
  state.vEgoRaw = speed
  state.steeringAngleDeg = measured_angle
  state.steeringTorque = 0.0
  return SimpleNamespace(
    out=state.as_reader(),
    lkas_cmd={
      "NEW_SIGNAL_5": 0,
      "NEW_SIGNAL_6": 0,
      "NEW_SIGNAL_7": 0,
      "NEW_SIGNAL_1": 0,
    },
    acc_cmd={},
    buttons_stock_values={},
  )


def make_controller():
  cp = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
  cp_sp = structs.CarParamsSP()
  return CarController(DBC[CAR.CHERY_OMODA_E5], cp, cp_sp)


def test_lateral_controller_sends_50_hz():
  controller = make_controller()
  control = make_control(True, 5.0)
  state = make_state(0.0, 10.0)
  steer_messages = 0
  for frame in range(100):
    _actuators, sends = controller.update(control, structs.CarControlSP(), state, frame * 10_000_000)
    steer_messages += sum(addr == 0x345 for addr, _dat, _bus in sends)
  assert steer_messages == 50


def test_lateral_inactive_tracks_measured_angle():
  controller = make_controller()
  measured_angle = 17.5
  actuators, _sends = controller.update(make_control(False, 80.0), structs.CarControlSP(), make_state(measured_angle), 0)
  assert actuators.steeringAngleDeg == measured_angle


def decode_steering_message(message):
  parser = CANParser("chery_canfd", [("LKAS_CAM_CMD_345", 0)], 0)
  parser.update([[0, [message]]])
  return parser.vl["LKAS_CAM_CMD_345"]


def test_lateral_first_frame_outside_angle_limit_stays_inactive():
  controller = make_controller()
  measured_angle = 151.0
  actuators, sends = controller.update(make_control(True, 80.0), structs.CarControlSP(), make_state(measured_angle), 0)

  steering_message = next(send for send in sends if send[0] == 0x345)
  values = decode_steering_message(steering_message)
  assert values["LKA_ACTIVE"] == 0
  assert values["CMD"] == int(measured_angle * 10 - 392)
  assert actuators.steeringAngleDeg == measured_angle


def test_lateral_transition_outside_angle_limit_stays_inactive_until_in_range():
  controller = make_controller()
  control = make_control(True, 80.0)

  actuators, sends = controller.update(control, structs.CarControlSP(), make_state(-151.0), 0)
  values = decode_steering_message(next(send for send in sends if send[0] == 0x345))
  assert values["LKA_ACTIVE"] == 0
  assert values["CMD"] == int(-151.0 * 10 - 392)
  assert actuators.steeringAngleDeg == -151.0

  controller.update(control, structs.CarControlSP(), make_state(0.0), 10_000_000)
  actuators, sends = controller.update(control, structs.CarControlSP(), make_state(0.0), 20_000_000)
  values = decode_steering_message(next(send for send in sends if send[0] == 0x345))
  assert values["LKA_ACTIVE"] == 1
  assert actuators.steeringAngleDeg != 0.0


def test_lateral_hard_cap_is_150_degrees():
  controller = make_controller()
  controller.apply_angle_last = 149.0
  actuators, _sends = controller.update(make_control(True, 500.0), structs.CarControlSP(), make_state(149.0), 0)
  assert abs(actuators.steeringAngleDeg) <= 150.


def test_parser_layout_matches_route():
  cp = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
  parsers = CarState.get_can_parsers(cp, structs.CarParamsSP())
  assert set(parsers) == {Bus.pt, Bus.cam, Bus.loopback}
  assert parsers[Bus.pt].bus == 0
  assert parsers[Bus.cam].bus == 2
  assert parsers[Bus.loopback].bus == 128


def test_chery_state_update_decodes_route_signals():
  cp = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
  parsers = CarState.get_can_parsers(cp, structs.CarParamsSP())
  packer = CANPacker("chery_canfd")

  messages = {
    Bus.pt: [
      ("WHEEL_SPEED_FRNT", {"WHEEL_SPEED_FR": 10, "WHEEL_SPEED_FL": 11}),
      ("WHEEL_SPEED_REAR", {"WHEEL_SPEED_RR": 12, "WHEEL_SPEED_RL": 13}),
      ("STEER_ANGLE_SENSOR", {"STEER_ANGLE": -12.3, "TORQUE": -7}),
      ("STEER_SENSOR_2", {"TORQUE_DRIVER": -24}),
      ("BRAKE_DATA", {"BRAKE_POS": 25}),
      ("ENGINE_DATA", {"GAS": 4, "BRAKE_PRESS": 1, "GEAR": 4}),
      ("STEER_BUTTON", {"ACC": 1, "RES_PLUS": 1}),
    ],
    Bus.cam: [
      ("ACC", {"ACC_ACTIVE": 1, "AEB_ACTIVE": 1}),
      ("ACC_CMD", {"STOPPED": 0, "GAS_PRESSED": 1}),
      ("SETTING", {"CC_SPEED": 72, "ACC_AVAILABLE": 1, "AEB_ACTIVE": 3}),
      ("LKAS_STATE", {"LKA_ACTIVE": 1}),
    ],
    Bus.loopback: [],
  }
  for bus, bus_messages in messages.items():
    frames = []
    for message, values in bus_messages:
      address, data, _ = packer.make_can_msg(message, parsers[bus].bus, values)
      frames.append((address, data, parsers[bus].bus))
    if frames:
      parsers[bus].update([0, frames])

  car_state = CarState(cp, structs.CarParamsSP())
  state, _ = car_state.update(parsers)
  assert state.wheelSpeeds.fl == pytest.approx(11 / 3.6, abs=1e-3)
  assert state.wheelSpeeds.fr == pytest.approx(10 / 3.6, abs=1e-3)
  assert state.wheelSpeeds.rl == pytest.approx(13 / 3.6, abs=1e-3)
  assert state.wheelSpeeds.rr == pytest.approx(12 / 3.6, abs=1e-3)
  assert state.steeringAngleDeg == pytest.approx(-12.3)
  assert state.steeringTorque == pytest.approx(-24)
  assert state.steeringTorqueEps == pytest.approx(-7)
  assert state.brakePressed
  assert state.gasPressed
  assert state.cruiseState.available
  assert state.cruiseState.enabled
  assert state.cruiseState.speed == pytest.approx(72 / 3.6)
  assert state.stockAeb
  assert state.stockFcw is False
  assert car_state.brake_pos == 25
  assert state.brakePressed
  assert state.gearShifter == structs.CarState.GearShifter.drive
  assert {str(event.type) for event in state.buttonEvents} == {"accelCruise"}
  assert car_state.lkas_cmd["CMD"] == 0
  assert car_state.acc_cmd["GAS_PRESSED"] == 1
  assert car_state.buttons_stock_values["RES_PLUS"] == 1
  assert not state.doorOpen
  assert not state.seatbeltUnlatched


def test_chery_state_update_decodes_bsm_when_enabled():
  cp = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
  cp.enableBsm = True
  parsers = CarState.get_can_parsers(cp, structs.CarParamsSP())
  packer = CANPacker("chery_canfd")
  frames = []
  for message, values in (("BSM_LEFT", {"BSM_LEFT_DETECT": 1}), ("BSM_RIGHT", {"BSM_RIGHT_DETECT": 1})):
    address, data, bus = packer.make_can_msg(message, parsers[Bus.pt].bus, values)
    frames.append((address, data, bus))
  parsers[Bus.pt].update([0, frames])
  state, _ = CarState(cp, structs.CarParamsSP()).update(parsers)
  assert state.leftBlindspot
  assert state.rightBlindspot


@pytest.mark.parametrize("gear, expected", [
  (1, structs.CarState.GearShifter.park), (2, structs.CarState.GearShifter.reverse),
  (3, structs.CarState.GearShifter.neutral), (4, structs.CarState.GearShifter.drive),
])
def test_engine_gear_values_parse(gear, expected):
  cp = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
  parsers = CarState.get_can_parsers(cp, structs.CarParamsSP())
  packer = CANPacker("chery_canfd")
  address, data, bus = packer.make_can_msg("ENGINE_DATA", parsers[Bus.pt].bus, {"GEAR": float(gear)})
  parsers[Bus.pt].update([[0, [(address, data, bus)]]])
  state, _ = CarState(cp, structs.CarParamsSP()).update(parsers)
  assert state.gearShifter == expected


@pytest.mark.parametrize("available, expected", [(0, False), (1, True), (2, True), (3, False)])
def test_acc_available_values(available, expected):
  cp = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
  parsers = CarState.get_can_parsers(cp, structs.CarParamsSP())
  packer = CANPacker("chery_canfd")
  address, data, bus = packer.make_can_msg("SETTING", parsers[Bus.cam].bus, {"ACC_AVAILABLE": float(available)})
  parsers[Bus.cam].update([[0, [(address, data, bus)]]])
  state, _ = CarState(cp, structs.CarParamsSP()).update(parsers)
  assert state.cruiseState.available is expected


@pytest.mark.parametrize("acc_aeb, setting_aeb, expected", [
  (0, 3, False),
  (1, 0, True),
  (1, 3, True),
])
def test_stock_aeb_uses_acc_signal_not_setting(acc_aeb, setting_aeb, expected):
  cp = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
  parsers = CarState.get_can_parsers(cp, structs.CarParamsSP())
  packer = CANPacker("chery_canfd")
  acc_address, acc_data, acc_bus = packer.make_can_msg(
    "ACC", parsers[Bus.cam].bus, {"AEB_ACTIVE": float(acc_aeb)},
  )
  setting_address, setting_data, setting_bus = packer.make_can_msg(
    "SETTING", parsers[Bus.cam].bus, {"AEB_ACTIVE": float(setting_aeb)},
  )
  parsers[Bus.cam].update([[0, [
    (acc_address, acc_data, acc_bus), (setting_address, setting_data, setting_bus),
  ]]])
  state, _ = CarState(cp, structs.CarParamsSP()).update(parsers)
  assert state.stockAeb is expected


@pytest.mark.parametrize("brake_pos, brake_press, expected", [(25, 0, False), (0, 1, True)])
def test_brake_state_uses_engine_switch_and_preserves_brake_position(brake_pos, brake_press, expected):
  cp = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
  parsers = CarState.get_can_parsers(cp, structs.CarParamsSP())
  packer = CANPacker("chery_canfd")
  address, data, bus = packer.make_can_msg(
    "BRAKE_DATA", parsers[Bus.pt].bus, {"BRAKE_POS": float(brake_pos)},
  )
  engine_address, engine_data, engine_bus = packer.make_can_msg(
    "ENGINE_DATA", parsers[Bus.pt].bus, {"BRAKE_PRESS": float(brake_press)},
  )
  parsers[Bus.pt].update([[0, [
    (address, data, bus), (engine_address, engine_data, engine_bus),
  ]]])
  car_state = CarState(cp, structs.CarParamsSP())
  state, _ = car_state.update(parsers)
  assert state.brakePressed is expected
  assert car_state.brake_pos == brake_pos


@pytest.mark.parametrize("active, acc_gas, engine_gas, expected", [
  (1, 1, 0, True), (1, 0, 100, False), (0, 0, 2, True), (0, 0, 1, False),
])
def test_gas_source_by_acc_active(active, acc_gas, engine_gas, expected):
  cp = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
  parsers = CarState.get_can_parsers(cp, structs.CarParamsSP())
  packer = CANPacker("chery_canfd")
  pt_addr, pt_data, pt_bus = packer.make_can_msg("ENGINE_DATA", parsers[Bus.pt].bus, {"GAS": float(engine_gas)})
  acc_addr, acc_data, acc_bus = packer.make_can_msg("ACC", parsers[Bus.cam].bus, {"ACC_ACTIVE": float(active)})
  cmd_addr, cmd_data, cmd_bus = packer.make_can_msg("ACC_CMD", parsers[Bus.cam].bus, {"GAS_PRESSED": float(acc_gas)})
  parsers[Bus.pt].update([[0, [(pt_addr, pt_data, pt_bus)]]])
  parsers[Bus.cam].update([[0, [(acc_addr, acc_data, acc_bus), (cmd_addr, cmd_data, cmd_bus)]]])
  state, _ = CarState(cp, structs.CarParamsSP()).update(parsers)
  assert state.gasPressed is expected


def test_buttons_emit_only_verified_resume_edges():
  cp = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
  parsers = CarState.get_can_parsers(cp, structs.CarParamsSP())
  packer = CANPacker("chery_canfd")
  state = CarState(cp, structs.CarParamsSP())
  for values, expected in [
    ({"RES_PLUS": 1, "RES_MINUS": 0, "ACC": 1, "CC_BTN": 1}, {"accelCruise"}),
    ({"RES_PLUS": 0, "RES_MINUS": 1, "ACC": 1, "CC_BTN": 1}, {"accelCruise", "decelCruise"}),
    ({"RES_PLUS": 0, "RES_MINUS": 0, "ACC": 0, "CC_BTN": 0}, {"decelCruise"}),
  ]:
    address, data, bus = packer.make_can_msg("STEER_BUTTON", parsers[Bus.pt].bus, values)
    parsers[Bus.pt].update([[0, [(address, data, bus)]]])
    events = state.update(parsers)[0].buttonEvents
    assert {str(event.type) for event in events} == expected
