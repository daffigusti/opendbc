from pathlib import Path
from types import SimpleNamespace

import pytest

from opendbc.car import Bus, DT_CTRL
from opendbc.car import structs
from opendbc.can import CANPacker, CANParser
from opendbc.car.chery.cherycan import CanBus, calculate_crc, create_hud_alert
from opendbc.car.chery.carcontroller import CarController
from opendbc.car.chery.carstate import CarState
from opendbc.car.chery.fingerprints import FINGERPRINTS, FW_VERSIONS
from opendbc.car.chery.interface import CarInterface
from opendbc.car.chery.values import CAR, CarControllerParams, CherySafetyFlags, DBC
from opendbc.car.fingerprints import _FINGERPRINTS
from opendbc.car.structs import CarParams
from opendbc.car.values import PLATFORMS
from opendbc.car.vehicle_model import calc_slip_factor
from opendbc.car.vehicle_model import VehicleModel


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
  assert calc_slip_factor(VehicleModel(lateral)) == pytest.approx(-0.0006377498827871491)

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
  assert int(CarParams.SafetyModel.cheryCanFd) == 39
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
    " SG_ COUNTER : 48|4@1+ (1,0) [0|15] \"\" XXX",
    " SG_ CHECKSUM : 56|8@1+ (1,0) [0|255] \"\" XXX",
    " SG_ ENGINE_DATA_CHECKSUM_0 : 0|8@1+ (1,0) [0|255] \"\" XXX",
    " SG_ ENGINE_DATA_COUNTER_0 : 8|4@1+ (1,0) [0|15] \"\" XXX",
    " SG_ ENGINE_DATA_CHECKSUM_1 : 64|8@1+ (1,0) [0|255] \"\" XXX",
    " SG_ ENGINE_DATA_COUNTER_1 : 72|4@1+ (1,0) [0|15] \"\" XXX",
    " SG_ ENGINE_DATA_CHECKSUM_2 : 128|8@1+ (1,0) [0|255] \"\" XXX",
    " SG_ ENGINE_DATA_COUNTER_2 : 136|4@1+ (1,0) [0|15] \"\" XXX",
    " SG_ ENGINE_DATA_CHECKSUM_3 : 192|8@1+ (1,0) [0|255] \"\" XXX",
    " SG_ ENGINE_DATA_COUNTER_3 : 200|4@1+ (1,0) [0|15] \"\" XXX",
    " SG_ ENGINE_DATA_CHECKSUM_4 : 256|8@1+ (1,0) [0|255] \"\" XXX",
    " SG_ ENGINE_DATA_COUNTER_4 : 264|4@1+ (1,0) [0|15] \"\" XXX",
  } <= dbc_lines


def make_control(lat_active: bool, angle: float, long_active: bool = False, visual_alert=None):
  control = structs.CarControl()
  control.latActive = lat_active
  control.longActive = long_active
  if visual_alert is not None:
    control.hudControl.visualAlert = visual_alert
  control.actuators.steeringAngleDeg = angle
  return control.as_reader()


def make_state(measured_angle: float, speed: float = 1.0, front_wheel_speed: float | None = None,
               steering_torque: float = 0.0, standstill: bool = False, acc_active: bool = False):
  state = structs.CarState()
  state.vEgo = speed
  state.vEgoRaw = speed
  state.steeringAngleDeg = measured_angle
  state.steeringTorque = steering_torque
  state.standstill = standstill
  return SimpleNamespace(
    front_wheel_speed=speed if front_wheel_speed is None else front_wheel_speed,
    out=state.as_reader(),
    acc_active=acc_active,
    eps_inactive=False,
    gap_setting=3,
    lkas_cmd={name: 0 for name in (
      "CMD", "NEW_SIGNAL_3", "LKA_ACTIVE", "NEW_SIGNAL_2", "SET_X0",
      "NEW_SIGNAL_5", "NEW_SIGNAL_6", "NEW_SIGNAL_7", "NEW_SIGNAL_1", "CHECKSUM",
    )},
    lkas_state={name: 0 for name in (
      "NEW_SIGNAL_1", "NEW_SIGNAL_2", "NEW_SIGNAL_3", "NEW_SIGNAL_4",
      "STATE", "LKA_ACTIVE", "COUNTER", "CHECKSUM",
    )},
    hud_alert={name: 0 for name in (
      "NEW_SIGNAL_6", "NEW_SIGNAL_7", "ICA_WARNING", "NEW_SIGNAL_4",
      "STEER_WARNING", "NEW_SIGNAL_3", "COUNTER", "CHECKSUM",
    )},
    acc_cmd={},
    buttons_stock_values={name: 0 for name in (
      "ACC", "CC_BTN", "RES_PLUS", "RES_MINUS", "NEW_SIGNAL_1",
      "GAP_ADJUST_UP", "GAP_ADJUST_DOWN", "COUNTER",
    )},
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


def test_longitudinal_alpha_gate_and_50hz_output_shape():
  stock = {name: 0 for name in (
    "ACC_STATE", "STOPPED", "ACC_STATE_2", "NEW_SIGNAL_12", "NEW_SIGNAL_9",
    "NEW_SIGNAL_2", "STOPPING", "NEW_SIGNAL_13", "NEW_SIGNAL_8", "NEW_SIGNAL_5",
    "NEW_SIGNAL_6", "NEW_SIGNAL_10", "NEW_SIGNAL_3", "NEW_SIGNAL_4", "AEB_REQ_STOP", "COUNTER",
  )}
  for alpha_long, expected_count in ((False, 0), (True, 50)):
    controller = make_controller()
    controller.CP.openpilotLongitudinalControl = alpha_long
    control = make_control(False, 0.0, long_active=True)
    state = make_state(0.0)
    state.acc_cmd = stock
    sends = []
    for frame in range(100):
      _actuators, frame_sends = controller.update(control, structs.CarControlSP(), state, frame * 10_000_000)
      sends.extend(frame_sends)
    acc_sends = [send for send in sends if send[0] == 0x3A2]
    assert len(acc_sends) == expected_count
    assert all(bus == 0 and len(data) == 8 for _addr, data, bus in acc_sends)


def test_lateral_inactive_tracks_measured_angle():
  controller = make_controller()
  measured_angle = 17.5
  actuators, _sends = controller.update(make_control(False, 80.0), structs.CarControlSP(), make_state(measured_angle), 0)
  assert actuators.steeringAngleDeg == pytest.approx(measured_angle)


def decode_steering_message(message):
  parser = CANParser("chery_canfd", [("LKAS_CAM_CMD_345", 0)], 0)
  parser.update([[0, [message]]])
  return parser.vl["LKAS_CAM_CMD_345"]


def stock_steering(raw):
  parser = CANParser("chery_canfd", [("LKAS_CAM_CMD_345", 2)], 2)
  parser.update([[0, [(0x345, raw, 2)]]])
  return parser.vl["LKAS_CAM_CMD_345"].copy()


# Real camera frames: stock lane keeping active, and one setting the bit 8 the DBC once left undefined.
@pytest.mark.parametrize("raw", ["7a6600000000abf5", "7c1b00fee73c4a3b", "78c4000cbefe2f27"])
def test_stock_steering_is_relayed_byte_exact_while_lateral_is_off(raw):
  raw = bytes.fromhex(raw)
  controller = make_controller()
  state = make_state(0.0, 10.0)
  state.lkas_cmd = stock_steering(raw)
  _actuators, sends = controller.update(make_control(False, 80.0), structs.CarControlSP(), state, 0)
  assert next(send for send in sends if send[0] == 0x345) == (0x345, raw, 0)


def test_stock_steering_is_relayed_while_the_driver_overrides():
  controller = make_controller()
  control = make_control(True, 30.0)
  raw = bytes.fromhex("7a6600000000abf5")
  for frame in range(161):
    state = make_state(0.0, 10.0, steering_torque=CarControllerParams.STEER_THRESHOLD + 1)
    state.lkas_cmd = stock_steering(raw)
    _actuators, sends = controller.update(control, structs.CarControlSP(), state, frame * 10_000_000)
  assert controller.steer_override
  assert any(send == (0x345, raw, 0) for send in sends)


def test_openpilot_steering_resumes_from_the_stock_angle():
  controller = make_controller()
  state = make_state(0.0, 60 / 3.6)
  state.lkas_cmd = stock_steering(bytes.fromhex("7a6600000000abf5"))
  actuators, _sends = controller.update(make_control(False, 0.0), structs.CarControlSP(), state, 0)
  assert actuators.steeringAngleDeg == pytest.approx(3.3)

  controller.update(make_control(True, 3.3), structs.CarControlSP(), state, 10_000_000)
  _actuators, sends = controller.update(make_control(True, 3.3), structs.CarControlSP(), state, 20_000_000)
  values = decode_steering_message(next(send for send in sends if send[0] == 0x345))
  assert values["LKA_ACTIVE"] == 1
  assert values["CMD"] == round(3.3 * 10 - 392)


def test_lateral_first_frame_outside_angle_limit_stays_inactive():
  controller = make_controller()
  measured_angle = 361.0
  actuators, sends = controller.update(make_control(True, 80.0), structs.CarControlSP(), make_state(measured_angle), 0)

  steering_message = next(send for send in sends if send[0] == 0x345)
  values = decode_steering_message(steering_message)
  assert values["LKA_ACTIVE"] == 0
  assert values["CMD"] == round(measured_angle * 10 - 392)
  assert actuators.steeringAngleDeg == pytest.approx(measured_angle)


@pytest.mark.parametrize("measured_angle", [-370.4, 370.4])
def test_lateral_representable_boundary_is_transmitted(measured_angle):
  controller = make_controller()
  actuators, sends = controller.update(make_control(True, 80.0), structs.CarControlSP(), make_state(measured_angle), 0)

  steering_message = next(send for send in sends if send[0] == 0x345)
  values = decode_steering_message(steering_message)
  assert values["LKA_ACTIVE"] == 0
  assert values["CMD"] == round(measured_angle * 10 - 392)
  assert actuators.steeringAngleDeg == pytest.approx(measured_angle)


@pytest.mark.parametrize("measured_angle", [-370.5, 370.5])
def test_lateral_unrepresentable_boundary_is_not_transmitted(measured_angle):
  controller = make_controller()
  actuators, sends = controller.update(make_control(True, 80.0), structs.CarControlSP(), make_state(measured_angle), 0)

  assert not any(send[0] == 0x345 for send in sends)
  assert actuators.steeringAngleDeg == measured_angle


@pytest.mark.parametrize("sign", [-1, 1])
def test_lateral_resume_sends_inactive_measured_angle_before_activation(sign):
  controller = make_controller()
  control = make_control(True, sign * 100.0)

  _actuators, sends = controller.update(control, structs.CarControlSP(), make_state(sign * 500.0), 0)
  assert not any(send[0] == 0x345 for send in sends)
  controller.update(control, structs.CarControlSP(), make_state(sign * 500.0), 10_000_000)

  actuators, sends = controller.update(control, structs.CarControlSP(), make_state(sign * 100.0), 20_000_000)
  values = decode_steering_message(next(send for send in sends if send[0] == 0x345))
  assert values["LKA_ACTIVE"] == 0
  assert values["CMD"] == round(sign * 100.0 * 10 - 392)
  assert actuators.steeringAngleDeg == sign * 100.0

  controller.update(control, structs.CarControlSP(), make_state(sign * 100.0), 30_000_000)
  actuators, sends = controller.update(control, structs.CarControlSP(), make_state(sign * 100.0), 40_000_000)
  values = decode_steering_message(next(send for send in sends if send[0] == 0x345))
  assert values["LKA_ACTIVE"] == 1
  assert abs(values["CMD"] + 392 - round(actuators.steeringAngleDeg * 10)) <= 1


@pytest.mark.parametrize("requested", [34.3, 44.3, -34.3, -44.3])
def test_lateral_limiter_starts_from_encoded_angle(requested):
  controller = make_controller()
  starting_angle = 39.4 if requested > 0 else -39.4
  controller.update(make_control(True, starting_angle), structs.CarControlSP(), make_state(starting_angle, speed=10.0), 0)
  control = make_control(True, requested)
  state = make_state(starting_angle, speed=10.0)

  controller.update(control, structs.CarControlSP(), state, 10_000_000)
  actuators, sends = controller.update(control, structs.CarControlSP(), state, 20_000_000)
  values = decode_steering_message(next(send for send in sends if send[0] == 0x345))
  assert values["LKA_ACTIVE"] == 1
  assert abs((values["CMD"] + 392) / 10 - starting_angle) <= 5.0
  assert actuators.steeringAngleDeg == pytest.approx((values["CMD"] + 392) / 10)


@pytest.mark.parametrize("starting_angle, requested_angle", [
  (34.2, 39.2),
  (34.3, 39.3),
  (44.2, 39.2),
  (44.3, 39.3),
])
def test_active_encoded_commands_rate_limit_across_reserved_gap(starting_angle, requested_angle):
  controller = make_controller()
  state = make_state(starting_angle, speed=1.0, front_wheel_speed=0.1)
  control = make_control(True, starting_angle)
  controller.update(control, structs.CarControlSP(), state, 0)
  previous_wire_angle = starting_angle

  for frame in range(1, 11):
    control = make_control(True, requested_angle)
    actuators, sends = controller.update(control, structs.CarControlSP(), state, frame * 10_000_000)
    steering = [send for send in sends if send[0] == 0x345]
    if not steering:
      continue
    values = decode_steering_message(steering[0])
    assert values["LKA_ACTIVE"] == 1
    wire_angle = (values["CMD"] + 392) / 10
    assert abs(wire_angle - previous_wire_angle) <= 5.0
    assert wire_angle == pytest.approx(actuators.steeringAngleDeg)
    assert values["CMD"] not in (0, 1)
    previous_wire_angle = wire_angle


def test_lateral_transition_outside_angle_limit_stays_inactive_until_in_range():
  controller = make_controller()
  control = make_control(True, 80.0)

  actuators, sends = controller.update(control, structs.CarControlSP(), make_state(-361.0), 0)
  values = decode_steering_message(next(send for send in sends if send[0] == 0x345))
  assert values["LKA_ACTIVE"] == 0
  assert values["CMD"] == round(-361.0 * 10 - 392)
  assert actuators.steeringAngleDeg == -361.0

  controller.update(control, structs.CarControlSP(), make_state(0.0), 10_000_000)
  actuators, sends = controller.update(control, structs.CarControlSP(), make_state(0.0), 20_000_000)
  values = decode_steering_message(next(send for send in sends if send[0] == 0x345))
  assert values["LKA_ACTIVE"] == 1
  assert actuators.steeringAngleDeg != 0.0


@pytest.mark.parametrize("measured_angle", [(500.0), (-500.0)])
def test_lateral_inactive_extreme_angle_is_not_transmitted(measured_angle):
  controller = make_controller()
  actuators, sends = controller.update(
    make_control(False, 80.0), structs.CarControlSP(), make_state(measured_angle), 0,
  )

  assert not any(send[0] == 0x345 for send in sends)
  assert actuators.steeringAngleDeg == measured_angle


def steer_wire_angles(controller, angles, speed, standstill=False, start_frame=0):
  """Feed alternating desired angles at 100 Hz with the wheel at 0 and return each 0x345 angle sent."""
  wire = []
  for i, angle in enumerate(angles):
    frame = start_frame + i
    _actuators, sends = controller.update(make_control(True, angle), structs.CarControlSP(),
                                          make_state(0.0, speed, standstill=standstill), frame * 10_000_000)
    wire += [(decode_steering_message(send)["CMD"] + 392) / 10 for send in sends if send[0] == 0x345]
  return wire


def jitter(period_frames=18, amplitude=2.0, frames=400):
  # ~2.8 Hz square-ish wobble, the rate the model's low-speed angle reverses at
  return [amplitude if (i // (period_frames // 2)) % 2 == 0 else -amplitude for i in range(frames)]


def test_low_speed_angle_jitter_is_smoothed():
  wire = steer_wire_angles(make_controller(), jitter(), speed=5 / 3.6)
  assert max(abs(angle) for angle in wire[100:]) < 1.0


def test_angle_passes_through_unfiltered_above_30_kph():
  wire = steer_wire_angles(make_controller(), jitter(), speed=60 / 3.6)
  assert max(abs(angle) for angle in wire[100:]) == pytest.approx(2.0)


def test_filter_lag_is_capped_through_a_hairpin():
  # 80 deg/s ramp at 22 kph, under the 100 deg/s rate limit, so only the filter can hold it back.
  ramp = [min(i * 0.8, 160.0) for i in range(300)]
  wire = steer_wire_angles(make_controller(), ramp, speed=22 / 3.6)
  lag = [abs(want - got) for want, got in zip(ramp[1::2], wire[1:])]
  # Uncapped, tau trails an 80 deg/s ramp by 23 deg here. The literal bound is the point of the test.
  assert max(lag) <= 8.0
  assert max(lag) <= CarControllerParams.ANGLE_FILTER_MAX_LAG + 2


def test_steady_low_speed_turn_still_reaches_the_requested_angle():
  wire = steer_wire_angles(make_controller(), [30.0] * 300, speed=5 / 3.6)
  assert wire[-1] == pytest.approx(30.0, abs=0.1)


def test_wheel_is_held_while_stopped():
  wire = steer_wire_angles(make_controller(), [20.0] * 100, speed=0.0, standstill=True)
  assert all(angle == pytest.approx(0.0) for angle in wire)


def test_first_engagement_starts_the_filter_from_the_wheel():
  controller = make_controller()
  _actuators, sends = controller.update(make_control(True, 30.0), structs.CarControlSP(), make_state(30.0, 5 / 3.6), 0)
  values = decode_steering_message(next(send for send in sends if send[0] == 0x345))
  assert (values["CMD"] + 392) / 10 == pytest.approx(30.0)


def test_filter_restarts_from_the_wheel_after_lateral_drops():
  controller = make_controller()
  steer_wire_angles(controller, [30.0] * 300, speed=5 / 3.6)
  controller.update(make_control(False, 30.0), structs.CarControlSP(), make_state(0.0, 5 / 3.6), 300 * 10_000_000)
  controller.update(make_control(False, 30.0), structs.CarControlSP(), make_state(0.0, 5 / 3.6), 301 * 10_000_000)
  wire = steer_wire_angles(controller, [30.0] * 2, speed=5 / 3.6, start_frame=302)
  assert abs(wire[0]) < 3.0


def test_lateral_hard_cap_is_360_degrees():
  controller = make_controller()
  controller.apply_angle_last = 359.0
  actuators, _sends = controller.update(make_control(True, 500.0), structs.CarControlSP(), make_state(359.0), 0)
  assert abs(actuators.steeringAngleDeg) <= 360.


def test_lateral_limits_use_front_wheel_mean_not_rear_speed():
  front_speed_controller = make_controller()
  rear_speed_controller = make_controller()
  control = make_control(True, 80.0)
  front_state = make_state(0.0, speed=14.0, front_wheel_speed=18.0)
  rear_state = make_state(0.0, speed=14.0, front_wheel_speed=14.0)
  front_actuators, _ = front_speed_controller.update(control, structs.CarControlSP(), front_state, 0)
  rear_actuators, _ = rear_speed_controller.update(control, structs.CarControlSP(), rear_state, 0)
  assert front_actuators.steeringAngleDeg < rear_actuators.steeringAngleDeg


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
      ("ENGINE_DATA", {"GAS": 4, "GAS_PEDAL": 4, "BRAKE_PRESS": 1, "GEAR": 4}),
      ("STEER_BUTTON", {"ACC": 1, "RES_PLUS": 1}),
    ],
    Bus.cam: [
      ("ACC", {"ACC_ACTIVE": 1, "AEB_ACTIVE": 1}),
      ("ACC_CMD", {"STOPPED": 0, "GAS_PRESSED": 1, "ACC_STATE": 3}),
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
  assert car_state.front_wheel_speed == pytest.approx((10 + 11) / 2 / 3.6, abs=1e-3)
  assert state.steeringAngleDeg == pytest.approx(-12.3)
  assert state.steeringTorque == pytest.approx(-24)
  assert state.steeringPressed is False
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


# ACC_AVAILABLE reads 3 during a driver accelerator override, while ACC_ACTIVE stays 1.
@pytest.mark.parametrize("available, acc_active, expected", [
  (0, 0, False), (1, 0, True), (2, 0, True), (3, 0, False), (3, 1, True), (0, 1, False),
])
def test_acc_available_values(available, acc_active, expected):
  cp = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
  parsers = CarState.get_can_parsers(cp, structs.CarParamsSP())
  packer = CANPacker("chery_canfd")
  frames = []
  for message, values in (("SETTING", {"ACC_AVAILABLE": float(available)}), ("ACC", {"ACC_ACTIVE": float(acc_active)})):
    address, data, bus = packer.make_can_msg(message, parsers[Bus.cam].bus, values)
    frames.append((address, data, bus))
  parsers[Bus.cam].update([[0, frames]])
  state, _ = CarState(cp, structs.CarParamsSP()).update(parsers)
  assert state.cruiseState.available is expected


# Route 494 seg 22: ~3s into a standstill hold the ACC drops ACC_ACTIVE and ACC_AVAILABLE reads 3
# while it waits for RES+. That must stay available, or wrongCarMode disengages before the resume tap.
def test_standstill_hold_stays_available():
  cp = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
  parsers = CarState.get_can_parsers(cp, structs.CarParamsSP())
  packer = CANPacker("chery_canfd")
  CS = CarState(cp, structs.CarParamsSP())
  for available, acc_active in ((2, 1), (3, 0)):
    frames = [packer.make_can_msg(message, parsers[Bus.cam].bus, values) for message, values in (
      ("SETTING", {"ACC_AVAILABLE": float(available)}),
      ("ACC", {"ACC_ACTIVE": float(acc_active)}),
      ("ACC_CMD", {"ACC_STATE": 2.0, "STOPPED": 1.0}),
    )]
    parsers[Bus.cam].update([[0, frames]])
    state, _ = CS.update(parsers)
  assert state.cruiseState.available
  assert state.cruiseState.enabled
  assert state.cruiseState.standstill


@pytest.mark.parametrize("acc_aeb, setting_aeb, aeb, fcw", [
  (0, 0, False, False),
  (0, 2, False, False),
  (0, 3, True, False),
  (1, 0, False, True),
  (1, 2, False, True),
  (1, 3, True, False),
])
def test_stock_aeb_from_setting_and_fcw_from_acc(acc_aeb, setting_aeb, aeb, fcw):
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
  assert state.stockAeb is aeb
  assert state.stockFcw is fcw


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


@pytest.mark.parametrize("active, acc_gas, engine_gas, pedal, expected", [
  # While the ACC owns the throttle, only the camera's driver-pedal bit counts: ENGINE_DATA.GAS
  # is openpilot's own request echoed back, and GAS_PEDAL also reports light touches the camera
  # rejects. With the ACC off, the pedal byte decides and the executed throttle is ignored.
  (1, 1, 0, 0, True), (1, 0, 100, 0, False), (1, 0, 60, 0, False), (1, 0, 0, 0, False),
  (0, 1, 0, 0, False), (0, 0, 60, 40, True), (0, 0, 60, 0, False), (0, 0, 0, 2, True),
  (0, 0, 0, 1, False),
])
def test_gas_source_by_acc_active(active, acc_gas, engine_gas, pedal, expected):
  cp = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
  parsers = CarState.get_can_parsers(cp, structs.CarParamsSP())
  packer = CANPacker("chery_canfd")
  pt_addr, pt_data, pt_bus = packer.make_can_msg("ENGINE_DATA", parsers[Bus.pt].bus,
                                                 {"GAS": float(engine_gas), "GAS_PEDAL": float(pedal)})
  acc_addr, acc_data, acc_bus = packer.make_can_msg("ACC", parsers[Bus.cam].bus, {"ACC_ACTIVE": float(active)})
  cmd_addr, cmd_data, cmd_bus = packer.make_can_msg("ACC_CMD", parsers[Bus.cam].bus, {"GAS_PRESSED": float(acc_gas)})
  parsers[Bus.pt].update([[0, [(pt_addr, pt_data, pt_bus)]]])
  parsers[Bus.cam].update([[0, [(acc_addr, acc_data, acc_bus), (cmd_addr, cmd_data, cmd_bus)]]])
  state, _ = CarState(cp, structs.CarParamsSP()).update(parsers)
  assert state.gasPressed is expected


@pytest.mark.parametrize("speed_kph", [0.0, -5.0, 30.0, 80.0])
def test_engine_data_speed_is_signed_around_its_offset(speed_kph):
  # Raw 30000 is standstill and reverse reads below it, so the signal only survives a boundary
  # change if the offset stays put: a 15-bit read or a lost sign bit breaks the round trip.
  cp = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
  parsers = CarState.get_can_parsers(cp, structs.CarParamsSP())
  packer = CANPacker("chery_canfd")
  address, data, bus = packer.make_can_msg("ENGINE_DATA", parsers[Bus.pt].bus, {"SPEED": speed_kph})
  parsers[Bus.pt].update([[0, [(address, data, bus)]]])
  assert parsers[Bus.pt].vl["ENGINE_DATA"]["SPEED"] == pytest.approx(speed_kph, abs=0.02)


def test_engine_data_throttle_bytes_are_independent():
  # GAS (executed throttle) and GAS_PEDAL (driver pedal) share neighbouring bytes; a 16-bit read
  # of either one swallows the other, which is what the old GAS definition did.
  cp = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
  parsers = CarState.get_can_parsers(cp, structs.CarParamsSP())
  packer = CANPacker("chery_canfd")
  address, data, bus = packer.make_can_msg("ENGINE_DATA", parsers[Bus.pt].bus, {"GAS": 60, "GAS_PEDAL": 0})
  parsers[Bus.pt].update([[0, [(address, data, bus)]]])
  assert parsers[Bus.pt].vl["ENGINE_DATA"]["GAS"] == 60
  assert parsers[Bus.pt].vl["ENGINE_DATA"]["GAS_PEDAL"] == 0


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


def test_gap_button_cycles_personality_and_gap_setting_is_read():
  cp = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
  parsers = CarState.get_can_parsers(cp, structs.CarParamsSP())
  packer = CANPacker("chery_canfd")
  state = CarState(cp, structs.CarParamsSP())
  for up, down, expected in ((1, 0, [True]), (0, 0, [False]), (0, 1, [True]), (0, 0, [False])):
    address, data, bus = packer.make_can_msg("STEER_BUTTON", parsers[Bus.pt].bus,
                                             {"GAP_ADJUST_UP": up, "GAP_ADJUST_DOWN": down})
    parsers[Bus.pt].update([[0, [(address, data, bus)]]])
    events = state.update(parsers)[0].buttonEvents
    assert [(str(event.type), event.pressed) for event in events] == [("gapAdjustCruise", p) for p in expected]

  address, data, bus = packer.make_can_msg("SETTING", parsers[Bus.cam].bus, {"GAP": 5})
  parsers[Bus.cam].update([[0, [(address, data, bus)]]])
  state.update(parsers)
  assert state.gap_setting == 5


def hud_frames(sends):
  return [send for send in sends if send[0] == 0x307]


def decode_hud(message):
  parser = CANParser("chery_canfd", [("LKAS_STATE", 0)], 0)
  parser.update([[0, [message]]])
  return parser.vl["LKAS_STATE"]


def decode_hud_alert(message):
  parser = CANParser("chery_canfd", [("HUD_ALERT", 0)], 0)
  parser.update([[0, [message]]])
  return parser.vl["HUD_ALERT"]


def test_hud_frame_goes_out_at_20hz_and_mirrors_lateral_state():
  controller = make_controller()
  sends = []
  for frame in range(100):
    _actuators, frame_sends = controller.update(make_control(True, 5.0), structs.CarControlSP(),
                                                make_state(0.0, 10.0), frame * 10_000_000)
    sends.extend(frame_sends)
  hud = hud_frames(sends)
  assert len(hud) == 20
  assert all(bus == 0 and len(data) == 8 for _addr, data, bus in hud)
  assert decode_hud(hud[-1])["LKA_ACTIVE"] == 1


def test_hud_frame_relays_stock_content_while_lateral_is_inactive():
  controller = make_controller()
  state = make_state(0.0, 10.0)
  state.lkas_state.update({"LKA_ACTIVE": 1, "STATE": 1, "COUNTER": 5, "CHECKSUM": 0x42})
  _actuators, sends = controller.update(make_control(False, 0.0), structs.CarControlSP(), state, 0)
  values = decode_hud(hud_frames(sends)[0])
  assert values["LKA_ACTIVE"] == 1
  assert values["STATE"] == 1
  assert values["CHECKSUM"] == 0x42


def run_frames(controller, control, frames, **state_kwargs):
  """Drive the controller over a frame range and return the last frame of each kind it emitted."""
  last = {}
  for frame in frames:
    _actuators, sends = controller.update(control, structs.CarControlSP(),
                                          make_state(0.0, 10.0, **state_kwargs), frame * 10_000_000)
    for send in sends:
      last[send[0]] = send
  return last


def test_driver_torque_hands_lateral_back_and_takes_it_returned():
  controller = make_controller()
  control = make_control(True, 30.0)
  override = CarControllerParams.STEER_THRESHOLD + 1

  # A brief tug does not drop lateral.
  last = run_frames(controller, control, range(50), steering_torque=override)
  assert decode_steering_message(last[0x345])["LKA_ACTIVE"] == 1
  assert decode_hud_alert(last[0x3FC])["ICA_WARNING"] == 0

  # Holding past a second does.
  last = run_frames(controller, control, range(50, 160), steering_torque=override)
  assert decode_steering_message(last[0x345])["LKA_ACTIVE"] == 0
  assert decode_hud(last[0x307])["LKA_ACTIVE"] == 0
  alert = last[0x3FC]
  assert decode_hud_alert(alert)["ICA_WARNING"] == 6
  assert alert[1][-1] == calculate_crc(alert[1][:-1])

  # And letting go for a second gives it back.
  last = run_frames(controller, control, range(160, 400), steering_torque=0.0)
  assert decode_steering_message(last[0x345])["LKA_ACTIVE"] == 1
  assert decode_hud(last[0x307])["LKA_ACTIVE"] == 1
  assert decode_hud_alert(last[0x3FC])["ICA_WARNING"] == 0


def test_hud_alert_relays_stock_content_when_not_overridden():
  controller = make_controller()
  state = make_state(0.0, 10.0)
  state.hud_alert.update({"ICA_WARNING": 3, "NEW_SIGNAL_3": -1, "CHECKSUM": 0x42})
  _actuators, sends = controller.update(make_control(False, 0.0), structs.CarControlSP(), state, 0)
  values = decode_hud_alert(next(send for send in sends if send[0] == 0x3FC))
  assert values["ICA_WARNING"] == 3
  assert values["NEW_SIGNAL_3"] == -1
  assert values["CHECKSUM"] == 0x42


@pytest.mark.parametrize("raw", ["000012401b0ff1fd", "000012201b0ff76d"])
def test_hud_alert_relay_is_byte_exact_on_real_camera_frames(raw):
  # Byte 4 (0x1B) had no DBC signal, so the relay zeroed it and sent the camera's CRC over the changed bytes.
  raw = bytes.fromhex(raw)
  assert raw[-1] == calculate_crc(raw[:-1])
  stock = decode_hud_alert((0x3FC, raw, 0)).copy()
  assert create_hud_alert(CANPacker("chery_canfd"), 0, stock, False, False)[1] == raw


def test_steer_required_alert_raises_steer_warning():
  VisualAlert = structs.CarControl.HUDControl.VisualAlert
  controller = make_controller()
  control = make_control(True, 0.0, visual_alert=VisualAlert.steerRequired)
  _actuators, sends = controller.update(control, structs.CarControlSP(), make_state(0.0, 10.0), 0)
  alert = next(send for send in sends if send[0] == 0x3FC)
  values = decode_hud_alert(alert)
  assert values["STEER_WARNING"] == 1
  assert values["ICA_WARNING"] == 0
  assert alert[1][-1] == calculate_crc(alert[1][:-1])

  # While openpilot steers, the camera's own hands-on nag is dropped: openpilot's driver
  # monitoring covers driver attention, and stock LKA nags for wheel torque it never gets.
  state = make_state(0.0, 10.0)
  state.hud_alert.update({"STEER_WARNING": 1, "CHECKSUM": 0x42})
  control = make_control(True, 0.0, visual_alert=VisualAlert.fcw)
  _actuators, sends = make_controller().update(control, structs.CarControlSP(), state, 0)
  alert = next(send for send in sends if send[0] == 0x3FC)
  values = decode_hud_alert(alert)
  assert values["STEER_WARNING"] == 0
  assert alert[1][-1] == calculate_crc(alert[1][:-1])

  # With openpilot's lateral off the camera owns lane keeping, so its warning is relayed verbatim.
  state = make_state(0.0, 10.0)
  state.hud_alert.update({"STEER_WARNING": 1, "CHECKSUM": 0x42})
  control = make_control(False, 0.0, visual_alert=VisualAlert.fcw)
  _actuators, sends = make_controller().update(control, structs.CarControlSP(), state, 0)
  values = decode_hud_alert(next(send for send in sends if send[0] == 0x3FC))
  assert values["STEER_WARNING"] == 1
  assert values["CHECKSUM"] == 0x42


def make_resume_control(resume: bool):
  control = structs.CarControl()
  control.latActive = False
  control.longActive = True
  control.cruiseControl.resume = resume
  return control.as_reader()


def button_frames(sends):
  return [send for send in sends if send[0] == 0x360]


def test_resume_taps_res_plus_instead_of_holding_it():
  controller = make_controller()
  state = make_state(0.0, 0.0, standstill=True, acc_active=False)
  pressed = []
  parser = CANParser("chery_canfd", [("STEER_BUTTON", 2)], 2)
  for frame in range(140):
    _actuators, sends = controller.update(make_resume_control(True), structs.CarControlSP(), state, frame * 10_000_000)
    for message in button_frames(sends):
      assert message[2] == 2
      parser.update([[0, [message]]])
      pressed.append(parser.vl["STEER_BUTTON"]["RES_PLUS"])

  # 140 frames is 28 button slots: two full 14-slot cycles of one frame each. More than one frame
  # per cycle reads as more than one press at the camera and raises the set speed on resume.
  assert len(pressed) == 2
  assert all(pressed)


def test_resume_stops_once_the_acc_is_active_again():
  controller = make_controller()
  active = make_state(0.0, 0.0, standstill=True, acc_active=True)
  for frame in range(40):
    _actuators, sends = controller.update(make_resume_control(True), structs.CarControlSP(), active, frame * 10_000_000)
    assert not button_frames(sends)


def test_no_resume_request_sends_no_buttons():
  controller = make_controller()
  state = make_state(0.0, 0.0, standstill=True, acc_active=False)
  for frame in range(40):
    _actuators, sends = controller.update(make_resume_control(False), structs.CarControlSP(), state, frame * 10_000_000)
    assert not button_frames(sends)


def acc_stock():
  return {name: 0 for name in (
    "ACC_STATE", "STOPPED", "ACC_STATE_2", "NEW_SIGNAL_12", "NEW_SIGNAL_9",
    "NEW_SIGNAL_2", "STOPPING", "NEW_SIGNAL_13", "NEW_SIGNAL_8", "NEW_SIGNAL_5",
    "NEW_SIGNAL_6", "NEW_SIGNAL_10", "NEW_SIGNAL_3", "NEW_SIGNAL_4", "AEB_REQ_STOP", "COUNTER",
  )}


@pytest.mark.parametrize("standstill, accel, expect_hold", [
  (True, -1.0, True),    # stopped and still braking: hold
  (True, 0.5, False),    # stopped but the plan wants to launch: release
  (False, -3.5, False),  # still rolling: a hold here is a full-force brake
])
def test_full_stop_hold_needs_a_stopped_car_and_a_braking_plan(standstill, accel, expect_hold):
  controller = make_controller()
  controller.CP.openpilotLongitudinalControl = True
  control = structs.CarControl()
  control.longActive = True
  control.actuators.accel = accel
  state = make_state(0.0, 0.0 if standstill else 5.0, standstill=standstill)
  state.acc_cmd = acc_stock()
  _actuators, sends = controller.update(control.as_reader(), structs.CarControlSP(), state, 0)
  parser = CANParser("chery_canfd", [("ACC_CMD", 0)], 0)
  parser.update([[0, [next(send for send in sends if send[0] == 0x3A2)]]])
  values = parser.vl["ACC_CMD"]
  assert values["STOPPED"] == int(expect_hold)
  assert (values["CMD"] == 400 and values["ACCEL_ON"] == 0) == expect_hold


def feed(parsers, packer, bus_key, messages):
  frames = []
  for message, values in messages:
    address, data, _ = packer.make_can_msg(message, parsers[bus_key].bus, values)
    frames.append((address, data, parsers[bus_key].bus))
  parsers[bus_key].update([0, frames])


def state_fixture():
  cp = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
  return cp, CarState.get_can_parsers(cp, structs.CarParamsSP()), CANPacker("chery_canfd")


@pytest.mark.parametrize("sign_signal, left, right", [(0, False, False), (1, False, True), (2, True, False)])
def test_blinkers_decode_from_bcm(sign_signal, left, right):
  cp, parsers, packer = state_fixture()
  feed(parsers, packer, Bus.pt, [("BCM_SIGNAL_1", {"SIGN_SIGNAL": float(sign_signal)})])
  state, _ = CarState(cp, structs.CarParamsSP()).update(parsers)
  assert (state.leftBlinker, state.rightBlinker) == (left, right)


@pytest.mark.parametrize("acc_active, stopped, expected", [
  (0, 1, True),   # the stock hold: ACC_ACTIVE has dropped, STOPPED carries it
  (1, 1, False),  # RES+ landed, ACC is back: must clear or the car never launches
  (0, 0, False),
  (1, 0, False),
])
def test_cruise_standstill_tracks_the_stock_hold_not_vego(acc_active, stopped, expected):
  cp, parsers, packer = state_fixture()
  feed(parsers, packer, Bus.cam, [
    ("ACC", {"ACC_ACTIVE": float(acc_active)}),
    ("ACC_CMD", {"STOPPED": float(stopped)}),
  ])
  state, _ = CarState(cp, structs.CarParamsSP()).update(parsers)
  assert state.standstill  # no wheel speed fed, so vEgo is zero throughout
  assert state.cruiseState.standstill is expected


@pytest.mark.parametrize("acc_state, gas, expected", [
  (3, 0, True),
  (2, 0, True),
  (1, 1, True),   # accelerator override keeps the ACC available
  (1, 0, False),
  (0, 0, False),  # route 00000483: camera abort with ACC_ACTIVE still 1
])
def test_cruise_enabled_follows_acc_state_like_the_panda(acc_state, gas, expected):
  cp, parsers, packer = state_fixture()
  feed(parsers, packer, Bus.cam, [
    ("ACC", {"ACC_ACTIVE": 1.0}),
    ("ACC_CMD", {"ACC_STATE": float(acc_state), "GAS_PRESSED": float(gas)}),
  ])
  state, _ = CarState(cp, structs.CarParamsSP()).update(parsers)
  assert state.cruiseState.enabled is expected


def test_stopped_only_holds_an_existing_engagement():
  cp, parsers, packer = state_fixture()
  car_state = CarState(cp, structs.CarParamsSP())
  feed(parsers, packer, Bus.cam, [("ACC", {"ACC_ACTIVE": 0.0}), ("ACC_CMD", {"ACC_STATE": 2.0, "STOPPED": 1.0})])
  assert not car_state.update(parsers)[0].cruiseState.enabled
  feed(parsers, packer, Bus.cam, [("ACC", {"ACC_ACTIVE": 1.0}), ("ACC_CMD", {"ACC_STATE": 2.0, "STOPPED": 1.0})])
  assert car_state.update(parsers)[0].cruiseState.enabled
  feed(parsers, packer, Bus.cam, [("ACC", {"ACC_ACTIVE": 0.0}), ("ACC_CMD", {"ACC_STATE": 2.0, "STOPPED": 1.0})])
  assert car_state.update(parsers)[0].cruiseState.enabled


def test_unresponsive_driver_stop_brakes_to_a_held_standstill():
  # Driver monitoring's noResponseForceDecel drops the planner's cruise speed to zero. On Chery that
  # only reaches the car through openpilot longitudinal: brake from 80 kph, then hold for a minute
  # after the stock ACC lets go of ACC_ACTIVE, without ever sending the standstill brake while rolling.
  controller = make_controller()
  controller.CP.openpilotLongitudinalControl = True
  parser = CANParser("chery_canfd", [("ACC_CMD", 0)], 0)
  speed, frame, rolling_cmds, held_cmds = 80 / 3.6, 0, [], []
  while frame < 80 * 100:
    standstill = speed == 0.
    control = structs.CarControl()
    control.longActive = True
    control.actuators.accel = -0.5 if standstill else -1.2
    state = make_state(0.0, speed, standstill=standstill)
    state.acc_cmd = acc_stock()
    _actuators, sends = controller.update(control.as_reader(), structs.CarControlSP(), state, frame * 10_000_000)
    for send in sends:
      if send[0] == 0x3A2:
        parser.update([[frame * 10_000_000, [send]]])
        (held_cmds if standstill else rolling_cmds).append(dict(parser.vl["ACC_CMD"]))
    speed = max(0., speed - 1.2 * DT_CTRL)
    frame += 1

  assert rolling_cmds and all(v["CMD"] < 0 and v["STOPPED"] == 0 and v["ACC_STATE"] == 3 for v in rolling_cmds)
  assert len(held_cmds) >= 60 * 50
  assert all(v["CMD"] == 400 and v["STOPPED"] == 1 and v["ACCEL_ON"] == 0 for v in held_cmds)

  # The stock ACC drops ACC_ACTIVE ~3 s into the hold; the engagement has to survive that for the whole minute.
  cp, parsers, packer = state_fixture()
  car_state = CarState(cp, structs.CarParamsSP())
  feed(parsers, packer, Bus.cam, [("ACC", {"ACC_ACTIVE": 1.0}), ("ACC_CMD", {"ACC_STATE": 3.0})])
  assert car_state.update(parsers)[0].cruiseState.enabled
  for _ in range(60 * 50):
    feed(parsers, packer, Bus.cam, [("ACC", {"ACC_ACTIVE": 0.0}), ("ACC_CMD", {"ACC_STATE": 2.0, "STOPPED": 1.0})])
    out = car_state.update(parsers)[0]
    assert out.cruiseState.enabled and out.cruiseState.standstill


# TORQUE_DRIVER is quantised to 0.24, so the pair straddling the threshold is 69.84 / 70.08.
@pytest.mark.parametrize("torque, expected", [(0, False), (69.84, False), (70.08, True), (-70.08, True)])
def test_steering_pressed_uses_torque_magnitude(torque, expected):
  cp, parsers, packer = state_fixture()
  feed(parsers, packer, Bus.pt, [("STEER_SENSOR_2", {"TORQUE_DRIVER": float(torque)})])
  state, _ = CarState(cp, structs.CarParamsSP()).update(parsers)
  assert state.steeringPressed is expected


def drive_eps(car_state, parsers, packer, eps_torque, commanding, frames):
  for _ in range(frames):
    feed(parsers, packer, Bus.pt, [
      ("WHEEL_SPEED_FRNT", {"WHEEL_SPEED_FR": 40, "WHEEL_SPEED_FL": 40}),
      ("WHEEL_SPEED_REAR", {"WHEEL_SPEED_RR": 40, "WHEEL_SPEED_RL": 40}),
      ("LKAS", {"EPS_TORQUE": float(eps_torque), "EPS_INACTIVE": float(eps_torque == 1023)}),
    ])
    feed(parsers, packer, Bus.cam, [("ACC", {"ACC_ACTIVE": 1}), ("ACC_CMD", {"ACC_STATE": 3})])
    feed(parsers, packer, Bus.loopback, [("LKAS_CAM_CMD_345", {"LKA_ACTIVE": float(commanding)})])
    state, _ = car_state.update(parsers)
  return state


def test_eps_fault_latches_only_after_a_sustained_dead_servo():
  cp, parsers, packer = state_fixture()
  car_state = CarState(cp, structs.CarParamsSP())
  timeout = CarControllerParams.STEER_TIMEOUT

  state = drive_eps(car_state, parsers, packer, 1023, True, timeout - 1)
  assert state.steerFaultTemporary is False
  state = drive_eps(car_state, parsers, packer, 1023, True, 1)
  assert state.steerFaultTemporary is True

  # A live servo clears the counter.
  state = drive_eps(car_state, parsers, packer, 5, True, 1)
  assert state.steerFaultTemporary is False

  # A re-arm gap (not commanding, servo still dead) holds the count instead of restarting it.
  drive_eps(car_state, parsers, packer, 1023, True, timeout - 1)
  state = drive_eps(car_state, parsers, packer, 1023, False, 100)
  assert state.steerFaultTemporary is False
  state = drive_eps(car_state, parsers, packer, 1023, True, 1)
  assert state.steerFaultTemporary is True


def steer_with_eps(controller, eps_inactive_at, frames):
  """Request lateral every frame; returns LKA_ACTIVE per steering frame sent."""
  sent = {}
  for frame in range(frames):
    state = make_state(0.0, 20.0)
    state.eps_inactive = eps_inactive_at(frame, sent)
    _actuators, sends = controller.update(make_control(True, 0.0), structs.CarControlSP(), state, frame * 10_000_000)
    for send in sends:
      if send[0] == 0x345:
        sent[frame] = decode_steering_message(send)["LKA_ACTIVE"]
  return sent


def test_latched_eps_is_rearmed_by_dropping_lka_active():
  latch = int(CarControllerParams.EPS_LATCH_TIME / DT_CTRL)
  rearm = int(CarControllerParams.EPS_REARM_TIME / DT_CTRL)

  def eps_inactive_at(frame, sent):
    # Latches at frame 100; like the real EPS, wakes only once LKA_ACTIVE has risen after a drop.
    dropped = [f for f, value in sent.items() if f > 100 and value == 0]
    return frame >= 100 and not (dropped and sent.get(frame - 4) == 1 and frame - 4 > dropped[0])

  sent = steer_with_eps(make_controller(), eps_inactive_at, 100 + latch + rearm + 50)
  dropped = [f for f, value in sent.items() if f > 100 and value == 0]
  assert dropped and dropped[0] - 100 <= latch + 2
  assert rearm - 2 <= dropped[-1] - dropped[0] + 2 <= rearm + 2
  # Back to commanding after the gap, and no second drop once the EPS woke up.
  assert all(value == 1 for f, value in sent.items() if f > dropped[-1])


def test_brief_eps_blips_do_not_drop_lka_active():
  # 20-30 ms blips like the ones seen while steering on route 0000049e.
  sent = steer_with_eps(make_controller(), lambda frame, _sent: frame > 20 and frame % 50 < 3, 500)
  assert all(value == 1 for f, value in sent.items() if f > 10)


def test_rearm_holds_off_until_openpilot_is_commanding():
  # Lateral just requested: the EPS still reports inactive for its ~40 ms engagement delay.
  sent = steer_with_eps(make_controller(), lambda frame, _sent: frame < 4, 200)
  assert all(value == 1 for f, value in sent.items() if f > 0)


def make_long_controller():
  cp = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
  cp.openpilotLongitudinalControl = True
  return CarController(DBC[CAR.CHERY_OMODA_E5], cp, structs.CarParamsSP())


def gap_tap_frames(controller, bars, gap_setting, frames=140, start_frame=0, **kwargs):
  """Frame numbers, relative to start_frame, on which a gap press went out."""
  sent = []
  gap_taps(controller, bars, gap_setting, frames=frames, start_frame=start_frame, sent_frames=sent, **kwargs)
  return sent


def gap_taps(controller, bars, gap_setting, long_active=True, acc_active=True, frames=140,
             start_frame=0, sent_frames=None):
  control = structs.CarControl()
  control.longActive = long_active
  control.hudControl.leadDistanceBars = bars
  state = make_state(0.0, 10.0, acc_active=acc_active)
  state.gap_setting = gap_setting
  state.acc_cmd = {name: 0 for name in (
    "ACC_STATE", "STOPPED", "ACC_STATE_2", "NEW_SIGNAL_12", "NEW_SIGNAL_9", "NEW_SIGNAL_2", "STOPPING",
    "NEW_SIGNAL_13", "NEW_SIGNAL_8", "NEW_SIGNAL_5", "NEW_SIGNAL_6", "NEW_SIGNAL_10",
    "NEW_SIGNAL_3", "NEW_SIGNAL_4", "AEB_REQ_STOP", "COUNTER",
  )}
  parser = CANParser("chery_canfd", [("STEER_BUTTON", 2)], 2)
  taps = []
  for frame in range(frames):
    _actuators, sends = controller.update(control.as_reader(), structs.CarControlSP(), state,
                                          (start_frame + frame) * 10_000_000)
    for send in button_frames(sends):
      parser.update([[0, [send]]])
      taps.append((parser.vl["STEER_BUTTON"]["GAP_ADJUST_UP"], parser.vl["STEER_BUTTON"]["GAP_ADJUST_DOWN"]))
      if sent_frames is not None:
        sent_frames.append(frame)
  return taps


@pytest.mark.parametrize("bars, gap_setting, expected", [
  (3, 3, (1, 0)),  # relaxed wants 4: farther
  (1, 3, (0, 1)),  # aggressive wants 2: closer
  (2, 5, (0, 1)),
])
def test_gap_taps_toward_personality_target(bars, gap_setting, expected):
  # One frame per press, 700ms apart: the camera reads every injected frame as its own press.
  assert gap_taps(make_long_controller(), bars, gap_setting) == [expected] * 2


def test_gap_taps_are_single_frames_spaced_for_the_readback():
  frames = gap_tap_frames(make_long_controller(), 3, 3, frames=1400)
  assert frames == [0, 70, 140, 210, 280, 350]  # 700ms apart, and no seventh press


def test_gap_taps_stop_until_the_target_moves():
  controller = make_long_controller()
  assert len(gap_tap_frames(controller, 3, 3, frames=1400)) == CarControllerParams.GAP_MAX_TAPS
  # A new personality is a new attempt; the same one is not.
  assert gap_tap_frames(controller, 3, 3, frames=200, start_frame=1400) == []
  assert gap_tap_frames(controller, 1, 3, frames=200, start_frame=1600) != []


@pytest.mark.parametrize("kwargs", [
  {"bars": 2, "gap_setting": 3},                       # already matched
  {"bars": 3, "gap_setting": 3, "long_active": False},
  {"bars": 3, "gap_setting": 3, "acc_active": False},
  {"bars": 0, "gap_setting": 3},                       # no personality published
])
def test_gap_taps_stay_quiet(kwargs):
  assert gap_taps(make_long_controller(), **kwargs) == []


def test_gap_taps_need_openpilot_longitudinal():
  assert gap_taps(make_controller(), 3, 3) == []


def make_icbm_control(send_button):
  control = structs.CarControlSP()
  control.intelligentCruiseButtonManagement.sendButton = send_button
  return control


def decode_buttons(messages):
  parser = CANParser("chery_canfd", [("STEER_BUTTON", 2)], 2)
  out = []
  for message in messages:
    parser.update([[0, [message]]])
    out.append((parser.vl["STEER_BUTTON"]["RES_PLUS"], parser.vl["STEER_BUTTON"]["RES_MINUS"]))
  return out


@pytest.mark.parametrize("send_button, expected", [
  (structs.IntelligentCruiseButtonManagement.SendButtonState.increase, (1, 0)),
  (structs.IntelligentCruiseButtonManagement.SendButtonState.decrease, (0, 1)),
])
def test_icbm_taps_set_speed_buttons_while_acc_active(send_button, expected):
  controller = make_controller()
  state = make_state(0.0, 10.0, acc_active=True)
  messages = []
  for frame in range(140):
    _actuators, sends = controller.update(make_resume_control(False), make_icbm_control(send_button), state, frame * 10_000_000)
    messages += [send for send in sends if send[0] == 0x360]
  assert all(bus == 2 for _addr, _data, bus in messages)
  # Same tap cadence as resume: one frame per 14-frame cycle, two cycles. Each frame the camera
  # sees is one press, so a wider tap would step the set speed once per frame.
  assert decode_buttons(messages) == [expected] * 2


def test_icbm_sends_nothing_without_an_active_acc():
  """RES- at ACC_ACTIVE 0 is SET and would engage the ACC."""
  controller = make_controller()
  state = make_state(0.0, 10.0, acc_active=False)
  decrease = structs.IntelligentCruiseButtonManagement.SendButtonState.decrease
  for frame in range(60):
    _actuators, sends = controller.update(make_resume_control(False), make_icbm_control(decrease), state, frame * 10_000_000)
    assert not button_frames(sends)


def test_icbm_yields_to_resume():
  controller = make_controller()
  state = make_state(0.0, 0.0, standstill=True, acc_active=False)
  decrease = structs.IntelligentCruiseButtonManagement.SendButtonState.decrease
  messages = []
  for frame in range(40):
    _actuators, sends = controller.update(make_resume_control(True), make_icbm_control(decrease), state, frame * 10_000_000)
    messages += button_frames(sends)
  assert messages and all(values == (1, 0) for values in decode_buttons(messages))


def test_icbm_available():
  cp = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.CHERY_OMODA_E5)
  assert cp_sp.intelligentCruiseButtonManagementAvailable


@pytest.mark.parametrize("door", ["FL_DOOR_OPEN", "FR_DOOR_OPEN", "RL_DOOR_OPEN", "RR_DOOR_OPEN"])
def test_any_open_door_reports_door_open(door):
  cp, parsers, packer = state_fixture()
  feed(parsers, packer, Bus.pt, [("BCM_SIGNAL_1", {door: 1})])
  state, _ = CarState(cp, structs.CarParamsSP()).update(parsers)
  assert state.doorOpen


@pytest.mark.parametrize("seatbelt, expected", [(0, False), (1, True)])
def test_seatbelt_bit_means_unlatched(seatbelt, expected):
  cp, parsers, packer = state_fixture()
  feed(parsers, packer, Bus.pt, [("BCM_SIGNAL_1", {}), ("NEW_MSG_430", {"SEATBELT": float(seatbelt)})])
  state, _ = CarState(cp, structs.CarParamsSP()).update(parsers)
  assert not state.doorOpen
  assert state.seatbeltUnlatched is expected


def make_cancel_control():
  control = structs.CarControl()
  control.cruiseControl.cancel = True
  return control.as_reader()


def decode_cancel(messages):
  parser = CANParser("chery_canfd", [("STEER_BUTTON", 2)], 2)
  out = []
  for message in messages:
    parser.update([[0, [message]]])
    out.append((parser.vl["STEER_BUTTON"]["ACC"], parser.vl["STEER_BUTTON"]["RES_PLUS"], parser.vl["STEER_BUTTON"]["RES_MINUS"]))
  return out


def test_cancel_taps_acc_button_while_active_and_beats_icbm():
  controller = make_controller()
  state = make_state(0.0, 10.0, acc_active=True)
  increase = structs.IntelligentCruiseButtonManagement.SendButtonState.increase
  messages = []
  for frame in range(140):
    _actuators, sends = controller.update(make_cancel_control(), make_icbm_control(increase), state, frame * 10_000_000)
    messages += button_frames(sends)
  assert all(bus == 2 for _addr, _data, bus in messages)
  # One frame per cycle, and cancel still owns the frame ICBM wanted.
  assert decode_cancel(messages) == [(1, 0, 0)] * 2


def test_cancel_sends_nothing_once_acc_is_off():
  """Pressing the ACC button with the ACC off would engage it."""
  controller = make_controller()
  state = make_state(0.0, 10.0, acc_active=False)
  for frame in range(60):
    _actuators, sends = controller.update(make_cancel_control(), structs.CarControlSP(), state, frame * 10_000_000)
    assert not button_frames(sends)


def test_steering_rate_takes_magnitude_from_rate_and_sign_from_angle():
  cp, parsers, packer = state_fixture()
  car_state = CarState(cp, structs.CarParamsSP())
  rates = []
  for angle, rate_raw in ((10.0, 0), (10.5, 20), (11.0, 20), (11.0, 5), (9.0, 30)):
    feed(parsers, packer, Bus.pt, [("STEER_SENSOR", {"STEER_ANGLE_HR": angle, "STEER_RATE": rate_raw * 4})])
    rates.append(car_state.update(parsers)[0].steeringRateDeg)
  assert rates == [pytest.approx(0), pytest.approx(80), pytest.approx(80), pytest.approx(20), pytest.approx(-120)]


def test_steering_torque_takes_sign_from_wheel_motion():
  cp, parsers, packer = state_fixture()
  car_state = CarState(cp, structs.CarParamsSP())
  torques = []
  # (angle, STEER_RATE): a one-LSB wiggle at rate 0 is sensor jitter and must not flip the sign
  for angle, rate in ((10.0, 4), (12.0, 4), (11.9375, 0), (8.0, 4), (8.0625, 0)):
    feed(parsers, packer, Bus.pt, [("STEER_SENSOR", {"STEER_ANGLE_HR": angle, "STEER_RATE": rate}),
                                   ("STEER_SENSOR_2", {"TORQUE_DRIVER": 100})])
    torques.append(car_state.update(parsers)[0].steeringTorque)
  assert torques[1:] == pytest.approx([100, 100, -100, -100], abs=0.5)


def test_steer_sensor_matches_route_frame():
  """0xC4 frame from a real route: 0x87cb is +124.7 deg, byte 2 = 1 is 4 deg/s."""
  parser = CANParser("chery_canfd", [("STEER_SENSOR", 0)], 0)
  parser.update([[0, [(0xC4, bytes.fromhex("87cb010c00c0008e"), 0)]]])
  assert parser.vl["STEER_SENSOR"]["STEER_ANGLE_HR"] == pytest.approx((0x87cb - 0x8000) * 0.0625)
  assert parser.vl["STEER_SENSOR"]["STEER_RATE"] == pytest.approx(4)
  assert parser.vl["STEER_SENSOR"]["COUNTER"] == 0xc
