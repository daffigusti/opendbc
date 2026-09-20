# Chery Omoda E5 parser gaps

- Door and seatbelt are parsed from one route. Any `BCM_SIGNAL_1` door bit reports
  `doorOpen`; the bits rose only in park or at a crawl. `NEW_MSG_430.SEATBELT`
  reads 1 when unlatched (in park before buckling and after exit) and 0 for the
  whole drive. Which door each bit maps to, and whether the belt signal covers
  seats other than the driver's, are unconfirmed.
- FCW is `ACC.AEB_ACTIVE == 1` with `SETTING.AEB_ACTIVE` not at 3, the warning separated
  from the braking: route 488 raised it for a dash collision warning with AEB switched off,
  and route 1b6 braked with `SETTING.AEB_ACTIVE=3` (owner-provided capture; `SETTING.SHOW_AEB`
  pulsed with both events there, so it may be the cluster's popup rather than the driver's AEB
  setting it is currently read as). Neither bit has fired since: 8 segments of routes 0000049e
  and 000004ad hold zero frames of either, so the mapping rests on those two events.

Future evidence required:

- Catch an FCW and an AEB event again to confirm the two bits on more than one route each,
  and to settle what `SHOW_AEB` tracks.
- Physical steer ratio and rack range require owner-labeled measurements.
- Stage C3, Panda bench, and controlled-drive validation before expanding support.
- `0x360` TX carries `RES_PLUS` (resume from a stopped hold, or +set speed) and
  `RES_MINUS` (-set speed), camera bus only, with controls authorized and no cancel,
  main or gap bit. `RES_MINUS` requires `ACC_ACTIVE`, since at 0 it is SET and
  engages the ACC; `RES_PLUS` requires `ACC_ACTIVE` or a stopped car. ICBM and resume
  share one tap cadence (4 frames on, 10 off, at 20Hz) taken from the driver's
  measured resume presses. No route yet has driver +/- taps with the ACC active, so
  kph per tap, auto-repeat on a held press, and whether the camera accepts spoofed
  presses while moving are all unconfirmed.
- Driver torque override is enforced at `abs(TORQUE_DRIVER) > 70` with one second
  of hysteresis either way. `TORQUE_DRIVER`'s sign is still unverified, so only its
  magnitude is used, and the threshold itself needs owner-labeled stationary
  correlation.
- Either gap button emits `gapAdjustCruise`, which cycles openpilot's longitudinal personality.
  `SETTING.GAP` is the stock following distance, 1..5 with 5 the farthest, and `GAP_ADJUST_UP`
  moves it farther (owner-identified). With openpilot longitudinal and the ACC active, `0x360`
  gap taps bring it to 2/3/4 for aggressive/standard/relaxed. The mapping is by eye, not measured
  time gap, and whether the camera accepts spoofed gap presses while moving is unconfirmed.
- `ENGINE_DATA.GAS` is not a driver-pedal signal and is no longer read as one.
  Across 943k moving frames its distribution under ACC and under the driver is
  indistinguishable (38.8% vs 51.9% at zero, both saturating above 26000), so no
  threshold separates them; reading it as a press denied controls in 98%+ of
  ACC-engaged frames while protecting against nothing. `gas_pressed` now comes only
  from the camera's `ACC_CMD.GAS_PRESSED` bit while `ACC_ACTIVE` is 1. Its low duty
  cycle (under 1% of frames) is rare overrides, not a dropped signal: the owner
  confirmed it stays 1 for the whole press, and `ACC.GAS_PRESSED` (`0x3A5`) rises with
  it. With the ACC off no pedal signal exists, so the throttle threshold stands in;
  openpilot is not engaged longitudinally then.
- The ACC command to acceleration map is fitted, not measured on a labelled sweep, but it
  has now been checked against 40 min of openpilot longitudinal on route 0000049e: the car
  delivers 0.97-1.04 of the requested acceleration for CMD from -400 to -100, and 0.89 below
  -400 (230 frames). `ACCEL_MIN` is -3.5 while the deepest braking ever seen is -2.81 m/s^2 at
  CMD -511, so the planner assumes braking the car cannot deliver. Both the deep-end scale and
  `ACCEL_MIN` want a labelled deceleration sweep.
- `ACC_CMD` full-stop uses the stock hold encoding (`CMD=400`, `ACCEL_ON=0`,
  `STOPPED=1`, `ACC_STATE=2`), derived from 10 hold episodes across 192 route
  segments. Panda permits it only while the car is already stopped. Driven on routes
  0000049e and 000004ae (the latter 29 min of congestion): 260 s of hold, longest 78.7 s,
  every frame of every hold below 0.1 kph, so the encoding never went out while rolling.
  Launches out of a hold reach 0.85-1.27 m/s^2.
- `LKAS_STATE` (`0x307`) is now transmitted by openpilot and the stock copy is
  blocked from forwarding once RX health is trusted. Cluster behaviour with the
  substituted frame is unverified.
- `HUD_ALERT` (`0x3FC`) is relayed the same way. While the driver torque override holds
  lateral off, openpilot sets `ICA_WARNING=6`, which the owner identified as the cluster's
  take-over warning; otherwise the camera's frame goes out verbatim. Byte 7 is CRC-8 over
  bytes 0-6, checked on one frame. openpilot's `steerRequired` visual alert (steer fault,
  steer saturated, driver monitoring) sets `STEER_WARNING=1`, which the owner identified as the
  cluster's "take over and steer carefully" warning; the camera's own bit is kept. The owner
  reported the camera's hands-on nag stops with the substituted frame; what the cluster renders
  for each bit is otherwise unverified.
- `steerRatio` 17 is fitted from locationd yaw rate against measured wheel angle
  on one 8-minute urban route (r=0.98, 16.8-17.2 across 11-32 kph); it has not been
  checked above 32 kph. `steerActuatorDelay` 0.15 follows the 130ms command-to-angle
  lag measured on the same route. Panda's `steer_ratio` must be changed with them or
  the VM angle limits diverge from the controller's.
- The low-speed angle filter trades wheel wobble against turn-in and openpilot cannot see the
  trade: `latcontrol_angle` measures saturation as the model's angle against the wheel, not
  against what this carcontroller sent, so the filter's own lag reads as the car failing to
  turn. On route 000004ad seg 10, a hairpin at 22 kph raised "Turn Exceeds Steering Limit" with
  69% of the error being filter lag. `ANGLE_FILTER_MAX_LAG` bounds that; whether the alert still
  fires through such a turn is unconfirmed on the car.
- A driver accelerator override is reported by the stock ACC as `ACC_STATE=1` and
  `SETTING.ACC_AVAILABLE=3` with `ACC_ACTIVE` still 1. Both are treated as an
  available ACC only while `ACC_ACTIVE` is 1 (and, in Panda, the pedal bit is set),
  so lateral survives the override. Seen on three presses in one route.
- MADS is partial support, as on Tesla and Rivian. There is no ACC main switch, and
  `ACC_STATE`/`ACC_AVAILABLE` drop on 98% of brake-pressed frames, so no stable main
  signal exists; Panda leaves `acc_main_on` false. MADS lateral engages with the ACC,
  survives an ACC cancel or unavailability, and is forced to disengage on brake. No
  LKAS toggle button has been identified, so lateral cannot be engaged without the ACC.
- `STEER_SENSOR` (`0xC4`) was re-decoded from a real route: `STEER_ANGLE_HR` is
  wheel angle at 0.0625 deg per LSB (r=1.000 against `STEER_ANGLE`) and `STEER_RATE`
  is an unsigned rate at 4 deg/s per LSB (r=0.992). `steeringRateDeg` takes its sign
  from the high-resolution angle; the sign agrees with the differentiated angle on
  98.8% of frames above 20 deg/s.
- Cancel taps `STEER_BUTTON.ACC` (`0x360` bit 24) on the camera bus, only while
  `ACC_ACTIVE` is 1. The button toggles the ACC, so the same press engages it when off.
  Panda allows it without controls and while braking, but not on untrusted RX. The
  cancel response time (0.1-0.25s) comes from driver presses; host presses are
  unconfirmed.
- Stock AEB is decoded and inhibited, not validated on the road: `SETTING.AEB_ACTIVE == 3`
  drops `controls_allowed` and rejects every host `ACC_CMD`, restoring OEM forwarding. No
  AEB event has happened with openpilot longitudinal engaged, so the handover is untested
  outside the safety tests.
- The EPS reports itself on `LKAS` (`0x1E3`): `EPS_INACTIVE`, a direction pair, and an
  11-bit output (see the DBC comments). A hard driver push past ~300 `TORQUE_DRIVER` latches
  the EPS off until `LKA_ACTIVE` falls and rises again, so the carcontroller re-arms it and
  `steerFaultTemporary` follows the same signal. Both come from route logs (3 dropouts,
  39 recoveries); neither has been exercised on the car. The EPS-fault candidates the driver
  captured in `0x40F` byte 0 (`0xD5` at both logged steer failures), and EPB/auto hold/HDC in
  `0x537`, `0x51D` and `0x502`, are still unparsed.

Publication rule: publish owner-provided evidence only after owner approval, and
strip route IDs, URLs, tokens, VINs, locations, timestamps, and raw identifying
captures before publication.
