# Chery Omoda E5 parser gaps

- Door and seatbelt are parsed from one route. Any `BCM_SIGNAL_1` door bit reports
  `doorOpen`; the bits rose only in park or at a crawl. `NEW_MSG_430.SEATBELT`
  reads 1 when unlatched (in park before buckling and after exit) and 0 for the
  whole drive. Which door each bit maps to, and whether the belt signal covers
  seats other than the driver's, are unconfirmed.
- FCW state unavailable. No confirmed FCW signal exists in route parser set;
  `SETTING.SHOW_AEB` is not treated as FCW. `stockFcw` intentionally stays false.

Future evidence required:

- Capture FCW behavior separately from AEB and confirm route signal mapping.
- Capture EPS fault and watchdog inputs before claiming steer-fault handling.
- Physical steer ratio and rack range require owner-labeled measurements.
- Raw ACC command to physical acceleration mapping requires owner-labeled
  measurements.
- Stock AEB interaction requires hardware validation.
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
- `ENGINE_DATA.GAS` is not a driver-pedal signal and is no longer read as one.
  Across 943k moving frames its distribution under ACC and under the driver is
  indistinguishable (38.8% vs 51.9% at zero, both saturating above 26000), so no
  threshold separates them; reading it as a press denied controls in 98%+ of
  ACC-engaged frames while protecting against nothing. `gas_pressed` now comes only
  from the camera's `ACC_CMD.GAS_PRESSED` bit, which is set in under 1% of frames
  even under full throttle. A real driver-pedal signal still has to be captured,
  most likely from a bus not present in the current logs.
- `ACC_CMD` full-stop uses the stock hold encoding (`CMD=400`, `ACCEL_ON=0`,
  `STOPPED=1`, `ACC_STATE=2`), derived from 10 hold episodes across 192 route
  segments. Panda permits it only while the car is already stopped. Not yet
  confirmed on-vehicle.
- `LKAS_STATE` (`0x307`) is now transmitted by openpilot and the stock copy is
  blocked from forwarding once RX health is trusted. Cluster behaviour with the
  substituted frame is unverified.
- `HUD_ALERT` (`0x3FC`) is relayed the same way. While the driver torque override holds
  lateral off, openpilot sets `ICA_WARNING=6`, which the owner identified as the cluster's
  take-over warning; otherwise the camera's frame goes out verbatim. Byte 7 is CRC-8 over
  bytes 0-6, checked on one frame. Cluster behaviour with the substituted frame is unverified.
- `steerRatio` 17 is fitted from locationd yaw rate against measured wheel angle
  on one 8-minute urban route (r=0.98, 16.8-17.2 across 11-32 kph); it has not been
  checked above 32 kph. `steerActuatorDelay` 0.15 follows the 130ms command-to-angle
  lag measured on the same route. Panda's `steer_ratio` must be changed with them or
  the VM angle limits diverge from the controller's.
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
- The driver's labelled captures place EPS-fault candidates in `0x40F` byte 0
  (`0xD5` at both logged steer failures) and EPB/auto hold/HDC in `0x537`,
  `0x51D` and `0x502`. None is parsed yet.

Publication rule: publish owner-provided evidence only after owner approval, and
strip route IDs, URLs, tokens, VINs, locations, timestamps, and raw identifying
captures before publication.
