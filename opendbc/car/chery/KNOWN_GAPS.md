# Chery Omoda E5 parser gaps

- Door state unavailable. DBC exposes `DOOR` (746) only as unnamed signals;
  `BCM_SIGNAL_1` door bits have unconfirmed target semantics.
- Seatbelt state unavailable. DBC exposes unnamed `SEATBELT` in `NEW_MSG_430`
  (1072), with no confirmed latch semantics.
- FCW state unavailable. No confirmed FCW signal exists in route parser set;
  `SETTING.SHOW_AEB` is not treated as FCW.

These fields intentionally remain false as an owner-approved unsupported
compatibility behavior. They are not verified indications that vehicle systems
are healthy and must not block verified controls.

Future evidence required:

- Capture and decode door and seatbelt frames with confirmed semantics.
- Capture FCW behavior separately from AEB and confirm route signal mapping.
- Capture EPS fault and watchdog inputs before claiming steer-fault handling.
- Physical steer ratio and rack range require owner-labeled measurements.
- Raw ACC command to physical acceleration mapping requires owner-labeled
  measurements.
- Stock AEB interaction requires hardware validation.
- Stage C3, Panda bench, and controlled-drive validation before expanding support.
- `RES_PLUS` TX is now allowed on `0x360`, camera bus only, and only while the car
  is stopped with controls authorized and no other button bit asserted. The tap
  cadence (4 frames on, 10 off, at 20Hz) is taken from the driver's own measured
  presses. Neither the cadence nor the TX path has been confirmed on-vehicle.
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
- `steerRatio` 14 and `steerActuatorDelay` 0.2 are carried over from the fork that
  drives this car, not measured. Panda's `steer_ratio` must be changed with them or
  the VM angle limits diverge from the controller's.

Publication rule: publish owner-provided evidence only after owner approval, and
strip route IDs, URLs, tokens, VINs, locations, timestamps, and raw identifying
captures before publication.
