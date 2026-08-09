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
- Owner-provided CAN capture verified ACC cancel. `RES_PLUS` TX remains
  unverified; `0x360` remains denied/forwarded.
- Driver torque override is decode-only. Sign and threshold require owner-labeled
  stationary correlation before enforcement; `steeringPressed` remains false until
  verified. Acceptance of override enforcement is intentionally deferred.

Publication rule: publish owner-provided evidence only after owner approval, and
strip route IDs, URLs, tokens, VINs, locations, timestamps, and raw identifying
captures before publication.
