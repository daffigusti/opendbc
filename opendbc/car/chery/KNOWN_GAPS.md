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
- Capture button behavior for ACC/main and CC_BTN/cancel before adding mappings;
  current RES_PLUS/RES_MINUS edge mappings are the only verified button semantics.
- Driver torque override is decode-only. Sign and threshold require owner-labeled
  stationary correlation before enforcement; `steeringPressed` remains false until
  verified. Acceptance of override enforcement is intentionally deferred.
