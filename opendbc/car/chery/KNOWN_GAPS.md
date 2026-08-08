# Chery Omoda E5 parser gaps

- Door state unavailable. DBC exposes `DOOR` (746) only as unnamed signals;
  `BCM_SIGNAL_1` door bits have unconfirmed target semantics.
- Seatbelt state unavailable. DBC exposes unnamed `SEATBELT` in `NEW_MSG_430`
  (1072), with no confirmed latch semantics.
- FCW state unavailable. No confirmed FCW signal exists in route parser set;
  `SETTING.SHOW_AEB` is not treated as FCW.

These fields intentionally remain false rather than guessing healthy state.
