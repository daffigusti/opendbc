from .fixtures import ENGINE_FW, GOLDEN_FRAMES, ROUTE_SOURCE, RX_LAYOUT, VIN_WMI


def test_route_identity():
  assert ROUTE_SOURCE == "private owner full rlog, segment 0"
  assert VIN_WMI == "MF7"
  assert len(VIN_WMI) == 3
  assert ENGINE_FW == b"00.02.12"


def test_golden_frames_cover_safety_layout():
  assert set(GOLDEN_FRAMES) == {
    (address, bus) for address, (bus, _dlc, _frequency) in RX_LAYOUT.items()
  }

  for address, (bus, dlc, _frequency) in RX_LAYOUT.items():
    key = (address, bus)
    assert key in GOLDEN_FRAMES, f"missing golden frames for 0x{address:03X} bus {bus}"
    frames = GOLDEN_FRAMES[key]
    assert len(frames) == 2, f"0x{address:03X} bus {bus}: expected 2 frames"
    for frame_index, frame in enumerate(frames):
      assert len(frame) == dlc, f"0x{address:03X} bus {bus} frame {frame_index}: expected DLC {dlc}, got {len(frame)}"
