from .fixtures import ENGINE_FW, GOLDEN_FRAMES, ROUTE_SOURCE, RX_LAYOUT, VIN_WMI


def test_route_identity():
  assert VIN_WMI == "MF7"
  assert ENGINE_FW == b"00.02.12"
  assert "route" not in ROUTE_SOURCE.lower()
  assert "dongle" not in ROUTE_SOURCE.lower()


def test_golden_frames_cover_safety_layout():
  assert set(GOLDEN_FRAMES) == {
    (address, bus) for address, (bus, _dlc, _frequency) in RX_LAYOUT.items()
  }
  assert all(bus == RX_LAYOUT[address][0] for address, bus in GOLDEN_FRAMES)
  assert all(len(frames) >= 2 for frames in GOLDEN_FRAMES.values())
  assert all(
    all(len(frame) == RX_LAYOUT[address][1] for frame in frames)
    for (address, _bus), frames in GOLDEN_FRAMES.items()
  )
