ROUTE_SOURCE = "private owner full rlog, segment 0"
VIN_WMI = "MF7"
ENGINE_FW = b"00.02.12"

RX_LAYOUT = {
  0x03E: (0, 48, 100),
  0x1D3: (0, 8, 100),
  0x29A: (0, 8, 50),
  0x316: (0, 8, 50),
  0x360: (0, 6, 20),
  0x394: (0, 8, 50),
  0x4ED: (0, 8, 10),
  0x345: (2, 8, 50),
  0x3A2: (2, 8, 50),
  0x3A5: (2, 8, 50),
}

GOLDEN_FRAMES = {
  (0x03E, 0): (
    bytes.fromhex("bc0680007ccd80006c0680647869741d0d067cf215040000ab0644d000002000ab067fd9400000000000000000000000"),
    bytes.fromhex("680780007cc880003b0780647865741d7c077cee15040000f60744d000002000f6077fd9400000000000000000000000"),
  ),
  (0x1D3, 0): (
    bytes.fromhex("78c0010000000523"),
    bytes.fromhex("78c000000000066e"),
  ),
  (0x29A, 0): (
    bytes.fromhex("000002c000800e10"),
    bytes.fromhex("000002c400800f43"),
  ),
  (0x316, 0): (
    bytes.fromhex("057e057e8c115efe"),
    bytes.fromhex("059105988c115fd0"),
  ),
  (0x345, 2): (
    bytes.fromhex("78c4000cbefe2f27"),
    bytes.fromhex("78c800000000c01f"),
  ),
  (0x360, 0): (
    bytes.fromhex("338000000000"),
    bytes.fromhex("dd9000000000"),
  ),
  (0x394, 0): (
    bytes.fromhex("17b000000800038a"),
    bytes.fromhex("14b000000800043e"),
  ),
  (0x3A2, 2): (
    bytes.fromhex("7d1102027f710f57"),
    bytes.fromhex("7d1102027f7100ec"),
  ),
  (0x3A5, 2): (
    bytes.fromhex("0000000000000fb1"),
    bytes.fromhex("000000000000000a"),
  ),
  (0x4ED, 0): (
    bytes.fromhex("1c0a1e0060040562"),
    bytes.fromhex("1c0a1e0060040645"),
  ),
}
