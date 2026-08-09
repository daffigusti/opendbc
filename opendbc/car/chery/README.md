# Chery Omoda E5

Chery Omoda E5 support includes lateral control. Alpha longitudinal control is
default OFF; it requires explicit enable and closed-course validation.

## Verified CAN layout

| Direction | Signal/frame | Bus | DLC | Frequency |
| --- | --- | ---: | ---: | ---: |
| RX | `0x03E` | 0 | 48 | 100 Hz |
| RX | `0x1D3` | 0 | 8 | 100 Hz |
| RX | `0x316` | 0 | 8 | 50 Hz |
| RX | `0x394` | 0 | 8 | 50 Hz |
| RX | `0x3A2` | 2 | 8 | 50 Hz |
| RX | `0x3A5` | 2 | 8 | 50 Hz |
| TX | steering command `0x345` | 0 | 8 | 50 Hz |
| TX | ACC command `0x3A2` (alpha-long) | 0 | 8 | 50 Hz |

Active steering commands are capped at +/-150 degrees. Inactive steering
commands support the representable range through +/-370.4 degrees. Stock
steering is dynamically passed through when measured rack angle is outside
that representable range. During detected stock AEB, OEM `0x3A2` is dynamically
passed through and host ACC transmission is inhibited.

## Focused validation

```sh
pytest opendbc_repo/opendbc/car/chery/tests -v
pytest opendbc_repo/opendbc/safety/tests/test_chery.py -v
```

This support is not a production-ready, ISO, or road-ready claim.
