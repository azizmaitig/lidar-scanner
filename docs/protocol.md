# Protocols

## LiDAR Packet Format

36-byte packets streamed over UART at 115200/230400 baud.

```
Offset  Size  Description
0       1     Sync byte (0x55)
1       5     Reserved / header
6       2     Counter (uint16 LE) — increments per packet, wraps on rotation
8       24    8 samples × 3 bytes (see below)
32      4     Footer / reserved
```

### Sample Format (3 bytes each)

```
Byte 0      Quality (0-255)
Byte 1-2    Distance (uint16 BE) × 0.25 mm
            If raw >= 0x8000, distance is invalid (0)
```

### Rotation Detection

Counter drops by >1000 between consecutive packets → new rotation started.

## ESP8266 TCP Stream

The ESP8266 runs a TCP server. On connect it streams:

- Raw 36-byte LiDAR packets (passthrough from UART)
- Encoder position packets every rotation (or periodically)

Format: `0xAA` + type byte + payload.

### Encoder Position Packet

```
0xAA   0x01   timestamp_32   position_s32   speed_f32
  1      1         4              4             4
```

## WebSocket JSON (PC → Browser)

```json
{
  "type": "scan",
  "rotation": 42,
  "samples": 512,
  "valid": 380,
  "tilt_angle": 15.5,
  "points": [
    {"angle": 0.0, "distance": 196.0, "quality": 13},
    ...
  ]
}
```
