# lidar-scanner

> **Work in progress.** Low-cost 3D LiDAR scanner built from a repurposed Neato XV-11 LiDAR, ESP8266, and a DC motor with quadrature encoder.
<img width="4624" height="3468" alt="20260515_214652" src="https://github.com/user-attachments/assets/ffebda76-4261-4382-aba3-404ac93eeadb" />

# progress(experimenting the use of phone sensor as stabiliser ) 

<img width="1913" height="927" alt="Capture d&#39;écran 2026-05-14 224032" src="https://github.com/user-attachments/assets/1a6cbdfa-80d5-4707-ac71-274e193dd188" />

## Architecture

```
LiDAR UART ──→ ESP8266 (custom firmware) ──WiFi TCP──→ PC (Python server)
                        ↑
Encoder A/B ──→ GPIO ISR (quadrature decode)
Motor PWM  ←── H-bridge ←── GPIO
```

The ESP8266 multiplexes LiDAR data, encoder position (φ), and motor control over a single TCP stream. The PC fuses each 2D scan slice with the tilt angle to produce pseudo-3D point clouds.

## Quick Start

```bash
# Python deps
pip install -r requirements.txt

# Run server (viewer only, no LiDAR)
python server/server.py --lidar-disable

# Open http://localhost:8765
```

## Hardware

| Component | Notes |
|-----------|-------|
| LiDAR | Neato XV-11, UART 115200/230400 baud, 4.5° angular step |
| MCU | ESP8266, custom firmware (PlatformIO) |
| Motor | Brushed DC + quadrature encoder A/B, 400 PPR |
| Driver | H-bridge (DRV8833 / L298N) |
| Slip ring | For continuous rotation (power + UART + encoder) |

See [`docs/wiring.md`](docs/wiring.md) for pinouts and [`docs/BOM.md`](docs/BOM.md) for parts.

## Project Status

- [x] LiDAR packet decoding (0x55, 36-byte, 8 samples/packet)
- [x] 2D scan visualization (Three.js WebGL + Canvas fallback)
- [x] Offline parser → CSV + matplotlib plots
- [ ] ESP8266 custom firmware (UART + encoder + motor PWM)
- [ ] Encoder angle fusion → pseudo-3D point cloud
- [ ] Motor control via Web UI
- [ ] Open3D offline viewer
- [ ] SLAM experiments

## License

MIT
