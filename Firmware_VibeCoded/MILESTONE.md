# Milestone: breadboard drive (20 Sep 2026)

Frozen working firmware after the breadboard bring-up. Restore this before PID self-balance, core split changes, or other large features.

Previous lock (camera/LED/IMU/spotlight only): `milestone-2026-09-03-working-ui` and `milestones/2026-09-03-working-ui/`.

## Build

- PlatformIO env: `esp32s3_simple_test`
- Source file: `src/main_camera_stream.cpp`
- Git tag: `milestone-2026-09-20-breadboard-drive`
- Upload over **TTL USB-C (CH340) only**. Do not plug OTG USB-C (encoders sit on GPIO 19/20).

## Working features

- OV3660 latest-frame JPEG view (HVGA 480x320)
- BMI270 pitch / roll / yaw
- WS2812B 6-LED animation, on/off, RGB color picker
- Spotlight on/off
- Hold-to-drive FWD / BACK / LEFT / RIGHT (web pad + WASD / arrows)
- DRV8833 PWM on both motor channels (idle pin forced LOW, 5 kHz)
- Hall encoder counts on the web status line
- Wi-Fi AP `R6_Recon_Drone` / `ReconDrone123`
- Web UI: `http://192.168.4.1`

## Locked GPIO map

| Feature | GPIO |
|---|---|
| IMU SDA / SCL | 1 / 2 |
| LED strip DIN | 3 |
| Spotlight | 38 |
| Motor A AIN1 / AIN2 | 41 / 42 |
| Motor B BIN1 / BIN2 | 39 / 40 |
| Left encoder A / B | 19 / 20 |
| Right encoder A / B | 21 / 47 |
| Camera DVP | 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15, 16, 17, 18 |

Motor LEDC channels: 2, 3, 4, 5 (camera XCLK stays on 0, spotlight on 7).

## Known hardware note

- One physical motor on the breadboard is faulty. Swapping motors between driver outputs confirmed the firmware and both DRV8833 channels are good. Replace that motor; no pin change required.

## Intentionally not in this milestone

- PID / self-balancing (unsafe until both motors are healthy)
- Dual-core 100 Hz balance loop (still camera/LED/IMU on core 0, HTTP on core 1)
- INMP441 microphone (removed)
- HD camera (quality traded for low delay)

## How to restore

1. Check out tag `milestone-2026-09-20-breadboard-drive`, or copy files from `milestones/2026-09-20-breadboard-drive/`
2. Build and upload `esp32s3_simple_test`
