# Milestone: working recon UI (3 Sep 2026)

Frozen working firmware. Restore this before adding motors or other features.

## Build

- PlatformIO env: `esp32s3_simple_test`
- Source file: `src/main_camera_stream.cpp`
- Git tag (if repo exists): `milestone-2026-09-03-working-ui`

## Working features

- OV3660 latest-frame JPEG view (HVGA 480x320, shown in a 16:9 box)
- BMI270 pitch / roll / yaw on the web page (`/imu`)
- WS2812B 6-LED middle-out animation
- LED on/off toggle
- LED color picker (all LEDs same RGB)
- Spotlight on/off
- Wi-Fi AP `R6_Recon_Drone` / `ReconDrone123`
- Web UI: `http://192.168.4.1`

## Intentionally not in this milestone

- INMP441 microphone (removed)
- Motor drive / PID balancing
- Encoders
- HD camera (quality traded for low delay)

## How to restore

1. Check out tag `milestone-2026-09-03-working-ui`, or copy files from `milestones/2026-09-03-working-ui/`
2. Build and upload `esp32s3_simple_test`
