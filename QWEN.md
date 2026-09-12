# Attitude — ESP32 Aviation Attitude Indicator

## Project Overview

ESP32-based attitude indicator (авиагоризонт) for automotive use. Renders a full flight instrument display on a TFT screen: pitch, roll, heading, altitude, and slip/skid. Two FreeRTOS tasks run on separate cores — sensor fusion at 100 Hz (core 0) and UI rendering at ~30 fps (core 1).

**Hardware:**
- **MCU:** ESP32 / ESP32-S3
- **IMU:** BNO085 (I²C, address 0x4B) — uses `SH2_GYRO_INTEGRATED_RV` mode (gyro-trusted, magnetometer-corrected yaw)
- **Barometer:** BMP085 (altitude from pressure)
- **Display:** TFT via LovyanGFX (two board configs: YCD and UEDX4848)
- **Magnetometer:** QMC5883P (legacy/alternative, currently not primary heading source)

**Sensor fusion:** BNO085 internal fusion (quaternion → Euler extraction with a body-frame correction quaternion `(0.5, -0.5, -0.5, -0.5)`). Madgwick AHRS implementation exists in `MadgwickAHRS.cpp` but is not currently used — available as fallback for custom fusion on raw data if needed.

## Build & Flash

PlatformIO project. Two environments:

```bash
pio run -e YCD          # build for YCD board (ESP32, BNO085 + baro)
pio run -e UEDX4848     # build for UEDX4848 board (ESP32-S3, demo mode)
pio run -e YCD -t upload
pio device monitor      # 115200 baud
```

## Architecture

- `src/main.cpp` — main application: IMU init, attitude computation, UI rendering, FreeRTOS tasks
- `src/MadgwickAHRS.{cpp,h}` — Madgwick filter (unused, available for custom fusion)
- `src/matrix2d.h` — 2D affine transforms for instrument rendering
- `src/utils.h` — utility helpers
- `src/LGFX_YCD.h` / `src/LGFX_UEDX4848.h` — display configurations per board
- `src/QMC5883P.{cpp,h}` — QMC5883P magnetometer driver

## Key Technical Details

- **BNO085 mode:** `SH2_GYRO_INTEGRATED_RV` with 5 ms report interval. Chosen over `ARVR_STABILIZED_RV` because the latter trusts accelerometer too aggressively, causing false pitch/roll during acceleration/braking in a car.
- **Heading:** magnetometer-based (magnetic north), sign-inverted due to body-frame quaternion correction. Rendered as [0°, 360°) with circular wrapping.
- **Skid/slip:** low-pass filtered accelerometer X axis (α=0.8).
- **Canvas:** `rgb332_1Byte` color depth to save RAM.
- **Baro altitude:** calibrated at startup against sea-level standard pressure (101325 Pa).

## Code Style

- clang-format based on Google style, 120 column limit
- C++ (Arduino framework), comments in Russian
- Build flags select features: `USE_BNO`, `USE_BARO`, `USE_DEMO`, `BOARD`
