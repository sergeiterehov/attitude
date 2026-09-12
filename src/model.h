#pragma once

#include <stdint.h>

#define STD_PRESSURE 101325.0f

struct Attitude {
  float roll = 0, pitch = 0, skid = 0, heading = 0, altitude = 0;
  uint8_t calibration = 0;
};

struct AppState {
  Attitude attitude;
  float sea_level_pressure = STD_PRESSURE;
  bool baro_available = false;
};
