/**
 * Обёртка над MahonyAHRS для тестирования.
 */

#pragma once

#include "filter_backend.h"

// MahonyAHRS library (PaulStoffregen fork)
// Включаем напрямую — библиотека не имеет Arduino-зависимостей
extern "C" {
// MahonyAHRS.h использует class, так что просто инклюдим
}
#include "MahonyAHRS.h"

class MahonyBackend : public FilterBackend {
  Mahony mahony;

 public:
  void begin(float sampleFreq) override { mahony.begin(sampleFreq); }

  FilterOutput update(float gx, float gy, float gz, float ax, float ay, float az, float dt) override {
    // Mahony ожидает gyro в deg/s
    float gx_dps = gx * 57.29578f;
    float gy_dps = gy * 57.29578f;
    float gz_dps = gz * 57.29578f;

    mahony.updateIMU(gx_dps, gy_dps, gz_dps, ax, ay, az);

    FilterOutput out;
    out.roll = mahony.getRollRadians();
    out.pitch = mahony.getPitchRadians();
    return out;
  }

  const char* name() override { return "Mahony"; }
};
