/**
 * Обёртка над AdaptiveMahony для тестирования.
 * Реализует адаптивную логику: Kp зависит от отклонения |a| от 1g,
 * интеграл замораживается в манёвре.
 */

#pragma once

#include "filter_backend.h"
#include "adaptive_mahony.h"
#include <math.h>

class AdaptiveMahonyBackend : public FilterBackend {
  AdaptiveMahony mahony;

  // Параметры адаптации
  static constexpr float KP_NOMINAL = 0.5f;    // Kp в прямом полёте
  static constexpr float KP_MIN = 0.001f;      // минимальный Kp в манёвре
  static constexpr float ADAPTATION_K = 50.0f; // крутизна адаптации
  static constexpr float DEVIATION_THRESHOLD = 0.03f;  // g, порог манёвра

 public:
  void begin(float sampleFreq) override {
    mahony.begin(sampleFreq);
    mahony.setKp(KP_NOMINAL);
    mahony.setKi(0.1f);
  }

  FilterOutput update(float gx, float gy, float gz, float ax, float ay, float az, float dt) override {
    // Вычисление отклонения |a| от 1g
    float a_mag = sqrtf(ax * ax + ay * ay + az * az);
    float a_mag_g = a_mag / 9.81f;
    float deviation = fabsf(a_mag_g - 1.0f);

    // Адаптивный Kp: плавно снижается при отклонении от 1g
    float adaptive_kp = KP_NOMINAL / (1.0f + ADAPTATION_K * deviation);
    if (adaptive_kp < KP_MIN) adaptive_kp = KP_MIN;
    mahony.setKp(adaptive_kp);

    // Заморозка интеграла в манёвре
    if (deviation > DEVIATION_THRESHOLD) {
      mahony.freezeIntegral();
    } else {
      mahony.unfreezeIntegral();
    }

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

  const char* name() override { return "AdaptiveMahony"; }
};
