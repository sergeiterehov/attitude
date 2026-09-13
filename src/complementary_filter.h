/**
 * Кватернионный комплементарный фильтр с адаптивной коррекцией акселерометром.
 * 
 * Особенности:
 * - Детектор манёвра: ||a| - 1g| > 0.05g ИЛИ |ω| > 5°/с
 * - В манёвре: accel_weight = 0, чистая интеграция гироскопа
 * - Оценка bias гироскопа в прямом полёте, заморозка в манёвре
 * - Мониторинг дрейфа → FAIL при превышении лимита
 */

#pragma once

#include <math.h>

class ComplementaryFilter {
 private:
  // Кватернион ориентации
  float q0, q1, q2, q3;

  // Оценка bias гироскопа (rad/s)
  float bias_x, bias_y, bias_z;

  // Состояние детектора манёвра
  bool in_maneuver;
  float maneuver_timer;  // время в манёвре (сек)

  // Мониторинг дрейфа
  float drift_estimate;  // накопленный дрейф (градусы)
  bool attitude_valid;

  // Параметры
  float sample_freq;
  float dt;

  // Пороги
  static constexpr float ACCEL_DEVIATION_THRESHOLD = 0.05f;  // g
  static constexpr float OMEGA_THRESHOLD = 5.0f * M_PI / 180.0f;  // rad/s (5 deg/s)
  static constexpr float MAX_DRIFT_DEG = 5.0f;  // градусы
  static constexpr float ALPHA_STRAIGHT = 0.95f;  // вес гироскопа в прямом полёте
  static constexpr float BIAS_LPF_TAU = 10.0f;  // постоянная времени LPF для bias (сек)

  // Вспомогательные
  float last_maneuver_exit_time;  // время последнего выхода из манёвра

 public:
  ComplementaryFilter() {
    q0 = 1.0f;
    q1 = q2 = q3 = 0.0f;
    bias_x = bias_y = bias_z = 0.0f;
    in_maneuver = false;
    maneuver_timer = 0.0f;
    drift_estimate = 0.0f;
    attitude_valid = true;
    sample_freq = 100.0f;
    dt = 0.01f;
    last_maneuver_exit_time = 0.0f;
  }

  void begin(float freq) {
    sample_freq = freq;
    dt = 1.0f / freq;
  }

  void update(float gx, float gy, float gz, float ax, float ay, float az) {
    // 1. Детектор манёвра
    float a_mag = sqrtf(ax * ax + ay * ay + az * az);
    float a_mag_g = a_mag / 9.81f;  // в g
    float deviation = fabsf(a_mag_g - 1.0f);

    float omega_mag = sqrtf(gx * gx + gy * gy + gz * gz);

    bool maneuver_detected = (deviation > ACCEL_DEVIATION_THRESHOLD) || (omega_mag > OMEGA_THRESHOLD);

    // Hysteresis: если в манёвре, требуем меньшее отклонение для выхода
    if (in_maneuver) {
      maneuver_detected = (deviation > ACCEL_DEVIATION_THRESHOLD * 0.5f) || (omega_mag > OMEGA_THRESHOLD * 0.5f);
    }

    // 2. Коррекция bias гироскопа
    float gx_corr = gx - bias_x;
    float gy_corr = gy - bias_y;
    float gz_corr = gz - bias_z;

    // 3. Интеграция гироскопа → кватернион
    integrateGyro(gx_corr, gy_corr, gz_corr);

    // 4. Обновление фильтра
    if (maneuver_detected || in_maneuver) {
      // В манёвре: чистая интеграция, акселерометр отключён
      if (!in_maneuver) {
        // Вход в манёвр
        in_maneuver = true;
        maneuver_timer = 0.0f;
      }

      maneuver_timer += dt;

      // Накапливаем дрейф
      float bias_rate = sqrtf(bias_x * bias_x + bias_y * bias_y + bias_z * bias_z) * 180.0f / M_PI;  // deg/s
      drift_estimate = bias_rate * maneuver_timer;

      // Проверка FAIL
      if (drift_estimate > MAX_DRIFT_DEG) {
        attitude_valid = false;
      }
    } else {
      // Прямой полёт: коррекция акселерометром
      if (in_maneuver) {
        // Выход из манёвра
        in_maneuver = false;
        last_maneuver_exit_time = maneuver_timer;
        maneuver_timer = 0.0f;
      }

      // Бленд с акселерометром
      correctFromAccel(ax, ay, az);

      // Сброс дрейфа
      drift_estimate = 0.0f;
      attitude_valid = true;

      // Обновление оценки bias
      updateBiasEstimate(gx, gy, gz, ax, ay, az);
    }

    // Нормализация кватерниона
    normalizeQuaternion();
  }

  float getRollRadians() const {
    // Извлечение roll из кватерниона
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    return atan2f(sinr_cosp, cosr_cosp);
  }

  float getPitchRadians() const {
    // Извлечение pitch из кватерниона
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f) {
      return copysignf(M_PI / 2.0f, sinp);  // use 90 degrees if out of range
    }
    return asinf(sinp);
  }

  float getRoll() const { return getRollRadians() * 180.0f / M_PI; }
  float getPitch() const { return getPitchRadians() * 180.0f / M_PI; }

  bool isValid() const { return attitude_valid; }
  float getDriftEstimate() const { return drift_estimate; }
  bool isInManeuver() const { return in_maneuver; }

 private:
  void integrateGyro(float gx, float gy, float gz) {
    // Интеграция угловых скоростей → обновление кватерниона
    // q_dot = 0.5 * q ⊗ ω
    float half_dt = 0.5f * dt;
    float dq0 = (-q1 * gx - q2 * gy - q3 * gz) * half_dt;
    float dq1 = (q0 * gx + q2 * gz - q3 * gy) * half_dt;
    float dq2 = (q0 * gy - q1 * gz + q3 * gx) * half_dt;
    float dq3 = (q0 * gz + q1 * gy - q2 * gx) * half_dt;

    q0 += dq0;
    q1 += dq1;
    q2 += dq2;
    q3 += dq3;
  }

  void correctFromAccel(float ax, float ay, float az) {
    // Вычисление кватерниона из акселерометра
    // Акелерометр измеряет g в body frame
    float a_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (a_norm < 1e-6f) return;

    // Нормализация
    float ax_n = ax / a_norm;
    float ay_n = ay / a_norm;
    float az_n = az / a_norm;

    // Гравитация в world frame: (0, 0, 1) (вверх)
    // В body frame должна быть (ax_n, ay_n, az_n)
    // Вычисляем кватернион, который поворачивает (0,0,1) в (ax_n, ay_n, az_n)

    // Вектор (0,0,1) в кватернионе: q = (0, 0, 0, 1)
    // Поворот: q_accel такой, что q_accel ⊗ (0,0,1) ⊗ q_accel* = (ax_n, ay_n, az_n)

    // Упрощённый метод: вычисляем pitch и roll из акселерометра, затем кватернион
    float pitch_accel = asinf(-ax_n);  // pitch от акселерометра
    float roll_accel = atan2f(ay_n, az_n);  // roll от акселерометра

    // Преобразование Euler → quaternion (только roll и pitch, yaw = 0)
    float cr = cosf(roll_accel * 0.5f);
    float sr = sinf(roll_accel * 0.5f);
    float cp = cosf(pitch_accel * 0.5f);
    float sp = sinf(pitch_accel * 0.5f);

    float q_accel_0 = cr * cp;
    float q_accel_1 = sr * cp;
    float q_accel_2 = cr * sp;
    float q_accel_3 = sr * sp;

    // Бленд: alpha * q_gyro + (1 - alpha) * q_accel
    float alpha = ALPHA_STRAIGHT;
    q0 = alpha * q0 + (1.0f - alpha) * q_accel_0;
    q1 = alpha * q1 + (1.0f - alpha) * q_accel_1;
    q2 = alpha * q2 + (1.0f - alpha) * q_accel_2;
    q3 = alpha * q3 + (1.0f - alpha) * q_accel_3;
  }

  void updateBiasEstimate(float gx, float gy, float gz, float ax, float ay, float az) {
    // Оценка bias гироскопа в прямом полёте
    // Идея: сравниваем gyro с ожидаемым от акселерометра

    // В прямом полёте акселерометр показывает g вдоль body Z
    // Ожидаемые угловые скорости = 0 (если нет вращения)
    // Разница = bias

    // Упрощённый метод: LPF от raw gyro
    float alpha = dt / BIAS_LPF_TAU;
    bias_x = (1.0f - alpha) * bias_x + alpha * gx;
    bias_y = (1.0f - alpha) * bias_y + alpha * gy;
    bias_z = (1.0f - alpha) * bias_z + alpha * gz;
  }

  void normalizeQuaternion() {
    float norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm > 1e-6f) {
      q0 /= norm;
      q1 /= norm;
      q2 /= norm;
      q3 /= norm;
    }
  }
};
