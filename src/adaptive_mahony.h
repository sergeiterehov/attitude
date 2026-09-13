/**
 * Adaptive Mahony AHRS filter.
 *
 * Модификация MahonyAHRS (PaulStoffregen) с адаптивными коэффициентами:
 * - setKp/setKi — динамическое изменение gains
 * - freezeIntegral() — заморозка интеграла без обнуления (для манёвров)
 * - unfreezeIntegral() — разморозка
 *
 * Оригинальный алгоритм: Mahony, Madgwick.
 * http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=4608934
 */

#pragma once

#include <math.h>

#define DEFAULT_SAMPLE_FREQ 512.0f
#define twoKpDef (2.0f * 0.5f)
#define twoKiDef (2.0f * 0.1f)

class AdaptiveMahony {
 private:
  float twoKp;
  float twoKi;
  float q0, q1, q2, q3;
  float integralFBx, integralFBy, integralFBz;
  float invSampleFreq;
  float roll, pitch, yaw;
  char anglesComputed;
  bool integralFrozen;

  static float invSqrt(float x);
  void computeAngles();

 public:
  AdaptiveMahony() {
    twoKp = twoKpDef;
    twoKi = twoKiDef;
    q0 = 1.0f;
    q1 = q2 = q3 = 0.0f;
    integralFBx = integralFBy = integralFBz = 0.0f;
    invSampleFreq = 1.0f / DEFAULT_SAMPLE_FREQ;
    anglesComputed = 0;
    integralFrozen = false;
  }

  void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }

  void setKp(float kp) { twoKp = 2.0f * kp; }
  void setKi(float ki) { twoKi = 2.0f * ki; }
  float getKp() const { return twoKp / 2.0f; }
  float getKi() const { return twoKi / 2.0f; }

  void freezeIntegral() { integralFrozen = true; }
  void unfreezeIntegral() { integralFrozen = false; }
  bool isIntegralFrozen() const { return integralFrozen; }

  float getIntegralX() const { return integralFBx; }
  float getIntegralY() const { return integralFBy; }
  float getIntegralZ() const { return integralFBz; }

  void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

  float getRoll() {
    if (!anglesComputed) computeAngles();
    return roll * 57.29578f;
  }
  float getPitch() {
    if (!anglesComputed) computeAngles();
    return pitch * 57.29578f;
  }
  float getYaw() {
    if (!anglesComputed) computeAngles();
    return yaw * 57.29578f + 180.0f;
  }
  float getRollRadians() {
    if (!anglesComputed) computeAngles();
    return roll;
  }
  float getPitchRadians() {
    if (!anglesComputed) computeAngles();
    return pitch;
  }
  float getYawRadians() {
    if (!anglesComputed) computeAngles();
    return yaw;
  }
};
