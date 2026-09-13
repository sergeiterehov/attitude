/**
 * Интерфейс фильтра ориентации для тестирования.
 * Позволяет переключаться между разными реализациями (Mahony, custom, etc.)
 */

#pragma once

struct FilterOutput {
  float roll;   // rad
  float pitch;  // rad
};

class FilterBackend {
 public:
  virtual ~FilterBackend() {}
  virtual void begin(float sampleFreq) = 0;
  virtual FilterOutput update(float gx, float gy, float gz, float ax, float ay, float az, float dt) = 0;
  virtual const char* name() = 0;
};
