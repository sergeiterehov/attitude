#include "sensors.h"

#include <Arduino.h>
#include <math.h>

#if USE_BNO
#include <Adafruit_BNO08x.h>
static Adafruit_BNO08x bno08x;
static sh2_SensorValue_t sensorValue;
#endif

#if USE_BARO
#include <BMP085.h>
static BMP085 barometer;
#endif

AppState app_state;

#if USE_BNO
static void _init_bno() {
  bno08x.enableReport(SH2_GYRO_INTEGRATED_RV, 5000);
  bno08x.enableReport(SH2_ACCELEROMETER);
}
#endif

void initImu() {
#if USE_BNO
  if (!bno08x.begin_I2C(0x4B, &Wire)) {
    Serial.println("BNO08x not found at 0x4B");
    while (1);
  }

  _init_bno();

  Serial.println("BNO08x OK");
#endif

#if USE_BARO
  Serial.print("Barometer...");
  barometer.initialize();
  app_state.baro_available = barometer.testConnection();
  Serial.println(app_state.baro_available ? "OK" : "FAIL");
#endif
}

void calibrateImu() {
#if USE_BARO
  if (!app_state.baro_available) return;
  barometer.setControl(BMP085_MODE_TEMPERATURE);
  barometer.getTemperatureC();
  barometer.setControl(BMP085_MODE_PRESSURE_3);
  app_state.sea_level_pressure = barometer.getPressure();
#endif
}

void update_attitude() {
  static unsigned long prev_measure_at = 0;

  unsigned long now = millis();

  prev_measure_at = now;

#if USE_BNO
  if (bno08x.wasReset()) _init_bno();

  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.status > 0) app_state.attitude.calibration = sensorValue.status;

    if (sensorValue.sensorId == SH2_GYRO_INTEGRATED_RV) {
      auto sn = sensorValue.un.gyroIntegratedRV;

      // 1. Получаем компоненты исходного кватерниона
      auto qr = sn.real;
      auto qi = sn.i;
      auto qj = sn.j;
      auto qk = sn.k;

      // 2. Задаем кватернион поправки (Sensor -> Body)
      const float ow = 0.5f;
      const float ox = -0.5f;
      const float oy = -0.5f;
      const float oz = -0.5f;

      // 3. Выполняем умножение кватернионов: q_corrected = sn * q_offset
      float cw = qr * ow - qi * ox - qj * oy - qk * oz;
      float cx = qr * ox + qi * ow + qj * oz - qk * oy;
      float cy = qr * oy - qi * oz + qj * ow + qk * ox;
      float cz = qr * oz + qi * oy - qj * ox + qk * ow;

      // 4. Используем исправленные компоненты (cw, cx, cy, cz) для расчета углов
      float sqr = cw * cw;
      float sqi = cx * cx;
      float sqj = cy * cy;
      float sqk = cz * cz;

      app_state.attitude.pitch = asin(2.0 * (cx * cz - cy * cw) / (sqi + sqj + sqk + sqr));
      app_state.attitude.roll = -1.0f * atan2(2.0 * (cy * cz + cx * cw), (-sqi - sqj + sqk + sqr));
      app_state.attitude.heading = -atan2(2.0 * (cx * cy + cz * cw), (sqi - sqj - sqk + sqr));
    } else if (sensorValue.sensorId == SH2_ACCELEROMETER) {
      // Небольшой фильтр, чтобы сгладить колебания
      constexpr float AK = 0.8;
      app_state.attitude.skid = AK * app_state.attitude.skid + (1.0f - AK) * -1.0f * (sensorValue.un.accelerometer.x / 5.0f);
    }
  }

#if USE_BARO
  if (app_state.baro_available) {
    barometer.setControl(BMP085_MODE_TEMPERATURE);
    barometer.getTemperatureC();
    barometer.setControl(BMP085_MODE_PRESSURE_3);
    float pressure = barometer.getPressure();
    app_state.attitude.altitude = 44330.0f * (1.0f - powf(pressure / app_state.sea_level_pressure, 0.190284f));
  }
#endif
#endif

#ifdef USE_DEMO
  app_state.attitude.roll = PI * 0.5 * sin((float)now / 2000.0);
  // app_state.attitude.pitch = PI * 0.5 * sin((float)now / 2000.0);
#endif
}

void run_attitude_task(void* args) {
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = 10;

  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    update_attitude();

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}
