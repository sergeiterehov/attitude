#include "sensors.h"

#include <Arduino.h>
#include <math.h>

#include <MahonyAHRS.h>

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
static Mahony mahony;
static float last_ax = 0, last_ay = 0, last_az = 0;
static float last_gx = 0, last_gy = 0, last_gz = 0;
static bool have_accel = false, have_gyro = false;
static unsigned long last_debug_us = 0;

static void _init_bno() {
  bno08x.enableReport(SH2_GYRO_INTEGRATED_RV, 5000);
  bno08x.enableReport(SH2_ACCELEROMETER, 10000);
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
}

// Sensor → Body: sensor X=right, Y=forward, Z=up
//   body_X (forward) = -sensor_Z
//   body_Y (left)    = -sensor_X
//   body_Z (up)      =  sensor_Y
static inline void sensorToBody(float vx, float vy, float vz, float& ox, float& oy, float& oz) {
  ox = -vz;
  oy = -vx;
  oz = vy;
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
  mahony.begin(100.0f);
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
  unsigned long now = micros();

#if USE_BNO
  if (bno08x.wasReset()) _init_bno();

  while (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.status > 0) app_state.attitude.calibration = sensorValue.status;

    if (sensorValue.sensorId == SH2_GYRO_INTEGRATED_RV) {
      auto& sn = sensorValue.un.gyroIntegratedRV;
      float qr = sn.real, qi = sn.i, qj = sn.j, qk = sn.k;

      // Body-frame correction: q_corrected = sn * q_offset
      float cw = qr * 0.5f + qi * 0.5f + qj * 0.5f + qk * 0.5f;
      float cx = -qr * 0.5f + qi * 0.5f - qj * 0.5f + qk * 0.5f;
      float cy = -qr * 0.5f + qi * 0.5f + qj * 0.5f - qk * 0.5f;
      float cz = -qr * 0.5f - qi * 0.5f + qj * 0.5f + qk * 0.5f;

      float sqr = cw * cw, sqi = cx * cx, sqj = cy * cy, sqk = cz * cz;
      app_state.attitude.ref_pitch = asinf(2.0f * (cx * cz - cy * cw) / (sqi + sqj + sqk + sqr));
      app_state.attitude.ref_roll = -atan2f(2.0f * (cy * cz + cx * cw), -sqi - sqj + sqk + sqr);

    } else if (sensorValue.sensorId == SH2_ACCELEROMETER) {
      auto& a = sensorValue.un.accelerometer;
      float bx, by, bz;
      sensorToBody(a.x, a.y, a.z, bx, by, bz);
      last_ax = bx;
      last_ay = by;
      last_az = bz;
      have_accel = true;

      // Skid/slip — без изменений
      constexpr float AK = 0.8f;
      app_state.attitude.skid = AK * app_state.attitude.skid + (1.0f - AK) * -1.0f * (a.x / 5.0f);

    } else if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      auto& g = sensorValue.un.gyroscope;
      float bx, by, bz;
      sensorToBody(g.x, g.y, g.z, bx, by, bz);
      last_gx = bx;
      last_gy = by;
      last_gz = bz;
      have_gyro = true;
    }
  }

  // Mahony update (библиотека ожидает gyro в deg/s)
  if (have_accel && have_gyro) {
    mahony.updateIMU(last_gx * RAD_TO_DEG, last_gy * RAD_TO_DEG, last_gz * RAD_TO_DEG, last_ax, last_ay, last_az);
    app_state.attitude.roll = mahony.getRollRadians();
    app_state.attitude.pitch = mahony.getPitchRadians();
  }

  // Отладка: сравнение нашего решения с BNO085
  if (now - last_debug_us >= 200000) {
    last_debug_us = now;
    Serial.printf("Mahony: r=%.1f p=%.1f | BNO: r=%.1f p=%.1f\n",
                  app_state.attitude.roll * 57.2958f,
                  app_state.attitude.pitch * 57.2958f,
                  app_state.attitude.ref_roll * 57.2958f,
                  app_state.attitude.ref_pitch * 57.2958f);
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
