#include <Arduino.h>
#include <BMP085.h>
#include <I2Cdev.h>
#include <MPU6050.h>

#include "LovyanGFX.h"
#include "Wire.h"
#include "lgfx/v1/panel/Panel_ILI9342.hpp"
#include "matrix2d.h"

#define TFT_BACKLIGHT 27
#define TFT_DC 2
#define TFT_CS 15
#define TFT_RST 0
#define TFT_MISO 12
#define TFT_MOSI 13
#define TFT_SCLK 14

#define MAG_USE_QMC5883P 1

lgfx::LGFX_Device tft;
lgfx::LGFX_Sprite* canvas;

MPU6050 mpu;
BMP085 barometer;
#ifdef MAG_USE_HMC5883L
#include <HMC5883L.h>
HMC5883L mag;
#endif
#ifdef MAG_USE_QMC5883P
#include <QMC5883P.h>
QMC5883P mag;
#endif

#define STD_PRESSURE 101325.0f

struct imu_data_t {
  struct {
    float x;
    float y;
    float z;
  } accel;
  struct {
    float x;
    float y;
    float z;
  } gyro;
  struct {
    float x;
    float y;
    float z;
  } mag;
  float pressure, altitude;
} imu;

void initImu() {
  Wire.begin();

  Serial.print("Accel/gyro...");
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "OK" : "FAIL");

  mpu.setI2CBypassEnabled(true);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setDLPFMode(MPU6050_DLPF_BW_98);

  Serial.print("Compass...");
  mag.initialize();
  Serial.println(mag.testConnection() ? "OK" : "FAIL");

  Serial.print("Barometer...");
  barometer.initialize();
  Serial.println(barometer.testConnection() ? "OK" : "FAIL");
}

struct {
  const float as = 4096.0, gs = 16.4, ms = 32768.0;
  float sea = STD_PRESSURE;
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
} z_imu;

void calibrateImu() {
  const int samples = 100;
  int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
  int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

  int16_t ax, ay, az, gx, gy, gz, mx, my, mz;

  for (int i = 0; i < samples; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    sum_ax += ax;
    sum_ay += ay;
    sum_az += az;

    sum_gx += gx;
    sum_gy += gy;
    sum_gz += gz;

    delay(5);
  }

  z_imu.ax = (sum_ax / (float)samples);
  z_imu.ay = (sum_ay / (float)samples);
  z_imu.az = (sum_az / (float)samples) - z_imu.as;

  z_imu.gx = (sum_gx / (float)samples);
  z_imu.gy = (sum_gy / (float)samples);
  z_imu.gz = (sum_gz / (float)samples);

  mag.getHeading(&mx, &my, &mz);

  // TODO: calculate min/max for getting zero value! zero is not zero field
  z_imu.mx = mx;
  z_imu.my = z_imu.ms - my;
  z_imu.mz = mz;

  barometer.setControl(BMP085_MODE_TEMPERATURE);
  barometer.getTemperatureC();
  barometer.setControl(BMP085_MODE_PRESSURE_3);
  z_imu.sea = barometer.getPressure();
}

void getImuData(imu_data_t& data) {
  static int16_t ax, ay, az;
  static int16_t gx, gy, gz;
  static int16_t mx, my, mz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  data.accel.x = (ax - z_imu.ax) / z_imu.as;
  data.accel.y = (ay - z_imu.ay) / z_imu.as;
  data.accel.z = (az - z_imu.az) / z_imu.as;

  data.gyro.x = (gx - z_imu.gx) / z_imu.gs;
  data.gyro.y = (gy - z_imu.gy) / z_imu.gs;
  data.gyro.z = (gz - z_imu.gz) / z_imu.gs;

  mag.getHeading(&mx, &my, &mz);
  Serial.printf("%i %i %i\n", mx, my, mz);

  data.mag.x = (mx - z_imu.gx) / z_imu.ms;
  data.mag.y = (my - z_imu.gy) / z_imu.ms;
  data.mag.z = (mz - z_imu.gz) / z_imu.ms;

  barometer.setControl(BMP085_MODE_TEMPERATURE);
  barometer.getTemperatureC();
  barometer.setControl(BMP085_MODE_PRESSURE_3);
  data.pressure = barometer.getPressure();
  data.altitude = 44330 * (1.0 - pow(data.pressure / z_imu.sea, 0.190284));
}

#define ALPHA 0.98f

struct {
  float bank = 0, pitch = 0, skid = 0, heading = 0, altitude = 0;
} attitude;

float round_angle(float angle) {
  float res = angle;
  while (res > M_PI) res -= 2.0f * M_PI;
  while (res < -M_PI) res += 2.0f * M_PI;
  return res;
}

void update_attitude() {
  static ulong prev_measure_at = 0;

  ulong now = millis();

  float acc_angle, dt;

  getImuData(imu);

  dt = (now - prev_measure_at) / 1000.0f;
  prev_measure_at = now;

  attitude.skid = -imu.accel.x;

  attitude.bank += -imu.gyro.y / 180.0f * PI * dt;
  acc_angle = atan2f(imu.accel.x, imu.accel.z);
  attitude.bank = round_angle(ALPHA * (attitude.bank) + (1.0f - ALPHA) * acc_angle);

  attitude.pitch += -imu.gyro.x / 180.0f * PI * dt;
  acc_angle = atan2f(-imu.accel.y, imu.accel.z);
  attitude.pitch = round_angle(ALPHA * (attitude.pitch) + (1.0f - ALPHA) * acc_angle);

  attitude.heading = atan2f(-imu.mag.y, imu.mag.x);

  attitude.altitude = imu.altitude;
}

void render_attitude() {
  float w = canvas->width();
  float h = canvas->height();

  canvas->setTextSize(1);

  auto a = attitude.bank;
  auto cx = w / 2.0;
  auto cy = h / 2.0;

  canvas->clear(TFT_BLUE);

  // Sky + Ground
  {
    canvas->setColor(TFT_BROWN);

    Mat2D t;
    mat2d_identity(&t);
    mat2d_translate(&t, cx, cy);
    mat2d_rotate(&t, attitude.bank);
    mat2d_translate(&t, 0, -attitude.pitch * 180.0f / PI * (h / 40));

    float px, py;
    mat2d_transform_point(&t, 0, 0, &px, &py);

    // Предрасчет нормали к прямой (nx, ny) один раз для экономии ресурсов
    // Прямая проходит через (cx, cy) под углом a.
    // Вектор направления прямой: (cos(a), sin(a))
    // Нормальный вектор (перпендикуляр): (-sin(a), cos(a))
    float nx = -sinf(a);
    float ny = cosf(a);

    // Константа уравнения прямой: C = nx*cx + ny*cy
    // Уравнение: nx*x + ny*y = C
    // Нам нужно закрасить область, где nx*x + ny*y > C (или < C, зависит от стороны)
    float c_val = nx * px + ny * py;

    for (int y = 0; y < h; y += 1) {
      // Вычисляем часть уравнения, зависящую от Y: valY = ny*y - c_val
      // Тогда условие nx*x > c_val - ny*y  =>  nx*x > -valY
      float valY = ny * y - c_val;

      int x_start = 0;
      int x_end = w;

      // Определяем границу пересечения с текущей строкой сканирования
      if (fabsf(nx) > 1e-6) {  // Проверка, чтобы не делить на ноль, если прямая горизонтальна
        float x_intersect = -valY / nx;

        if (nx > 0) {
          // Если нормаль смотрит вправо, закрашиваем справа от пересечения
          x_start = (int)ceilf(x_intersect);
          if (x_start < 0) x_start = 0;
          if (x_start > w) x_start = w;
        } else {
          // Если нормаль смотрит влево, закрашиваем слева от пересечения
          x_end = (int)floorf(x_intersect);
          if (x_end < 0) x_end = 0;
          if (x_end > w) x_end = w;
        }
      } else {
        // Прямая горизонтальна (nx == 0).
        // Если ny > 0 и valY < 0 (y меньше границы), то закрашиваем всю строку или ничего
        // В данном случае, если nx=0, то условие зависит только от Y.
        // Если мы должны закрасить сторону, где неравенство выполняется для любого X:
        if ((ny > 0 && -valY < 0) || (ny < 0 && -valY > 0)) {
          x_start = 0;
          x_end = w;
        } else {
          x_start = w;
          x_end = 0;  // Пустая строка
        }
      }

      if (x_end > x_start) {
        canvas->writeFastHLine(x_start, y, x_end - x_start);
      }
    }
  }

  // Pitch scale
  {
    Mat2D t;
    mat2d_identity(&t);
    mat2d_translate(&t, cx, cy);
    mat2d_rotate(&t, attitude.bank);
    mat2d_translate(&t, 0, -attitude.pitch * 180.0f / PI * (h / 40));

    auto style = canvas->getTextStyle();
    style.size_x = 1;
    style.size_y = style.size_x;
    style.back_rgb888 = style.fore_rgb888;
    canvas->setTextStyle(style);

    auto textHeight = canvas->fontHeight();

    float ax, ay, bx, by;

    mat2d_transform_point(&t, -w, 0, &ax, &ay);
    mat2d_transform_point(&t, w, 0, &bx, &by);
    canvas->drawLine(ax, ay, bx, by, TFT_WHITE);

    for (int i = -900; i <= 900; i += 25) {
      if (i == 0) continue;

      float dy = i * 0.1 * (h / 40.0);
      float len = 40.0f * (i % 100 == 0 ? 1.0 : i % 50 == 0 ? 0.5 : 0.12);

      mat2d_transform_point(&t, -len / 2.0, dy, &ax, &ay);
      mat2d_transform_point(&t, len / 2.0, dy, &bx, &by);

      if (ay < 0 && by < 0) continue;
      if (ay > h && by > h) break;

      canvas->drawLine(ax, ay, bx, by, TFT_WHITE);

      if (i != 0 && i % 100 == 0) {
        char text[4] = {0};
        sprintf(text, "%i", abs(i / 10));
        auto textWidth = canvas->textWidth(text);
        canvas->drawString(text, ax - 5 - textWidth, ay - textHeight / 2);
        canvas->drawString(text, bx + 5, by - textHeight / 2);
      }
    }
  }

  // Roll
  {
    Mat2D t;
    mat2d_identity(&t);
    mat2d_translate(&t, cx, cy);

    float ax, ay, bx, by, cx, cy;

    const float r = h / 2 - 30;

    mat2d_transform_point(&t, 0, 0, &ax, &ay);
    canvas->drawArc(ax, ay, r, r, -60 - 90, 60 - 90, TFT_WHITE);

    for (int k = -1; k < 2; k += 2) {
      for (int j = 0; j <= 60; j += (j < 30 ? 10 : 15)) {
        if (j == 0) continue;

        int i = k * j;

        Mat2D rt = t;
        mat2d_rotate(&rt, i * PI / 180 - PI);

        mat2d_transform_point(&rt, 0, r, &ax, &ay);
        mat2d_transform_point(&rt, 0, r + (j % 30 == 0 ? 8 : 4), &bx, &by);
        canvas->drawLine(ax, ay, bx, by, TFT_WHITE);
      }

      {
        Mat2D rt = t;
        mat2d_rotate(&rt, -PI);

        mat2d_transform_point(&rt, 0, r, &cx, &cy);
        mat2d_transform_point(&rt, -2, r + 4, &ax, &ay);
        mat2d_transform_point(&rt, 2, r + 4, &bx, &by);
        canvas->fillTriangle(ax, ay, bx, by, cx, cy, TFT_WHITE);
      }
    }

    // Roll pointer
    {
      Mat2D rt = t;
      mat2d_rotate(&rt, attitude.bank - PI);

      mat2d_transform_point(&rt, 0, r, &cx, &cy);
      mat2d_transform_point(&rt, -6, r - 10, &ax, &ay);
      mat2d_transform_point(&rt, 6, r - 10, &bx, &by);
      canvas->fillTriangle(ax, ay, bx, by, cx, cy, TFT_YELLOW);
      canvas->drawTriangle(ax, ay, bx, by, cx, cy, TFT_BLACK);
    }
  }

  // Bird
  {
    Mat2D t;
    mat2d_identity(&t);
    mat2d_translate(&t, cx, cy);

    float offset = 50.0;
    float wing = 10.0;
    float ax, ay, bx, by;

    mat2d_transform_point(&t, offset, 0, &ax, &ay);
    mat2d_transform_point(&t, offset + wing, 0, &bx, &by);
    canvas->drawWideLine(ax, ay, bx, by, 3, TFT_BLACK);
    canvas->drawWideLine(ax, ay, bx, by, 2, TFT_YELLOW);

    mat2d_transform_point(&t, -offset, 0, &ax, &ay);
    mat2d_transform_point(&t, -offset - wing, 0, &bx, &by);
    canvas->drawWideLine(ax, ay, bx, by, 3, TFT_BLACK);
    canvas->drawWideLine(ax, ay, bx, by, 2, TFT_YELLOW);

    mat2d_transform_point(&t, 0, 0, &ax, &ay);
    mat2d_transform_point(&t, 30, 15, &bx, &by);
    canvas->drawWedgeLine(ax, ay, bx, by, 1, 4, TFT_BLACK);
    canvas->drawWedgeLine(ax, ay, bx, by, 0, 3, TFT_YELLOW);
    mat2d_transform_point(&t, -30, 15, &bx, &by);
    canvas->drawWedgeLine(ax, ay, bx, by, 1, 4, TFT_BLACK);
    canvas->drawWedgeLine(ax, ay, bx, by, 0, 3, TFT_YELLOW);
  }

  // Slip/skid
  {
    float r = 10;
    float d = r * 2;
    float lw = 2;

    Mat2D t;
    mat2d_identity(&t);
    mat2d_translate(&t, cx, h - d);

    float ax, ay, bx, by;

    mat2d_transform_point(&t, d * attitude.skid, 0, &ax, &ay);
    canvas->fillCircle(ax, ay, r, TFT_WHITE);
    canvas->drawCircle(ax, ay, r + 1, TFT_BLACK);

    mat2d_transform_point(&t, -r - 1 - lw, -r, &ax, &ay);
    mat2d_transform_point(&t, -r - 1 - lw, r, &bx, &by);
    canvas->drawWideLine(ax, ay, bx, by, lw, TFT_BLACK);
    canvas->drawWideLine(ax, ay, bx, by, lw - 1, TFT_WHITE);

    mat2d_transform_point(&t, r + 1 + lw, -r, &ax, &ay);
    mat2d_transform_point(&t, r + 1 + lw, r, &bx, &by);
    canvas->drawWideLine(ax, ay, bx, by, lw, TFT_BLACK);
    canvas->drawWideLine(ax, ay, bx, by, lw - 1, TFT_WHITE);
  }

  // Heading
  {
    Mat2D t;
    mat2d_identity(&t);
    mat2d_translate(&t, cx, 0);

    float ax, ay, bx, by;

    int ph = 24;
    char str[32];

    float hdg = abs(180.0 * attitude.heading / PI);

    int range = 50;
    for (int i = 0; i < 360; i += 1) {
      if (i < hdg - range / 2 || i > hdg + range / 2) continue;

      int lh = i % 10 == 0 ? 5 : i % 5 == 0 ? 3 : 1;

      mat2d_transform_point(&t, (w - 100) * (i - hdg) / range, 0, &ax, &ay);
      canvas->drawLine(ax, ay + ph - lh, ax, ay + ph, TFT_WHITE);

      if (i % 10 == 0) {
        sprintf(str, "%03i", abs(i));
        int tw = canvas->textWidth(str);
        canvas->drawString(str, ax - tw / 2, ay);
      }
    }

    {
      auto _text_style = canvas->getTextStyle();
      sprintf(str, "%03.0f", abs(hdg));
      mat2d_transform_point(&t, 0, 0, &ax, &ay);
      canvas->setTextSize(2);
      int tw = canvas->textWidth(str);
      canvas->fillRect(ax - tw / 2 - 4, ay - 2, tw + 8, canvas->fontHeight() + 4, TFT_BLACK);
      canvas->drawString(str, ax - tw / 2, ay);
      canvas->setTextStyle(_text_style);
    }
  }

  // Altitude
  {
    int pw = 50;

    Mat2D t;
    mat2d_identity(&t);
    mat2d_translate(&t, w - pw, cy);

    float ax, ay, bx, by;

    mat2d_transform_point(&t, -1, 0, &ax, &ay);
    canvas->drawLine(ax, ay - h / 2, ax, ay + h / 2, TFT_BLACK);

    float alt = attitude.altitude * 3.28084;  // ft
    char str[32];

    int step = 100;
    int range = 4 * step;
    int range_center = (int)((alt) / step) * step;

    // Red line
    {
      int start = min(0, range_center + range / 2);

      mat2d_transform_point(&t, 0, -h * (start - alt) / range, &ax, &ay);
      canvas->fillRect(ax, ay, 10, h, TFT_RED);
    }

    for (int i = max(0, range_center - range / 2); i <= range_center + range / 2; i += step) {
      for (int sub_step = step / 5, j = i + sub_step; j < i + step; j += sub_step) {
        mat2d_transform_point(&t, 0, -h * (j - alt) / range, &ax, &ay);
        canvas->drawLine(ax, ay, ax + 2, ay, TFT_WHITE);
      }

      mat2d_transform_point(&t, 0, -h * (i - alt) / range, &ax, &ay);
      canvas->drawLine(ax, ay, ax + 5, ay, TFT_WHITE);

      sprintf(str, "%i", i);
      canvas->drawString(str, ax + 8, ay - canvas->fontHeight() / 2);
    }

    // Alt cursor
    {
      auto _text_style = canvas->getTextStyle();

      bool is_alt_k = abs(alt) >= 1000;
      bool is_alt_dk = is_alt_k && abs(alt) >= 10000;

      mat2d_transform_point(&t, 0, 0, &ax, &ay);

      canvas->setTextSize(2);
      int bg_h = canvas->fontHeight();
      canvas->fillRect(ax + 6 - 4, ay - bg_h / 2 - 2, pw - 6 + 8, bg_h + 4, TFT_BLACK);

      sprintf(str, "%i", (int)(alt) % 1000);
      canvas->setTextSize(is_alt_dk ? 1 : is_alt_k ? 1.5 : 2);
      int tw = canvas->textWidth(str);
      int th = canvas->fontHeight();
      canvas->drawRightString(str, ax + pw, ay + bg_h / 2 - th + 1);

      if (is_alt_k) {
        sprintf(str, "%i", (int)(abs((int)alt) / 1000));
        canvas->setTextSize(2);
        th = canvas->fontHeight();
        canvas->drawRightString(str, ax + pw - tw, ay + bg_h / 2 - th + 1);
      }

      canvas->setTextStyle(_text_style);
    }

    // Pressure
    {
      auto _text_style = canvas->getTextStyle();

      int bg_h = canvas->fontHeight() + 2;
      mat2d_transform_point(&t, 0, h / 2 - bg_h, &ax, &ay);
      if (z_imu.sea == STD_PRESSURE) {
        sprintf(str, "=STD=");
      } else {
        sprintf(str, "%i", (int)(z_imu.sea / 100));
      }
      int tw = canvas->textWidth(str);
      canvas->fillRect(ax, ay, pw, bg_h, TFT_BLACK);
      canvas->setTextColor(TFT_CYAN);
      canvas->drawString(str, ax + 4, ay + bg_h - canvas->fontHeight());

      canvas->setTextStyle(_text_style);
    }
  }

  canvas->pushSprite(0, 0);
}

void run_ui_task() {
  static ulong now = 0, prev_render_at = 0;
  static imu_data_t imu;

  for (;;) {
    now = millis();

    update_attitude();

    if (now - prev_render_at > 33) {
      prev_render_at = now;
      render_attitude();
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  initImu();
  delay(50);
  calibrateImu();

  auto _light = new lgfx::Light_PWM();
  auto _bus = new lgfx::Bus_SPI();
  auto _panel = new lgfx::Panel_ILI9342();

  {
    auto cfg = _light->config();
    cfg.pin_bl = TFT_BACKLIGHT;
    cfg.invert = false;

    _light->config(cfg);
    _light->init(255);
  }

  {
    auto cfg = _bus->config();
    cfg.freq_write = 80000000;
    cfg.freq_read = 16000000;
    cfg.spi_host = SPI2_HOST;
    cfg.pin_dc = TFT_DC;
    cfg.pin_sclk = TFT_SCLK;
    cfg.pin_miso = TFT_MISO;
    cfg.pin_mosi = TFT_MOSI;

    _bus->config(cfg);
  }

  {
    auto cfg = _panel->config();
    cfg.pin_cs = TFT_CS;
    cfg.pin_rst = TFT_RST;
    cfg.invert = true;

    _panel->setBus(_bus);
    _panel->config(cfg);

    tft.setPanel(_panel);
  }

  if (!tft.init()) ESP_LOGE("MAIN", "TFT INIT FAIL");

  tft.setRotation(2);

  canvas = new lgfx::LGFX_Sprite(&tft);
  canvas->setColorDepth(lgfx::v1::color_depth_t::rgb332_1Byte);
  canvas->createSprite(tft.width(), tft.height());
  canvas->setTextScroll(true);

  run_ui_task();
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
