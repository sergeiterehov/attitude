#include <Arduino.h>
#include <M5GFX.h>
#include <MPU6050.h>

#include "Panel.hpp"
#include "Wire.h"
#include "lgfx/v1/panel/Panel_ILI9342.hpp"
#include "matrix2d.h"

#define TFT_BACKLIGHT 27
#define TFT_DC 2     // Data/command pin
#define TFT_CS 15    // Chip-select pin
#define TFT_RST 0    // Reset pin
#define TFT_MISO 12  // ???
#define TFT_MOSI 13  // = SDA
#define TFT_SCLK 14  //

lgfx::LGFX_Device tft;
lgfx::LGFX_Sprite* canvas;

MPU6050 mpu;

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
};

void initImu() {
  Wire.begin();
  mpu.initialize();

  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setDLPFMode(MPU6050_DLPF_BW_256);
}

struct {
  const float as = 16384, gs = 131;
  float ax, ay, az;
  float gx, gy, gz;
} z_imu;

void calibrateImu() {
  const int samples = 100;
  int32_t sum_ax = 0, sum_ay = 0, sum_az = 0, sum_gx = 0, sum_gy = 0, sum_gz = 0;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
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
  z_imu.ay = (sum_ay / (float)samples) - z_imu.as;
  z_imu.az = (sum_az / (float)samples);

  z_imu.gx = (sum_gx / (float)samples);
  z_imu.gy = (sum_gy / (float)samples);
  z_imu.gz = (sum_gz / (float)samples);
}

void getImuData(imu_data_t& data) {
  static int16_t ax, ay, az;  // Acceleration variables
  static int16_t gx, gy, gz;  // Gyroscope variables

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  data.accel.x = (ax - z_imu.ax) / z_imu.as;
  data.accel.y = (ay - z_imu.ay) / z_imu.as;
  data.accel.z = (az - z_imu.az) / z_imu.as;

  data.gyro.x = (gx - z_imu.gx) / z_imu.gs;
  data.gyro.y = (gy - z_imu.gy) / z_imu.gs;
  data.gyro.z = (gz - z_imu.gz) / z_imu.gs;
}

#define ALPHA 0.98f

struct {
  float bank = 0, pitch = 0, skid = 0;
} attitude;

void update_attitude() {
  static ulong prev_measure_at = 0;
  static imu_data_t imu;

  ulong now = millis();

  float acc_angle, dt;

  getImuData(imu);

  dt = (now - prev_measure_at) / 1000.0f;
  prev_measure_at = now;

  attitude.skid = -imu.accel.x;

  attitude.bank += imu.gyro.z / 180.0f * PI * dt;
  acc_angle = atan2f(imu.accel.x, imu.accel.y);
  attitude.bank = ALPHA * (attitude.bank) + (1.0f - ALPHA) * acc_angle;

  attitude.pitch += -imu.gyro.x / 180.0f * PI * dt;
  acc_angle = atan2f(imu.accel.z, imu.accel.y);
  attitude.pitch = ALPHA * (attitude.pitch) + (1.0f - ALPHA) * acc_angle;

  // Нормализация угла в диапазон [-PI, PI] (опционально, для удобства отрисовки)
  while (attitude.bank > M_PI) attitude.bank -= 2.0f * M_PI;
  while (attitude.bank < -M_PI) attitude.bank += 2.0f * M_PI;
  // Нормализация угла в диапазон [-PI, PI] (опционально, для удобства отрисовки)
  while (attitude.pitch > M_PI) attitude.pitch -= 2.0f * M_PI;
  while (attitude.pitch < -M_PI) attitude.pitch += 2.0f * M_PI;

  // TODO: demo
  {
    // bank = sin(millis() / 3000.0) * PI / 4;
    // pitch = sin(millis() / 10000.0) * PI / 8;
  }
}

void run_ui_task() {
  static ulong now = 0, prev_render_at = 0;
  static imu_data_t imu;

  float w = canvas->width();
  float h = canvas->height();

  for (;;) {
    now = millis();

    update_attitude();

    if (now - prev_render_at > 33) {
      prev_render_at = now;

      auto a = attitude.bank;
      auto cx = w / 2.0;
      auto cy = h / 2.0;

      canvas->clear(BLUE);

      // Sky + Ground
      {
        canvas->setColor(BROWN);

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
        canvas->drawLine(ax, ay, bx, by, WHITE);

        for (int i = -900; i <= 900; i += 25) {
          if (i == 0) continue;

          float dy = i * 0.1 * (h / 40.0);
          float len = 40.0f * (i % 100 == 0 ? 1.0 : i % 50 == 0 ? 0.5 : 0.12);

          mat2d_transform_point(&t, -len / 2.0, dy, &ax, &ay);
          mat2d_transform_point(&t, len / 2.0, dy, &bx, &by);

          if (ay < 0 && by < 0) continue;
          if (ay > h && by > h) break;

          canvas->drawLine(ax, ay, bx, by, WHITE);

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

        mat2d_transform_point(&t, 0, 0, &ax, &ay);
        canvas->drawArc(ax, ay, h / 2 - 4, h / 2 - 4, -60 - 90, 60 - 90, WHITE);

        for (int k = -1; k < 2; k += 2) {
          for (int j = 0; j <= 60; j += (j < 30 ? 10 : 15)) {
            if (j == 0) continue;

            int i = k * j;

            Mat2D rt = t;
            mat2d_rotate(&rt, i * PI / 180 - PI);

            mat2d_transform_point(&rt, 0, h / 2 - 4, &ax, &ay);
            mat2d_transform_point(&rt, 0, h / 2 - 4 + (j % 30 == 0 ? 8 : 4), &bx, &by);
            canvas->drawLine(ax, ay, bx, by, WHITE);
          }

          {
            Mat2D rt = t;
            mat2d_rotate(&rt, -PI);

            mat2d_transform_point(&rt, 0, h / 2 - 4, &cx, &cy);
            mat2d_transform_point(&rt, -2, h / 2 - 4 + 4, &ax, &ay);
            mat2d_transform_point(&rt, 2, h / 2 - 4 + 4, &bx, &by);
            canvas->fillTriangle(ax, ay, bx, by, cx, cy, WHITE);
          }
        }

        // Roll pointer
        {
          Mat2D rt = t;
          mat2d_rotate(&rt, attitude.bank - PI);

          mat2d_transform_point(&rt, 0, h / 2 - 4, &cx, &cy);
          mat2d_transform_point(&rt, -4, h / 2 - 4 - 8, &ax, &ay);
          mat2d_transform_point(&rt, 4, h / 2 - 4 - 8, &bx, &by);
          canvas->fillTriangle(ax, ay, bx, by, cx, cy, WHITE);
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
        canvas->drawWideLine(ax, ay, bx, by, 3, BLACK);
        canvas->drawWideLine(ax, ay, bx, by, 2, YELLOW);

        mat2d_transform_point(&t, -offset, 0, &ax, &ay);
        mat2d_transform_point(&t, -offset - wing, 0, &bx, &by);
        canvas->drawWideLine(ax, ay, bx, by, 3, BLACK);
        canvas->drawWideLine(ax, ay, bx, by, 2, YELLOW);

        mat2d_transform_point(&t, 0, 0, &ax, &ay);
        mat2d_transform_point(&t, 30, 15, &bx, &by);
        canvas->drawWedgeLine(ax, ay, bx, by, 1, 4, BLACK);
        canvas->drawWedgeLine(ax, ay, bx, by, 0, 3, YELLOW);
        mat2d_transform_point(&t, -30, 15, &bx, &by);
        canvas->drawWedgeLine(ax, ay, bx, by, 1, 4, BLACK);
        canvas->drawWedgeLine(ax, ay, bx, by, 0, 3, YELLOW);
      }

      // Slip/skid
      {
        Mat2D t;
        mat2d_identity(&t);
        mat2d_translate(&t, cx, h - 10);

        float ax, ay, bx, by;

        mat2d_transform_point(&t, 10.0 * attitude.skid, 0, &ax, &ay);
        canvas->fillCircle(ax, ay, 5, WHITE);
        canvas->drawCircle(ax, ay, 6, BLACK);

        mat2d_transform_point(&t, -10, -5, &ax, &ay);
        mat2d_transform_point(&t, -10, 5, &bx, &by);
        canvas->drawWideLine(ax, ay, bx, by, 2, BLACK);
        canvas->drawWideLine(ax, ay, bx, by, 1, WHITE);

        mat2d_transform_point(&t, 10, -5, &ax, &ay);
        mat2d_transform_point(&t, 10, 5, &bx, &by);
        canvas->drawWideLine(ax, ay, bx, by, 2, BLACK);
        canvas->drawWideLine(ax, ay, bx, by, 1, WHITE);
      }

      canvas->pushSprite(0, 0);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  initImu();
  delay(300);
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
    cfg.freq_write = 60000000;
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

  tft.setRotation(0);

  canvas = new lgfx::LGFX_Sprite(&tft);
  canvas->setColorDepth(lgfx::v1::color_depth_t::rgb332_1Byte);
  canvas->createSprite(tft.width(), tft.height());
  canvas->setTextScroll(true);

  run_ui_task();
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
