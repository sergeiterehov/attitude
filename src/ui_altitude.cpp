#include "ui_altitude.h"

#include <math.h>
#include "matrix2d.h"

void draw_altitude(lgfx::LGFX_Sprite* canvas, const AppState& state) {
  float w = canvas->width();
  float h = canvas->height();
  int pw = 50;

  auto cy = h / 2.0;

  Mat2D t;
  mat2d_identity(&t);
  mat2d_translate(&t, w - pw, cy);

  float ax, ay;

  mat2d_transform_point(&t, -1, 0, &ax, &ay);
  canvas->drawLine(ax, ay - h / 2, ax, ay + h / 2, TFT_BLACK);

  float alt = state.attitude.altitude * 3.28084;  // ft
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

    if (state.baro_available) {
      sprintf(str, "%i", (int)(alt) % 1000);
    } else {
      sprintf(str, "FAIL");
      canvas->setTextColor(TFT_RED);
    }
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
#if USE_BARO
    if (state.baro_available) {
      if (state.sea_level_pressure == STD_PRESSURE) {
        sprintf(str, "=STD=");
      } else {
        sprintf(str, "%i", (int)(state.sea_level_pressure / 100));
      }
    } else {
      sprintf(str, "---");
    }
#else
    sprintf(str, "---");
#endif
    int tw = canvas->textWidth(str);
    canvas->fillRect(ax, ay, pw, bg_h, TFT_BLACK);
    canvas->setTextColor(TFT_CYAN);
    canvas->drawString(str, ax + 4, ay + bg_h - canvas->fontHeight());

    canvas->setTextStyle(_text_style);
  }
}
