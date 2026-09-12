#include "ui_compass.h"

#include <math.h>
#include "matrix2d.h"

void draw_compass(lgfx::LGFX_Sprite* canvas, const AppState& state) {
  float w = canvas->width();
  const auto& att = state.attitude;

  Mat2D t;
  mat2d_identity(&t);
  mat2d_translate(&t, w / 2.0, 0);

  float ax, ay;

  int ph = 24;
  char str[32];

  float hdg_deg = att.heading * 180.0f / PI;
  if (hdg_deg < 0) hdg_deg += 360.0f;

  int range = 50;
  for (int i = 0; i < 360; i += 1) {
    int diff = i - (int)hdg_deg;
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;
    if (abs(diff) > range / 2) continue;

    int lh = i % 10 == 0 ? 5 : i % 5 == 0 ? 3 : 1;

    mat2d_transform_point(&t, (w - 100) * diff / (float)range, 0, &ax, &ay);
    canvas->drawLine(ax, ay + ph - lh, ax, ay + ph, TFT_WHITE);

    if (i % 10 == 0) {
      sprintf(str, "%03i", i);
      int tw = canvas->textWidth(str);
      canvas->drawString(str, ax - tw / 2, ay);
    }
  }

  {
    auto _text_style = canvas->getTextStyle();
    sprintf(str, "%03.0f", hdg_deg);
    mat2d_transform_point(&t, 0, 0, &ax, &ay);
    canvas->setTextSize(2);
    int tw = canvas->textWidth(str);
    canvas->fillRect(ax - tw / 2 - 4, ay - 2, tw + 8, canvas->fontHeight() + 4, TFT_BLACK);
    canvas->drawString(str, ax - tw / 2, ay);
    canvas->setTextStyle(_text_style);
  }
}
