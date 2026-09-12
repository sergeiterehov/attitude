#include "ui_attitude.h"

#include <math.h>
#include "matrix2d.h"

void draw_attitude(lgfx::LGFX_Sprite* canvas, const AppState& state) {
  const auto& att = state.attitude;
  float w = canvas->width();
  float h = canvas->height();

  auto cx = w / 2.0;
  auto cy = h / 2.0;

  // Sky + Ground
  {
    canvas->clear(TFT_BLUE);

    Mat2D t;
    mat2d_identity(&t);
    mat2d_translate(&t, cx, cy);
    mat2d_rotate(&t, -att.roll);
    mat2d_translate(&t, 0, -att.pitch * 180.0f / PI * (h / 40));

    float ax, ay;

    auto a = att.roll;

    mat2d_transform_point(&t, 0, 0, &ax, &ay);

    canvas->setColor(TFT_BROWN);

    float nx = sinf(a);
    float ny = cosf(a);

    float c_val = nx * ax + ny * ay;

    for (int y = 0; y < h; y += 1) {
      float valY = ny * y - c_val;

      int x_start = 0;
      int x_end = w;

      if (fabsf(nx) > 1e-6) {
        float x_intersect = -valY / nx;

        if (nx > 0) {
          x_start = (int)ceilf(x_intersect);
          if (x_start < 0) x_start = 0;
          if (x_start > w) x_start = w;
        } else {
          x_end = (int)floorf(x_intersect);
          if (x_end < 0) x_end = 0;
          if (x_end > w) x_end = w;
        }
      } else {
        if ((ny > 0 && -valY < 0) || (ny < 0 && -valY > 0)) {
          x_start = 0;
          x_end = w;
        } else {
          continue;
        }
      }

      if (x_end > x_start) canvas->writeFastHLine(x_start, y, x_end - x_start);
    }

    // Horizon line
    float bx, by;
    mat2d_transform_point(&t, -w, 0, &ax, &ay);
    mat2d_transform_point(&t, w, 0, &bx, &by);
    canvas->drawLine(ax, ay, bx, by, TFT_WHITE);
  }

  // BNO085 reference horizon (зелёная линия)
  {
    Mat2D t;
    mat2d_identity(&t);
    mat2d_translate(&t, cx, cy);
    mat2d_rotate(&t, -state.attitude.ref_roll);
    mat2d_translate(&t, 0, -state.attitude.ref_pitch * 180.0f / PI * (h / 40));

    float ax, ay, bx, by;
    mat2d_transform_point(&t, -w, 0, &ax, &ay);
    mat2d_transform_point(&t, w, 0, &bx, &by);
    canvas->drawLine(ax, ay, bx, by, TFT_GREEN);
  }

  // Pitch scale
  {
    Mat2D t;
    mat2d_identity(&t);
    mat2d_translate(&t, cx, cy);
    mat2d_rotate(&t, -att.roll);
    mat2d_translate(&t, 0, -att.pitch * 180.0f / PI * (h / 40));

    auto style = canvas->getTextStyle();
    style.size_x = 1;
    style.size_y = style.size_x;
    style.back_rgb888 = style.fore_rgb888;
    canvas->setTextStyle(style);

    auto textHeight = canvas->fontHeight();

    float ax, ay, bx, by;

    for (int i = -900; i <= 900; i += 25) {
      if (i == 0) continue;

      float dy = i * 0.1 * (h / 40.0);
      float len = 40.0f * (i % 100 == 0 ? 1.0 : i % 50 == 0 ? 0.5 : 0.12);

      mat2d_transform_point(&t, -len / 2.0, dy, &ax, &ay);
      mat2d_transform_point(&t, len / 2.0, dy, &bx, &by);

      if (ay < 0 && by < 0 || ay > h && by > h) continue;

      canvas->drawLine(ax, ay, bx, by, TFT_WHITE);

      auto textWidth_2 = canvas->textWidth("00") / 2;

      mat2d_transform_point(&t, -len / 2.0 - 5 - textWidth_2, dy, &ax, &ay);
      mat2d_transform_point(&t, len / 2.0 + 5 + textWidth_2, dy, &bx, &by);

      if (i != 0 && i % 100 == 0) {
        char text[4] = {0};
        sprintf(text, "%i", abs(i / 10));
        canvas->drawString(text, ax - textWidth_2, ay - textHeight / 2);
        canvas->drawString(text, bx - textWidth_2, by - textHeight / 2);
      }
    }
  }

  // Roll
  {
    Mat2D t;
    mat2d_identity(&t);
    mat2d_translate(&t, cx, cy);

    float ax, ay, bx, by, px, py;

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

        mat2d_transform_point(&rt, 0, r, &px, &py);
        mat2d_transform_point(&rt, -2, r + 4, &ax, &ay);
        mat2d_transform_point(&rt, 2, r + 4, &bx, &by);
        canvas->fillTriangle(ax, ay, bx, by, px, py, TFT_WHITE);
      }
    }

    // Roll pointer
    {
      Mat2D rt = t;
      mat2d_rotate(&rt, PI - att.roll);

      mat2d_transform_point(&rt, 0, r, &px, &py);
      mat2d_transform_point(&rt, -6, r - 10, &ax, &ay);
      mat2d_transform_point(&rt, 6, r - 10, &bx, &by);
      canvas->fillTriangle(ax, ay, bx, by, px, py, TFT_YELLOW);
      canvas->drawTriangle(ax, ay, bx, by, px, py, TFT_BLACK);
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

    mat2d_transform_point(&t, d * att.skid, 0, &ax, &ay);
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
}
