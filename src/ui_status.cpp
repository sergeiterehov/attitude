#include "ui_status.h"

void draw_status(lgfx::LGFX_Sprite* canvas, const AppState& state) {
  float h = canvas->height();
  const auto& att = state.attitude;
  char str[32];

  sprintf(str, "P=%.1f R=%.1f H=%.1f", att.pitch, att.roll, att.heading);
  canvas->drawString(str, 0, h - canvas->fontHeight());

  sprintf(str, "CAL:%i", att.calibration);
  canvas->drawString(str, 0, h - canvas->fontHeight() * 2);

#if BOOT_BTN != -1
  if (digitalRead(BOOT_BTN) == 0) {
    canvas->drawString("PRESSED", 0, 0);
  }
#endif
}
