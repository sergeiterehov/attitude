#include "input.h"

#include <Arduino.h>

#if BOARD == BOARD_YCD
#include <SPI.h>
#include <XPT2046_Touchscreen.h>

// Touchscreen pins for ESP32-2432S028R
#define XPT2046_IRQ 36   // T_IRQ
#define XPT2046_MOSI 32  // T_DIN
#define XPT2046_MISO 39  // T_OUT
#define XPT2046_CLK 25   // T_CLK
#define XPT2046_CS 33    // T_CS

static SPIClass touchscreenSPI = SPIClass(VSPI);
static XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);
#endif

void initTouch() {
#if BOARD == BOARD_YCD
  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touchscreen.begin(touchscreenSPI);
  touchscreen.setRotation(1);
  Serial.println("Touchscreen initialized.");
#endif
}

void handleTouch() {
#if BOARD == BOARD_YCD
  static unsigned long lastTouchTime = 0;
  unsigned long currentTime = millis();

  if (touchscreen.tirqTouched() && touchscreen.touched()) {
    TS_Point p = touchscreen.getPoint();

    int touchX = map(p.x, 200, 3700, 1, 240);
    int touchY = map(p.y, 240, 3800, 1, 320);
    int touchZ = p.z;

    static unsigned long lastLogTime = 0;
    if (currentTime - lastLogTime > 500) {
      Serial.print("Touch detected at X=");
      Serial.print(touchX);
      Serial.print(", Y=");
      Serial.print(touchY);
      Serial.print(", Z=");
      Serial.println(touchZ);
      lastLogTime = currentTime;
    }
  }
#endif
}
