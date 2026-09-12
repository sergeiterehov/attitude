#include <Arduino.h>

#include "LovyanGFX.h"
#ifndef BOARD
#error "BOARD is not defined"
#endif
#if BOARD == BOARD_UEDX4848
#include "LGFX_UEDX4848.h"
#endif
#if BOARD == BOARD_YCD
#include "LGFX_YCD.h"
#endif

#include "Wire.h"
#include "sensors.h"
#include "ui_altitude.h"
#include "ui_attitude.h"
#include "ui_compass.h"
#include "ui_status.h"

LGFX display;
lgfx::LGFX_Sprite canvas(&display);

void setRGB(uint8_t r, uint8_t g, uint8_t b) {
#ifdef LED_RED
  digitalWrite(LED_RED, r ? LOW : HIGH);
  digitalWrite(LED_GREEN, g ? LOW : HIGH);
  digitalWrite(LED_BLUE, b ? LOW : HIGH);
#endif
}

void render_ui() {
  canvas.setTextSize(1);

  draw_attitude(&canvas, app_state);
  draw_compass(&canvas, app_state);
  draw_altitude(&canvas, app_state);
  draw_status(&canvas, app_state);

  canvas.pushSprite(0, 0);
}

void run_ui_task(void* args) {
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = 33;

  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    display.getTouch(&app_state.pointer.x, &app_state.pointer.y);

    render_ui();
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

void setup() {
#if BOOT_BTN != -1
  pinMode(BOOT_BTN, INPUT_PULLUP);
#endif

  setRGB(0, 0, 0);

#ifdef LED_RED
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
#endif

  Serial.begin(115200);

#if BOARD == BOARD_YCD
  display.setRotation(2);
#endif

  if (!display.init()) ESP_LOGE("MAIN", "TFT INIT FAIL");

  display.startWrite();
  display.clear(TFT_BLACK);
  display.setTextColor(TFT_GREEN);
  display.drawString("TEREHOV RESEARCH", 120, 100);
  display.light()->setBrightness(255);
  display.endWrite();

  canvas.setColorDepth(lgfx::v1::color_depth_t::rgb332_1Byte);
  canvas.createSprite(display.width(), display.height());

  Wire.begin();
  Wire.setClock(400000);

  initImu();
  delay(50);
  calibrateImu();

  xTaskCreatePinnedToCore(run_ui_task, "UI", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(run_attitude_task, "Attitude", 4096, NULL, 2, NULL, 0);
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
