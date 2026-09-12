#define LGFX_USE_V1
#include <driver/i2c.h>

#include <LovyanGFX.hpp>

#include "lgfx/v1/panel/Panel_ILI9342.hpp"

class LGFX : public lgfx::LGFX_Device {
 public:
  lgfx::Light_PWM _light;
  lgfx::Bus_SPI _bus;
  lgfx::Panel_ILI9342 _panel;
  lgfx::Touch_XPT2046 _touch;

  LGFX(void) {
    {
      auto cfg = _light.config();
      cfg.pin_bl = TFT_BACKLIGHT;
      cfg.invert = false;

      _light.config(cfg);
    }

    {
      auto cfg = _bus.config();
      cfg.freq_write = 80000000;
      cfg.freq_read = 16000000;
      cfg.spi_host = SPI2_HOST;
      cfg.pin_dc = TFT_DC;
      cfg.pin_sclk = TFT_SCLK;
      cfg.pin_miso = TFT_MISO;
      cfg.pin_mosi = TFT_MOSI;

      _bus.config(cfg);
    }

    {
      auto cfg = _panel.config();
      cfg.pin_cs = TFT_CS;
      cfg.pin_rst = TFT_RST;
      cfg.invert = true;

      _panel.setBus(&_bus);
      _panel.config(cfg);
    }

    {
      auto cfg = _touch.config();

      cfg.x_min = 400;
      cfg.x_max = 3800;
      cfg.y_min = 400;
      cfg.y_max = 3800;

      cfg.pin_cs = 33;
      cfg.pin_int = 36;

      cfg.bus_shared = true;
      cfg.spi_host = SPI2_HOST;

      _touch.config(cfg);
      _panel.setTouch(&_touch);
    }

    _panel.light(&_light);

    setPanel(&_panel);
  }
};
