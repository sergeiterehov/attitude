#pragma once

#include "I2Cdev.h"

class QMC5883P {
 public:
  QMC5883P();
  QMC5883P(uint8_t address);

  void initialize();
  bool testConnection();

  void getHeading(int16_t* x, int16_t* y, int16_t* z);

 private:
  uint8_t devAddr;
  uint8_t buffer[6];
};
