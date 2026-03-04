#include "QMC5883P.h"

// https://cdn-shop.adafruit.com/product-files/6388/C2847467.pdf

QMC5883P::QMC5883P() { devAddr = 0x2C; }
QMC5883P::QMC5883P(uint8_t address) { devAddr = address; }

void QMC5883P::initialize() {
  I2Cdev::writeByte(devAddr, 0x0A, 0xC3);  // Continuous mode, ODR=200Hz
  I2Cdev::writeByte(devAddr, 0x0B, 0x08);  // Range ±8G, Set/Reset On
}

bool QMC5883P::testConnection() {
  I2Cdev::readByte(devAddr, 0x00, buffer);
  return buffer[0] == 0x80;
}

void QMC5883P::getHeading(int16_t* x, int16_t* y, int16_t* z) {
  I2Cdev::readBytes(devAddr, 0x01, 6, buffer);

  *x = ((int16_t)buffer[1] << 8) + buffer[0];
  *y = ((int16_t)buffer[3] << 8) + buffer[2];
  *z = ((int16_t)buffer[5] << 8) + buffer[4];
}
