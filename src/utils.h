#pragma once

#include "math.h"

float round_angle(float angle) {
  float res = angle;
  while (res > M_PI) res -= 2.0f * M_PI;
  while (res < -M_PI) res += 2.0f * M_PI;
  return res;
}
