/**
 * Обёртка над ComplementaryFilter для тестирования.
 */

#pragma once

#include "filter_backend.h"
#include "complementary_filter.h"

class ComplementaryBackend : public FilterBackend {
  ComplementaryFilter filter;

 public:
  void begin(float sampleFreq) override { filter.begin(sampleFreq); }

  FilterOutput update(float gx, float gy, float gz, float ax, float ay, float az, float dt) override {
    filter.update(gx, gy, gz, ax, ay, az);

    FilterOutput out;
    out.roll = filter.getRollRadians();
    out.pitch = filter.getPitchRadians();
    return out;
  }

  const char* name() override { return "Complementary"; }
};
