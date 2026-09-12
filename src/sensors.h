#pragma once

#include "model.h"

extern AppState app_state;

void initImu();
void calibrateImu();
void applyCompassCalibration();
void update_attitude();
void run_attitude_task(void* args);
