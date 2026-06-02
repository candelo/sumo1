#pragma once
#include <stdbool.h>

// Initialize GPIO inputs for floor/edge sensors.
// GPIO 34 and 35 are input-only with no internal pullup.
void line_sensor_init(void);

// Returns true if white line (ring edge) is detected on sensor 0 or 1.
// Logic depends on your sensor: adjust LINE_ACTIVE_LEVEL in line_sensor.c.
bool line_sensor_read(int idx);
