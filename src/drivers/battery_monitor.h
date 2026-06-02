#pragma once

// Initialize ADC2 for battery voltage monitoring.
void battery_monitor_init(void);

// Read battery voltage in volts. Applies BATT_DIVIDER_RATIO from hardware_map.h.
float battery_monitor_read_volts(void);
