#pragma once

// Sensor task: reads all ADC + line sensors every 10ms.
// Updates g_sensor_data (protected by g_sensor_mutex).
void vTaskSensors(void *arg);
