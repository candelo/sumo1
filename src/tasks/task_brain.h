#pragma once

// Brain task: combat state machine + test/calibration modes.
// Reads g_sensor_data, sends to g_motor_queue and g_feedback_queue.
void vTaskBrain(void *arg);
