#pragma once

// Motor task: consumes motor_cmd_t from g_motor_queue and applies to TB6612FNG.
void vTaskMotors(void *arg);
