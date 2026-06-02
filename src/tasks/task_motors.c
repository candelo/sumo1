#include "task_motors.h"
#include "robot_state.h"
#include "tb6612fng_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

void vTaskMotors(void *arg)
{
    (void)arg;
    motor_cmd_t cmd;

    while (1) {
        // 20ms timeout: if no command arrives, apply brake (safety)
        if (xQueueReceive(g_motor_queue, &cmd, pdMS_TO_TICKS(20)) == pdTRUE) {
            if (cmd.brake) {
                motors_brake();
            } else {
                motors_drive(cmd.left, cmd.right);
            }
        } else {
            // Watchdog: stop motors if brain task stops sending commands
            if (g_combat_state != STATE_IDLE) {
                motors_brake();
            }
        }
    }
}
