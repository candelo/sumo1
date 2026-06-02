#include "task_feedback.h"
#include "robot_state.h"
#include "feedback.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

void vTaskFeedback(void *arg)
{
    (void)arg;
    feedback_cmd_t cmd;

    while (1) {
        // Block until a command arrives (no busy-wait)
        if (xQueueReceive(g_feedback_queue, &cmd, portMAX_DELAY) == pdTRUE) {
            if (cmd.led1 >= 0) feedback_led(0, cmd.led1);
            if (cmd.led2 >= 0) feedback_led(1, cmd.led2);
            if (cmd.freq_hz > 0 && cmd.duration_ms > 0) {
                feedback_tone(cmd.freq_hz, cmd.duration_ms);
            } else {
                feedback_silent();
            }
        }
    }
}
