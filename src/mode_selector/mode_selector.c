#include "mode_selector.h"
#include "hardware_map.h"
#include "robot_state.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>

static const char *TAG = "ModeBtn";

// Timing constants (ms)
#define DEBOUNCE_MS         30    // Ignore transitions faster than this
#define INTER_PRESS_MS      400   // Max gap between presses in a sequence
#define LONG_PRESS_MS       1000  // Hold > this = long press (reserved for reset)

// Mode names for logging
static const char *MODE_NAMES[] = {
    [MODE_IDLE]              = "IDLE (esperando modo)",
    [MODE_COMBAT]            = "COMBAT",
    [MODE_TEST_DISTANCE]     = "TEST DISTANCE SENSORS",
    [MODE_TEST_LINE]         = "TEST LINE SENSORS",
    [MODE_TEST_MOTORS]       = "TEST MOTORS",
    [MODE_CALIBRATE_SENSORS] = "CALIBRATE SENSORS",
    [MODE_TOGGLE_DOHYO]      = "TOGGLE DOHYO TYPE",
    [MODE_TEST_OPPONENT]     = "TEST OPPONENT SENSORS",
};

static void _beep_press(void)
{
    // 2800 Hz, 45ms: agudo y corto, se escucha claramente sin molestar
    feedback_cmd_t cmd = {
        .freq_hz     = 2800,
        .duration_ms = 45,
        .led1 = -1, .led2 = -1,
    };
    // pdMS_TO_TICKS(5): pequeño timeout para no perder el beep si la cola está llena
    xQueueSend(g_feedback_queue, &cmd, pdMS_TO_TICKS(5));
}

static void _beep_confirm(robot_mode_t mode)
{
    // Play N beeps matching the mode number
    for (int i = 0; i < (int)mode; i++) {
        feedback_cmd_t cmd = {
            .freq_hz     = BEEP_CONFIRM_HZ,
            .duration_ms = BEEP_CONFIRM_MS,
            .led1 = -1, .led2 = -1,
        };
        xQueueSend(g_feedback_queue, &cmd, 0);
        vTaskDelay(pdMS_TO_TICKS(BEEP_CONFIRM_MS + 60));
    }
}

static void _mode_selector_task(void *arg)
{
    (void)arg;

    // State machine variables
    int  press_count     = 0;
    bool btn_was_pressed = false;
    TickType_t last_press_tick = 0;

    while (1) {
        bool btn_pressed = (gpio_get_level(BTN_MODE_GPIO) == 0); // active LOW

        if (btn_pressed && !btn_was_pressed) {
            // Falling edge: button just pressed
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS)); // debounce
            if (gpio_get_level(BTN_MODE_GPIO) == 0) {
                press_count++;
                last_press_tick = xTaskGetTickCount();
                _beep_press();
                ESP_LOGD(TAG, "Press #%d", press_count);
            }
        }
        btn_was_pressed = btn_pressed;

        // Commit sequence after INTER_PRESS_MS of silence
        if (press_count > 0) {
            TickType_t elapsed = (xTaskGetTickCount() - last_press_tick) * portTICK_PERIOD_MS;
            if (elapsed >= INTER_PRESS_MS && !btn_pressed) {
                // Clamp to valid mode range
                if (press_count > MODE_TEST_OPPONENT) {
                    press_count = MODE_TEST_OPPONENT;
                }
                robot_mode_t new_mode = (robot_mode_t)press_count;
                g_robot_mode = new_mode;

                const char *name = (new_mode >= 1 && new_mode <= MODE_TEST_OPPONENT)
                                   ? MODE_NAMES[new_mode] : "UNKNOWN";
                ESP_LOGI(TAG, "Mode selected: %d – %s", (int)new_mode, name);
                printf("\n>> MODE: %s <<\n\n", name);

                _beep_confirm(new_mode);
                press_count = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void mode_selector_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << BTN_MODE_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,   // GPIO5 supports internal pullup
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    xTaskCreate(_mode_selector_task, "mode_btn", 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "Mode selector ready (GPIO %d)", BTN_MODE_GPIO);
}

robot_mode_t mode_selector_get(void)
{
    return g_robot_mode;
}
