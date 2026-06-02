#include "feedback.h"
#include "hardware_map.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char    *TAG       = "Feedback";
static const gpio_num_t LED_PINS[2] = { LED1_GPIO, LED2_GPIO };

void feedback_init(void)
{
    // LEDs
    for (int i = 0; i < 2; i++) {
        gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << LED_PINS[i],
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&cfg);
        gpio_set_level(LED_PINS[i], 0);
    }

    // LEDC timer for speaker PWM
    ledc_timer_config_t timer = {
        .speed_mode      = SPEAKER_LEDC_MODE,
        .timer_num       = SPEAKER_LEDC_TIMER,
        .duty_resolution = SPEAKER_DUTY_RES,
        .freq_hz         = 1000,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    // LEDC channel – start silent (duty=0)
    ledc_channel_config_t ch = {
        .speed_mode = SPEAKER_LEDC_MODE,
        .channel    = SPEAKER_LEDC_CHANNEL,
        .timer_sel  = SPEAKER_LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = SPEAKER_GPIO,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    ESP_LOGI(TAG, "Feedback init OK");
}

void feedback_tone(uint32_t freq_hz, uint32_t duration_ms)
{
    if (freq_hz == 0) {
        feedback_silent();
        return;
    }
    ledc_set_freq(SPEAKER_LEDC_MODE, SPEAKER_LEDC_TIMER, freq_hz);
    ledc_set_duty(SPEAKER_LEDC_MODE, SPEAKER_LEDC_CHANNEL, SPEAKER_DUTY_50PCT);
    ledc_update_duty(SPEAKER_LEDC_MODE, SPEAKER_LEDC_CHANNEL);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    feedback_silent();
}

void feedback_silent(void)
{
    ledc_set_duty(SPEAKER_LEDC_MODE, SPEAKER_LEDC_CHANNEL, 0);
    ledc_update_duty(SPEAKER_LEDC_MODE, SPEAKER_LEDC_CHANNEL);
}

void feedback_led(int led_idx, int state)
{
    if (led_idx < 0 || led_idx > 1) return;
    gpio_set_level(LED_PINS[led_idx], state ? 1 : 0);
}
