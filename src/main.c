#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

#include "robot_state.h"
#include "hardware_map.h"

// Drivers
#include "tb6612fng_driver.h"
#include "gp2y0e03_driver.h"
#include "line_sensor.h"
#include "feedback.h"
#include "battery_monitor.h"
#include "mode_selector.h"

// Tasks
#include "task_sensors.h"
#include "task_brain.h"
#include "task_motors.h"
#include "task_feedback.h"

static const char *TAG = "Main";

// ─── GLOBAL STATE (declared extern in robot_state.h) ─────────────────────────
QueueHandle_t     g_motor_queue    = NULL;
QueueHandle_t     g_feedback_queue = NULL;
SemaphoreHandle_t g_sensor_mutex   = NULL;
sensor_data_t     g_sensor_data    = {0};
volatile robot_mode_t   g_robot_mode   = MODE_IDLE;
volatile combat_state_t g_combat_state = STATE_IDLE;
volatile dohyo_type_t   g_dohyo_type   = DOHYO_BLACK_RING;

// ─── STARTUP JINGLE ───────────────────────────────────────────────────────────
// Notes: Do-Mi-Sol-Mi-Do' (C5-E5-G5-E5-C6) – arpeggio ascendente + remate
// Sends each note to the feedback queue and waits for it to finish before
// sending the next, so notes never overlap.
static void _play_note(uint32_t freq_hz, uint32_t ms, int8_t led1, int8_t led2)
{
    feedback_cmd_t cmd = {
        .freq_hz = freq_hz, .duration_ms = ms,
        .led1 = led1, .led2 = led2,
    };
    xQueueSend(g_feedback_queue, &cmd, portMAX_DELAY);
    // Wait for the feedback task to finish playing before sending next note
    vTaskDelay(pdMS_TO_TICKS(ms + 25));
}

static void _startup_sequence(void)
{
    // C5 → E5 → G5 → E5 → C6  (Do Mayor, arpeggio ascendente con remate)
    _play_note(523,  90, 1, 0);   // Do5  – LED1 on
    _play_note(659,  90, 0, 1);   // Mi5  – LED2 on
    _play_note(784,  90, 1, 1);   // Sol5 – ambos LEDs
    _play_note(659,  70, 0, 0);   // Mi5  – LEDs off (tensión)
    _play_note(1047, 220, 1, 1);  // Do6  – remate: ambos LEDs
    // Apagar LEDs al final
    feedback_cmd_t off = { .freq_hz = 0, .duration_ms = 0, .led1 = 0, .led2 = 0 };
    xQueueSend(g_feedback_queue, &off, portMAX_DELAY);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Sumo1 – booting");

    // ── NVS (required for calibration storage) ──────────────────────────────
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Load saved dohyo type (default DOHYO_BLACK_RING if not set)
    {
        nvs_handle_t h;
        if (nvs_open("sumo_cfg", NVS_READONLY, &h) == ESP_OK) {
            uint8_t v = 0;
            if (nvs_get_u8(h, "dohyo", &v) == ESP_OK)
                g_dohyo_type = (dohyo_type_t)v;
            nvs_close(h);
        }
        ESP_LOGI(TAG, "Dohyo: %s",
                 g_dohyo_type == DOHYO_BLACK_RING
                 ? "NEGRO/borde blanco" : "BLANCO/borde negro");
    }

    // ── IPC primitives ──────────────────────────────────────────────────────
    g_motor_queue    = xQueueCreate(1, sizeof(motor_cmd_t));    // overwrite-style
    g_feedback_queue = xQueueCreate(8, sizeof(feedback_cmd_t));
    g_sensor_mutex   = xSemaphoreCreateMutex();

    configASSERT(g_motor_queue);
    configASSERT(g_feedback_queue);
    configASSERT(g_sensor_mutex);

    // ── Hardware init (order matters) ────────────────────────────────────────
    feedback_init();         // LEDs + LEDC speaker – first, so startup beep works
    motors_init();           // MCPWM
    gp2y_init();             // ADC1 + load NVS calibration
    line_sensor_init();      // GPIO 34/35
    battery_monitor_init();  // ADC2
    mode_selector_init();    // GPIO 5 + internal task

    ESP_LOGI(TAG, "Hardware init complete");

    // ── FreeRTOS tasks ───────────────────────────────────────────────────────
    // Priority: higher number = higher priority
    // Sensors  : priority 4 – must not miss 10ms window
    // Motors   : priority 3 – close to sensors for low latency
    // Brain    : priority 2 – decision logic
    // Feedback : priority 1 – non-critical
    xTaskCreate(vTaskSensors,  "sensors",  4096, NULL, 4, NULL);
    xTaskCreate(vTaskMotors,   "motors",   3072, NULL, 3, NULL);
    xTaskCreate(vTaskBrain,    "brain",    4096, NULL, 2, NULL);
    xTaskCreate(vTaskFeedback, "feedback", 2048, NULL, 1, NULL);

    // Startup beep (feedback task is now running)
    vTaskDelay(pdMS_TO_TICKS(100));
    _startup_sequence();

    // Dohyo type indicator: after jingle, hold the matching LED on for 2s
    // LED1 = dohyo negro/borde blanco  |  LED2 = dohyo blanco/borde negro
    vTaskDelay(pdMS_TO_TICKS(300));
    _play_note(0, 2000,
               g_dohyo_type == DOHYO_BLACK_RING ? 1 : 0,
               g_dohyo_type == DOHYO_WHITE_RING ? 1 : 0);
    feedback_cmd_t leds_off = { .freq_hz = 0, .duration_ms = 0, .led1 = 0, .led2 = 0 };
    xQueueSend(g_feedback_queue, &leds_off, portMAX_DELAY);

    ESP_LOGI(TAG, "All tasks started – press button to select mode");
    ESP_LOGI(TAG, "1=COMBAT  2=TEST_DIST  3=TEST_LINE  4=TEST_MOTORS  5=CALIBRATE  6=TOGGLE_DOHYO  7=TEST_OPPONENT");
}
