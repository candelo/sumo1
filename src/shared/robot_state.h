#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// ─── OPERATION MODES ──────────────────────────────────────────────────────────
typedef enum {
    MODE_IDLE              = 0,   // inicio: motores apagados, espera botón
    MODE_COMBAT            = 1,   // 1 press – pelea normal
    MODE_TEST_DISTANCE     = 2,   // 2 presses – sensores de distancia por serial
    MODE_TEST_LINE         = 3,   // 3 presses – sensores de piso por serial
    MODE_TEST_MOTORS       = 4,   // 4 presses – secuencia de prueba de motores
    MODE_CALIBRATE_SENSORS = 5,   // 5 presses – calibración interactiva
    MODE_TOGGLE_DOHYO      = 6,   // 6 presses – cambiar tipo de dohyo y volver a IDLE
    MODE_TEST_OPPONENT     = 7,   // 7 presses – detectar oponente con LEDs y sonido
} robot_mode_t;

// ─── DOHYO TYPE ───────────────────────────────────────────────────────────────
typedef enum {
    DOHYO_BLACK_RING = 0,   // estándar: superficie negra, borde blanco
    DOHYO_WHITE_RING = 1,   // invertido: superficie blanca, borde negro
} dohyo_type_t;

extern volatile dohyo_type_t g_dohyo_type;

// ─── COMBAT STATE MACHINE ─────────────────────────────────────────────────────
typedef enum {
    STATE_IDLE    = 0,
    STATE_SEARCH,
    STATE_ATTACK,
    STATE_EVADE,
} combat_state_t;

// ─── SENSOR DATA (written by vTaskSensors, read by vTaskBrain) ────────────────
typedef struct {
    int   dist_raw[3];         // Raw ADC values
    int   dist_filtered[3];    // Moving-average filtered values
    float dist_cm[3];          // Converted distance in cm
    bool  line_detected[2];    // true = white line (edge of ring) detected
    float battery_voltage;
    int64_t timestamp_us;      // esp_timer_get_time() at last update
} sensor_data_t;

// ─── MOTOR COMMAND (sent via queue from vTaskBrain to vTaskMotors) ────────────
typedef struct {
    float left;   // -100.0 .. +100.0
    float right;  // -100.0 .. +100.0
    bool  brake;  // if true, apply short brake regardless of left/right
} motor_cmd_t;

// ─── FEEDBACK COMMAND (any task -> vTaskFeedback) ─────────────────────────────
typedef struct {
    uint32_t freq_hz;       // 0 = speaker off
    uint32_t duration_ms;
    int8_t   led1;          // -1 = no change, 0 = off, 1 = on
    int8_t   led2;
} feedback_cmd_t;

// ─── GLOBAL HANDLES (defined in main.c) ──────────────────────────────────────
extern QueueHandle_t    g_motor_queue;
extern QueueHandle_t    g_feedback_queue;
extern SemaphoreHandle_t g_sensor_mutex;
extern sensor_data_t    g_sensor_data;
extern volatile robot_mode_t    g_robot_mode;
extern volatile combat_state_t  g_combat_state;

// ─── CONVENIENCE BEEP PRESETS ─────────────────────────────────────────────────
#define BEEP_SHORT_HZ   1500
#define BEEP_SHORT_MS   60
#define BEEP_CONFIRM_HZ 2000
#define BEEP_CONFIRM_MS 120
#define BEEP_ERROR_HZ   400
#define BEEP_ERROR_MS   300

static inline void feedback_beep(uint32_t hz, uint32_t ms)
{
    extern QueueHandle_t g_feedback_queue;
    feedback_cmd_t cmd = { .freq_hz = hz, .duration_ms = ms, .led1 = -1, .led2 = -1 };
    xQueueSend(g_feedback_queue, &cmd, 0);
}
