#include "task_brain.h"
#include "robot_state.h"
#include "tb6612fng_driver.h"
#include "gp2y0e03_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "Brain";

// ─── TUNING PARAMETERS ────────────────────────────────────────────────────────
#define ENEMY_DETECT_CM         30.0f   // Attack if opponent closer than this
#define ENEMY_LOST_CM           35.0f   // Return to search if farther than this
#define SPEED_ATTACK            100.0f  // Full speed toward opponent
#define SPEED_SEARCH             40.0f  // Rotation speed while searching
#define EVADE_BACKUP_MS         300     // ms to back up when line detected
#define EVADE_TURN_MS           200     // ms to turn after backing up

// ─── HELPERS ──────────────────────────────────────────────────────────────────

static void _send_motors(float left, float right)
{
    motor_cmd_t cmd = { .left = left, .right = right, .brake = false };
    xQueueOverwrite(g_motor_queue, &cmd);
}

static void _send_brake(void)
{
    motor_cmd_t cmd = { .brake = true };
    xQueueOverwrite(g_motor_queue, &cmd);
}

static sensor_data_t _read_sensors(void)
{
    sensor_data_t snap;
    xSemaphoreTake(g_sensor_mutex, portMAX_DELAY);
    snap = g_sensor_data;
    xSemaphoreGive(g_sensor_mutex);
    return snap;
}

// Returns index of sensor detecting nearest object, or -1 if none within range.
static int _nearest_enemy(const sensor_data_t *s, float *out_cm)
{
    int   best_idx = -1;
    float best_cm  = ENEMY_DETECT_CM;
    for (int i = 0; i < DIST_SENSOR_COUNT; i++) {
        if (s->dist_cm[i] > 0.0f && s->dist_cm[i] < best_cm) {
            best_cm  = s->dist_cm[i];
            best_idx = i;
        }
    }
    if (out_cm) *out_cm = best_cm;
    return best_idx;
}

// ─── COMBAT STATE MACHINE ─────────────────────────────────────────────────────

static void _run_combat(void)
{
    sensor_data_t s = _read_sensors();

    switch (g_combat_state) {
    case STATE_IDLE:
        _send_brake();
        break;

    case STATE_SEARCH:
        // Rotate in place until enemy spotted
        _send_motors(-SPEED_SEARCH, SPEED_SEARCH);
        {
            float cm;
            if (_nearest_enemy(&s, &cm) >= 0) {
                ESP_LOGI(TAG, "Enemy spotted at %.1f cm → ATTACK", cm);
                g_combat_state = STATE_ATTACK;
                feedback_beep(2500, 80);
            }
        }
        // Check for ring edge while searching
        if (s.line_detected[0] || s.line_detected[1]) {
            g_combat_state = STATE_EVADE;
        }
        break;

    case STATE_ATTACK: {
        float cm;
        int   idx = _nearest_enemy(&s, &cm);

        // Ring edge: evade first
        if (s.line_detected[0] || s.line_detected[1]) {
            ESP_LOGI(TAG, "Line detected during attack → EVADE");
            g_combat_state = STATE_EVADE;
            break;
        }

        if (idx < 0) {
            // Lost opponent
            ESP_LOGI(TAG, "Enemy lost → SEARCH");
            g_combat_state = STATE_SEARCH;
            break;
        }

        // Layout físico: idx=0 FRENTE, idx=1 IZQ 45°, idx=2 DER 45°
        float l = SPEED_ATTACK, r = SPEED_ATTACK;
        if (idx == 1) { l = SPEED_ATTACK * 0.6f; }      // enemigo izq: frenar motor izq → gira izq
        else if (idx == 2) { r = SPEED_ATTACK * 0.6f; } // enemigo der: frenar motor der → gira der
        // idx == 0: frente, velocidad máxima
        _send_motors(l, r);
        break;
    }

    case STATE_EVADE: {
        // Refresh motor command every 10ms to stay ahead of vTaskMotors 20ms watchdog.
        // A single send + long vTaskDelay causes brake-oscillation every 20ms.
        int i;
        for (i = 0; i < EVADE_BACKUP_MS / 10; i++) {
            _send_motors(-80.0f, -80.0f);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        for (i = 0; i < EVADE_TURN_MS / 10; i++) {
            _send_motors(-70.0f, 70.0f);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        g_combat_state = STATE_SEARCH;
        break;
    }
    }
}

// ─── TEST MODES ───────────────────────────────────────────────────────────────

static void _test_distance_sensors(void)
{
    // Lee UNA muestra instantánea por sensor (sin filtro) para ver si el ADC
    // sigue la tensión real. Luego muestra también los valores filtrados de
    // g_sensor_data que llena vTaskSensors (único dueño del filtro MA).
    int inst[DIST_SENSOR_COUNT];
    for (int i = 0; i < DIST_SENSOR_COUNT; i++) {
        inst[i] = gp2y_read_instant(i);
    }

    sensor_data_t s = _read_sensors();

    printf("[DIST] GPIO33(frente) GPIO4(izq45) GPIO32(der45)\n");
    printf("  inst  %-7d %-7d %-7d   (ADC raw, sin filtro)\n",
           inst[0], inst[1], inst[2]);
    printf("  filt  %-7d %-7d %-7d   (media movil)\n",
           s.dist_raw[0], s.dist_raw[1], s.dist_raw[2]);
    printf("  cm   %-7.1f %-7.1f %-7.1f   (-1=sin calibrar/fuera de rango)\n\n",
           s.dist_cm[0], s.dist_cm[1], s.dist_cm[2]);

    vTaskDelay(pdMS_TO_TICKS(500));
}

static void _test_line_sensors(void)
{
    sensor_data_t s = _read_sensors();
    printf("[LINE] S0=%s  S1=%s\n",
           s.line_detected[0] ? "WHITE" : "black",
           s.line_detected[1] ? "WHITE" : "black");
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void _test_motors(void)
{
    static int phase = 0;
    static TickType_t phase_start = 0;
    TickType_t now = xTaskGetTickCount();

    if ((now - phase_start) * portTICK_PERIOD_MS < 2000) return;
    phase_start = now;

    // Prueba cada motor por separado para identificar polaridad y dirección física
    switch (phase % 8) {
    case 0: printf("[MOTORS] FRENO\n");               _send_brake();            break;
    case 1: printf("[MOTORS] IZQ adelante (+60)\n");  _send_motors( 60,   0);  break;
    case 2: printf("[MOTORS] FRENO\n");               _send_brake();            break;
    case 3: printf("[MOTORS] IZQ atras  (-60)\n");    _send_motors(-60,   0);  break;
    case 4: printf("[MOTORS] FRENO\n");               _send_brake();            break;
    case 5: printf("[MOTORS] DER adelante (+60)\n");  _send_motors(  0,  60);  break;
    case 6: printf("[MOTORS] FRENO\n");               _send_brake();            break;
    case 7: printf("[MOTORS] DER atras  (-60)\n");    _send_motors(  0, -60);  break;
    }
    phase++;
}

// ─── TEST OPPONENT ────────────────────────────────────────────────────────────

// Detection range for test mode – wider than combat to verify sensor coverage.
#define TEST_DETECT_CM  40.0f

static void _test_opponent(void)
{
    sensor_data_t s = _read_sensors();

    // Find nearest object within TEST_DETECT_CM on any sensor
    int   best_idx = -1;
    float best_cm  = TEST_DETECT_CM;
    for (int i = 0; i < DIST_SENSOR_COUNT; i++) {
        if (s.dist_cm[i] > 0.0f && s.dist_cm[i] < best_cm) {
            best_cm  = s.dist_cm[i];
            best_idx = i;
        }
    }

    if (best_idx < 0) {
        // Nothing in range – LEDs off, no sound
        feedback_cmd_t off = { .freq_hz = 0, .duration_ms = 0, .led1 = 0, .led2 = 0 };
        xQueueSend(g_feedback_queue, &off, 0);
        vTaskDelay(pdMS_TO_TICKS(120));
        return;
    }

    // Each position gets distinct pitch + LED pattern:
    //   IZQ  : 700 Hz  – LED1 solo  (lado izquierdo)
    //   FRENTE: 1500 Hz – ambos LEDs (centro)
    //   DER  : 2800 Hz  – LED2 solo  (lado derecho)
    uint32_t    hz;
    int8_t      l1, l2;
    const char *pos;
    switch (best_idx) {
    case 1:  hz =  700; l1 = 1; l2 = 0; pos = "IZQ   "; break;
    case 0:  hz = 1500; l1 = 1; l2 = 1; pos = "FRENTE"; break;
    case 2:  hz = 2800; l1 = 0; l2 = 1; pos = "DER   "; break;
    default: return;
    }

    printf("[OPNT] %s  %.1f cm\n", pos, best_cm);

    // Blink on: LEDs + beep
    feedback_cmd_t on = { .freq_hz = hz, .duration_ms = 80, .led1 = l1, .led2 = l2 };
    xQueueSend(g_feedback_queue, &on, 0);
    vTaskDelay(pdMS_TO_TICKS(110));

    // Blink off: LEDs apagados
    feedback_cmd_t off = { .freq_hz = 0, .duration_ms = 0, .led1 = 0, .led2 = 0 };
    xQueueSend(g_feedback_queue, &off, 0);
    vTaskDelay(pdMS_TO_TICKS(90));
}

// ─── MAIN TASK ────────────────────────────────────────────────────────────────

void vTaskBrain(void *arg)
{
    (void)arg;
    // Inicializar last_mode igual al modo de arranque para que NO se detecte
    // un "cambio de modo" en el primer tick y no se habiliten motores.
    robot_mode_t last_mode = MODE_IDLE;

    while (1) {
        robot_mode_t mode = g_robot_mode;

        // Cambio de modo: frenar, habilitar/deshabilitar motores según corresponda
        if (mode != last_mode) {
            _send_brake();
            motors_enable(mode == MODE_COMBAT || mode == MODE_TEST_MOTORS);
            g_combat_state = STATE_IDLE;
            last_mode = mode;
            ESP_LOGI(TAG, "Mode changed to %d", (int)mode);
        }

        switch (mode) {
        case MODE_IDLE:
            // Esperar con motores apagados hasta que el usuario seleccione un modo
            motors_enable(false);
            _send_brake();
            vTaskDelay(pdMS_TO_TICKS(100));
            break;

        case MODE_COMBAT:
            if (g_combat_state == STATE_IDLE) {
                g_combat_state = STATE_SEARCH;
            }
            _run_combat();
            vTaskDelay(pdMS_TO_TICKS(10));
            break;

        case MODE_TEST_DISTANCE:
            _test_distance_sensors();
            break;

        case MODE_TEST_LINE:
            _test_line_sensors();
            break;

        case MODE_TEST_MOTORS:
            _test_motors();
            vTaskDelay(pdMS_TO_TICKS(50));
            break;

        case MODE_TEST_OPPONENT:
            _test_opponent();
            break;

        case MODE_CALIBRATE_SENSORS:
            // Run calibration once, then go idle waiting for next mode change
            gp2y_calibrate_interactive();
            // Prevent re-running until mode changes
            while (g_robot_mode == MODE_CALIBRATE_SENSORS) {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            break;

        case MODE_TOGGLE_DOHYO: {
            g_dohyo_type = (g_dohyo_type == DOHYO_BLACK_RING)
                           ? DOHYO_WHITE_RING : DOHYO_BLACK_RING;

            // Persist to NVS
            nvs_handle_t h;
            if (nvs_open("sumo_cfg", NVS_READWRITE, &h) == ESP_OK) {
                nvs_set_u8(h, "dohyo", (uint8_t)g_dohyo_type);
                nvs_commit(h);
                nvs_close(h);
            }

            const bool is_black = (g_dohyo_type == DOHYO_BLACK_RING);
            ESP_LOGI(TAG, "Dohyo: %s", is_black ? "NEGRO/borde blanco" : "BLANCO/borde negro");
            printf("\n>> DOHYO: %s <<\n\n", is_black ? "NEGRO con borde BLANCO" : "BLANCO con borde NEGRO");

            // Two beeps: low pitch = black ring (standard), high pitch = white ring
            uint32_t hz = is_black ? 600 : 2400;
            feedback_beep(hz, 130);
            vTaskDelay(pdMS_TO_TICKS(200));
            feedback_beep(hz, 130);
            vTaskDelay(pdMS_TO_TICKS(200));

            g_robot_mode = MODE_IDLE;
            break;
        }

        default:
            vTaskDelay(pdMS_TO_TICKS(50));
            break;
        }
    }
}
