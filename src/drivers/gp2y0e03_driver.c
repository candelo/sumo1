#include "gp2y0e03_driver.h"
#include "hal/hardware_map.h"

#include "driver/adc.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static const char *TAG    = "GP2Y0E03";
static const char *NVS_NS = "gp2y_cal";

// Mutex: las funciones legacy adc1_get_raw / adc2_get_raw no son thread-safe
static SemaphoreHandle_t s_adc_mutex = NULL;

static gp2y_sensor_t s_sensors[DIST_SENSOR_COUNT];

// Distancias de referencia para la calibración interactiva (cm)
static const float CAL_DISTANCES_CM[GP2Y_CAL_POINTS] = {5.0f, 8.0f, 12.0f, 18.0f, 25.0f, 35.0f};

// ─── TABLA DE CANALES ─────────────────────────────────────────────────────────
// S0 frente   → GPIO33 → ADC1_CH5
// S1 izq 45°  → GPIO4  → ADC2_CH0  (unidad ADC2)
// S2 der 45°  → GPIO32 → ADC1_CH4

typedef enum { UNIT_ADC1, UNIT_ADC2 } adc_unit_sel_t;

static const adc_unit_sel_t SENSOR_UNIT[DIST_SENSOR_COUNT] = {
    UNIT_ADC1,   // S0 – GPIO33
    UNIT_ADC2,   // S1 – GPIO4
    UNIT_ADC1,   // S2 – GPIO32
};

static const adc1_channel_t ADC1_CH[DIST_SENSOR_COUNT] = {
    ADC1_CHANNEL_5,   // GPIO33
    (adc1_channel_t)0, // no aplica para S1
    ADC1_CHANNEL_4,   // GPIO32
};

// ─── ADC INIT ─────────────────────────────────────────────────────────────────

static void _adc_init(void)
{
    s_adc_mutex = xSemaphoreCreateMutex();
    configASSERT(s_adc_mutex);

    // ADC1: S0 (GPIO33) y S2 (GPIO32)
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11)); // GPIO33
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11)); // GPIO32

    // ADC2: S1 (GPIO4)
    ESP_ERROR_CHECK(adc2_config_channel_atten(ADC2_CHANNEL_0, ADC_ATTEN_DB_11)); // GPIO4

    ESP_LOGI(TAG, "ADC init OK – S0=GPIO33(frente) S1=GPIO4(izq45) S2=GPIO32(der45)");
}

// ─── LECTURA CON OVERSAMPLING ─────────────────────────────────────────────────

#define ADC_OVERSAMPLE 4

static int _read_channel(int idx)
{
    int32_t sum = 0;
    if (SENSOR_UNIT[idx] == UNIT_ADC1) {
        for (int i = 0; i < ADC_OVERSAMPLE; i++) {
            sum += adc1_get_raw(ADC1_CH[idx]);
        }
    } else {
        int raw = 0;
        for (int i = 0; i < ADC_OVERSAMPLE; i++) {
            esp_err_t err = adc2_get_raw(ADC2_CHANNEL_0, ADC_WIDTH_BIT_12, &raw);
            if (err != ESP_OK) return -1;
            sum += raw;
        }
    }
    return (int)(sum / ADC_OVERSAMPLE);
}

// ─── MOVING AVERAGE ───────────────────────────────────────────────────────────

static int _ma_update(gp2y_sensor_t *s, int new_val)
{
    s->ma_sum -= s->ma_buf[s->ma_idx];
    s->ma_buf[s->ma_idx] = new_val;
    s->ma_sum += new_val;
    s->ma_idx  = (s->ma_idx + 1) % GP2Y_MA_WINDOW;
    return s->ma_sum / GP2Y_MA_WINDOW;
}

// ─── CONVERSIÓN A CM ──────────────────────────────────────────────────────────

static float _raw_to_cm(const gp2y_cal_t *cal, int raw)
{
    if (!cal->valid) return -1.0f;
    for (int i = 0; i < GP2Y_CAL_POINTS - 1; i++) {
        int r_hi = cal->adc_raw[i];
        int r_lo = cal->adc_raw[i + 1];
        if (raw <= r_hi && raw >= r_lo) {
            float t = (float)(r_hi - raw) / (float)(r_hi - r_lo);
            return cal->dist_cm[i] + t * (cal->dist_cm[i + 1] - cal->dist_cm[i]);
        }
    }
    if (raw > cal->adc_raw[0])             return cal->dist_cm[0];   // más cerca que 5 cm
    return -1.0f;   // debajo del ADC mínimo calibrado → sin objeto en rango
}

// ─── NVS ──────────────────────────────────────────────────────────────────────

bool gp2y_load_calibration(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    bool all_ok = true;
    for (int s = 0; s < DIST_SENSOR_COUNT; s++) {
        char key[8];
        size_t len = sizeof(gp2y_cal_t);
        snprintf(key, sizeof(key), "cal%d", s);
        esp_err_t ret = nvs_get_blob(h, key, &s_sensors[s].cal, &len);
        if (ret != ESP_OK || len != sizeof(gp2y_cal_t) || !s_sensors[s].cal.valid) {
            s_sensors[s].cal.valid = false;
            all_ok = false;
        }
    }
    nvs_close(h);
    ESP_LOGI(TAG, "Calibracion: %s", all_ok ? "OK" : "sin datos (usar modo 5)");
    return all_ok;
}

void gp2y_save_calibration(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed"); return;
    }
    for (int s = 0; s < DIST_SENSOR_COUNT; s++) {
        char key[8];
        snprintf(key, sizeof(key), "cal%d", s);
        nvs_set_blob(h, key, &s_sensors[s].cal, sizeof(gp2y_cal_t));
    }
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "Calibracion guardada en NVS");
}

// ─── API PÚBLICA ──────────────────────────────────────────────────────────────

void gp2y_init(void)
{
    memset(s_sensors, 0, sizeof(s_sensors));
    _adc_init();
    gp2y_load_calibration();
}

int gp2y_read_instant(int idx)
{
    if (idx < 0 || idx >= DIST_SENSOR_COUNT) return -1;
    xSemaphoreTake(s_adc_mutex, portMAX_DELAY);
    int val = _read_channel(idx);
    xSemaphoreGive(s_adc_mutex);
    return val;
}

int gp2y_read_raw(int idx)
{
    if (idx < 0 || idx >= DIST_SENSOR_COUNT) return 0;
    xSemaphoreTake(s_adc_mutex, portMAX_DELAY);
    int val = _read_channel(idx);
    xSemaphoreGive(s_adc_mutex);
    if (val < 0) return 0;
    return _ma_update(&s_sensors[idx], val);
}

float gp2y_raw_to_cm(int idx, int raw)
{
    if (idx < 0 || idx >= DIST_SENSOR_COUNT) return -1.0f;
    return _raw_to_cm(&s_sensors[idx].cal, raw);
}

// ─── CALIBRACIÓN INTERACTIVA ──────────────────────────────────────────────────

void gp2y_calibrate_interactive(void)
{
    static const int GPIOS[DIST_SENSOR_COUNT] = {33, 4, 32};
    static const char *NAMES[DIST_SENSOR_COUNT] = {"FRENTE", "IZQ 45°", "DER 45°"};

    printf("\n=== CALIBRACION GP2Y0E03 ===\n");
    printf("Distancias: ");
    for (int p = 0; p < GP2Y_CAL_POINTS; p++) printf("%.0fcm ", CAL_DISTANCES_CM[p]);
    printf("\n\n");

    for (int s = 0; s < DIST_SENSOR_COUNT; s++) {
        gp2y_cal_t *cal = &s_sensors[s].cal;
        printf("--- Sensor %d  GPIO%d  %s ---\n", s, GPIOS[s], NAMES[s]);

        for (int p = 0; p < GP2Y_CAL_POINTS; p++) {
            printf("Pon objeto a %.0f cm y presiona ENTER...\n", CAL_DISTANCES_CM[p]);
            // Esperar ENTER con yield para no bloquear el watchdog
            int c = EOF;
            while (c != '\n' && c != '\r') {
                c = getchar();
                vTaskDelay(pdMS_TO_TICKS(20));
            }

            // Tomar 32 muestras – mutex suelto entre muestras para no privar a vTaskSensors
            int32_t sum = 0;
            for (int i = 0; i < 32; i++) {
                xSemaphoreTake(s_adc_mutex, portMAX_DELAY);
                int v = _read_channel(s);
                xSemaphoreGive(s_adc_mutex);
                sum += (v < 0) ? 0 : v;
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            cal->dist_cm[p] = CAL_DISTANCES_CM[p];
            cal->adc_raw[p] = (int)(sum / 32);
            printf("  Guardado: %.0fcm -> ADC %d\n", CAL_DISTANCES_CM[p], cal->adc_raw[p]);
        }
        cal->valid = true;
        printf("Sensor %d listo.\n\n", s);
    }
    gp2y_save_calibration();
    printf("=== CALIBRACION GUARDADA ===\n\n");
}
