#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "hardware_map.h"

// Number of samples in the moving-average filter
// 4 samples × 10ms period = 40ms lag – fast enough to catch a target during rotation
#define GP2Y_MA_WINDOW  4

// Calibration: 6 reference points (distance_cm, adc_raw)
#define GP2Y_CAL_POINTS 6

typedef struct {
    float    dist_cm[GP2Y_CAL_POINTS];
    int      adc_raw[GP2Y_CAL_POINTS];
    bool     valid;
} gp2y_cal_t;

typedef struct {
    int ma_buf[GP2Y_MA_WINDOW];
    int ma_sum;
    int ma_idx;
    gp2y_cal_t cal;
} gp2y_sensor_t;

// Initialize all 3 GP2Y0E03 sensors. Creates ADC1 handle internally.
void gp2y_init(void);

// Lee UNA muestra instantánea del ADC sin tocar el filtro MA. Solo para debug.
int  gp2y_read_instant(int idx);

// Lee el ADC y actualiza el filtro de media móvil. Llamar solo desde vTaskSensors.
int  gp2y_read_raw(int idx);

// Convierte un valor raw (ya leído) a cm usando la tabla de calibración.
// No hace ninguna lectura de hardware.
float gp2y_raw_to_cm(int idx, int raw);

// ─── CALIBRATION ──────────────────────────────────────────────────────────────
void gp2y_calibrate_interactive(void);
bool gp2y_load_calibration(void);
void gp2y_save_calibration(void);
