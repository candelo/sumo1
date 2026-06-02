#pragma once

#include "driver/gpio.h"
#include "driver/adc.h"

// ─── MOTOR DRIVER TB6612FNG ───────────────────────────────────────────────────
#define MOTOR_PWM_L         GPIO_NUM_26
#define MOTOR_PWM_R         GPIO_NUM_25
#define MOTOR_ENABLE        GPIO_NUM_23  // STBY – HIGH to enable motors

#define MOTOR_LIN1          GPIO_NUM_22
#define MOTOR_LIN2          GPIO_NUM_21
#define MOTOR_RIN1          GPIO_NUM_18
#define MOTOR_RIN2          GPIO_NUM_19

// ─── FLOOR SENSORS (input-only, sin pullup interno) ──────────────────────────
#define LINE_SENSOR_1       GPIO_NUM_34
#define LINE_SENSOR_2       GPIO_NUM_35

// ─── DISTANCE SENSORS GP2Y0E03 ───────────────────────────────────────────────
// Posición física:  S0=frente  S1=izquierda 45°  S2=derecha 45°
//
// S0 FRENTE       – GPIO33 – ADC1_CHANNEL_5
// S1 IZQ 45°      – GPIO4  – ADC2_CHANNEL_0  (¡ADC2, unidad distinta!)
// S2 DER 45°      – GPIO32 – ADC1_CHANNEL_4
//
// NOTA: GPIO4 (S1) era el pin de batería. El monitoreo de batería queda
//       deshabilitado hasta reasignar ese pin.
#define DIST_SENSOR_COUNT   3

#define DIST0_GPIO          GPIO_NUM_33   // frente
#define DIST1_GPIO          GPIO_NUM_4    // izquierda 45°
#define DIST2_GPIO          GPIO_NUM_32   // derecha 45°

// ─── BATTERY MONITOR ─────────────────────────────────────────────────────────
// GPIO4 ahora lo usa el sensor de distancia izquierdo.
// Reasignar a otro pin disponible antes de habilitar el monitoreo de batería.
// #define BATT_GPIO        GPIO_NUM_XX
#define BATT_DIVIDER_RATIO  2.0f

// ─── FEEDBACK ─────────────────────────────────────────────────────────────────
#define LED1_GPIO           GPIO_NUM_13
#define LED2_GPIO           GPIO_NUM_14
#define SPEAKER_GPIO        GPIO_NUM_27

// ─── MODE BUTTON (internal pullup, active LOW) ────────────────────────────────
#define BTN_MODE_GPIO       GPIO_NUM_5

// ─── MOTOR ORIENTATION ───────────────────────────────────────────────────────
// Definir si un motor está físicamente montado al revés (invierte la dirección).
#define MOTOR_LEFT_INVERTED   1
#define MOTOR_RIGHT_INVERTED  0

// ─── MCPWM CONFIG ─────────────────────────────────────────────────────────────
#define MOTOR_PWM_FREQ_HZ   20000
#define MOTOR_PWM_RES_HZ    10000000

// ─── LEDC SPEAKER CONFIG ──────────────────────────────────────────────────────
#define SPEAKER_LEDC_TIMER      LEDC_TIMER_0
#define SPEAKER_LEDC_CHANNEL    LEDC_CHANNEL_0
#define SPEAKER_LEDC_MODE       LEDC_LOW_SPEED_MODE
#define SPEAKER_DUTY_RES        LEDC_TIMER_10_BIT
#define SPEAKER_DUTY_50PCT      512
