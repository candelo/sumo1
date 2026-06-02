#pragma once

#include <stdbool.h>

typedef enum {
    MOTOR_LEFT  = 0,
    MOTOR_RIGHT = 1,
} motor_side_t;

// Initialize MCPWM (20kHz) and direction GPIOs. Must be called once at startup.
void motors_init(void);

// Set speed for one motor. duty: -100.0 (full reverse) .. +100.0 (full forward).
// Negative values invert direction pins automatically.
void motors_set_speed(motor_side_t side, float duty);

// Convenience: set both motors in one call.
void motors_drive(float left, float right);

// Short brake: IN1=HIGH, IN2=HIGH on both sides (TB6612FNG dynamic brake).
void motors_brake(void);

// Coast (IN1=LOW, IN2=LOW): motors spin freely.
void motors_coast(void);

// Enable / disable STBY pin.
void motors_enable(bool en);
