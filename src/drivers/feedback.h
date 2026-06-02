#pragma once

#include <stdint.h>

// Initialize LEDC for speaker and GPIO outputs for LEDs.
void feedback_init(void);

// Play a tone on the speaker. Blocks for duration_ms, then silences.
// Call from the feedback task (not from ISR).
void feedback_tone(uint32_t freq_hz, uint32_t duration_ms);

// Immediately silence speaker.
void feedback_silent(void);

// Set LED state (0=off, 1=on).
void feedback_led(int led_idx, int state);
