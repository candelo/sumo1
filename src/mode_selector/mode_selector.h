#pragma once

#include "robot_state.h"

// Initialize button GPIO (GPIO5, internal pullup, active LOW).
// Starts an internal FreeRTOS task to handle debounce and multi-press counting.
void mode_selector_init(void);

// Returns the currently active mode (updated after each multi-press sequence).
robot_mode_t mode_selector_get(void);
