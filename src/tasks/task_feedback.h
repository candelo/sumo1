#pragma once

// Feedback task: processes feedback_cmd_t from g_feedback_queue.
// Drives speaker (LEDC) and LEDs. Runs at low priority.
void vTaskFeedback(void *arg);
