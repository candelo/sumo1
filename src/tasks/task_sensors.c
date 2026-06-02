#include "task_sensors.h"
#include "robot_state.h"
#include "gp2y0e03_driver.h"
#include "line_sensor.h"
#include "battery_monitor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#define SENSOR_PERIOD_MS    10
// Battery is slow – read every 1s (every 100 sensor cycles)
#define BATT_DIVIDER        100

void vTaskSensors(void *arg)
{
    (void)arg;
    int batt_tick = 0;

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        sensor_data_t buf;

        // Distance sensors – única tarea que toca el filtro MA
        for (int i = 0; i < DIST_SENSOR_COUNT; i++) {
            int filtered         = gp2y_read_raw(i);   // lee HW + actualiza MA
            buf.dist_raw[i]      = filtered;
            buf.dist_filtered[i] = filtered;
            buf.dist_cm[i]       = gp2y_raw_to_cm(i, filtered);
        }

        // Line sensors
        buf.line_detected[0] = line_sensor_read(0);
        buf.line_detected[1] = line_sensor_read(1);

        // Battery (throttled)
        if (++batt_tick >= BATT_DIVIDER) {
            buf.battery_voltage = battery_monitor_read_volts();
            batt_tick = 0;
        } else {
            // Keep previous value
            if (xSemaphoreTake(g_sensor_mutex, 0) == pdTRUE) {
                buf.battery_voltage = g_sensor_data.battery_voltage;
                xSemaphoreGive(g_sensor_mutex);
            } else {
                buf.battery_voltage = 0.0f;
            }
        }

        buf.timestamp_us = esp_timer_get_time();

        // Publish to shared structure
        xSemaphoreTake(g_sensor_mutex, portMAX_DELAY);
        g_sensor_data = buf;
        xSemaphoreGive(g_sensor_mutex);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}
