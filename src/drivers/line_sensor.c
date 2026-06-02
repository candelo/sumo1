#include "line_sensor.h"
#include "hardware_map.h"
#include "robot_state.h"
#include "driver/gpio.h"

static const gpio_num_t LINE_PINS[2] = { LINE_SENSOR_1, LINE_SENSOR_2 };

void line_sensor_init(void)
{
    for (int i = 0; i < 2; i++) {
        gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << LINE_PINS[i],
            .mode         = GPIO_MODE_INPUT,
            // GPIO 34/35 cannot use internal pullup — use external resistor
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&cfg);
    }
}

bool line_sensor_read(int idx)
{
    if (idx < 0 || idx > 1) return false;
    // BLACK_RING (standard): sensor goes LOW on white border → active_level=0
    // WHITE_RING (inverted): sensor goes HIGH on black border → active_level=1
    int active_level = (g_dohyo_type == DOHYO_BLACK_RING) ? 0 : 1;
    return gpio_get_level(LINE_PINS[idx]) == active_level;
}
