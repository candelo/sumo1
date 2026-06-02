#include "battery_monitor.h"
#include "hal/hardware_map.h"
#include "esp_log.h"

// GPIO4 (ADC2_CH0) ahora lo usa el sensor de distancia izquierdo.
// Reasignar BATT_GPIO a un pin libre antes de rehabilitar esta función.

static const char *TAG = "BattMon";

void battery_monitor_init(void)
{
    ESP_LOGW(TAG, "Monitoreo de bateria deshabilitado – GPIO4 en uso por sensor dist.");
}

float battery_monitor_read_volts(void)
{
    return 0.0f;
}
