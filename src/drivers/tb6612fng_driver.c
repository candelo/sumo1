#include "tb6612fng_driver.h"
#include "hardware_map.h"

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "TB6612";

// MCPWM handles
static mcpwm_timer_handle_t   s_timer     = NULL;
static mcpwm_oper_handle_t    s_oper_l    = NULL;
static mcpwm_oper_handle_t    s_oper_r    = NULL;
static mcpwm_cmpr_handle_t    s_cmp_l     = NULL;
static mcpwm_cmpr_handle_t    s_cmp_r     = NULL;
static mcpwm_gen_handle_t     s_gen_l     = NULL;
static mcpwm_gen_handle_t     s_gen_r     = NULL;

// Period ticks: 10MHz / 20kHz = 500 ticks
#define PWM_PERIOD_TICKS  (MOTOR_PWM_RES_HZ / MOTOR_PWM_FREQ_HZ)

static void _gpio_output(gpio_num_t pin)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}

static void _setup_generator(mcpwm_oper_handle_t oper,
                              mcpwm_cmpr_handle_t cmp,
                              mcpwm_gen_handle_t *gen_out,
                              gpio_num_t gpio)
{
    mcpwm_generator_config_t gen_cfg = { .gen_gpio_num = gpio };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_cfg, gen_out));

    // High at timer empty (start of period), low at compare match
    mcpwm_gen_timer_event_action_t tea = {
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .event     = MCPWM_TIMER_EVENT_EMPTY,
        .action    = MCPWM_GEN_ACTION_HIGH,
    };
    mcpwm_gen_compare_event_action_t cea = {
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .comparator = cmp,
        .action    = MCPWM_GEN_ACTION_LOW,
    };
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(*gen_out, tea));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(*gen_out, cea));
}

void motors_init(void)
{
    // STBY (MOTOR_ENABLE) primero: escribir 0 en el registro de salida ANTES de
    // configurar como output para que el pin nunca flote en alto durante el boot.
    // El TB6612FNG tiene pull-up interno en STBY y arrancaría si flotara.
    gpio_set_level(MOTOR_ENABLE, 0);
    _gpio_output(MOTOR_ENABLE);     // ahora el pin sale inmediatamente en LOW

    // Dirección – también asegurar LOW antes de activar como salida
    gpio_set_level(MOTOR_LIN1, 0);  _gpio_output(MOTOR_LIN1);
    gpio_set_level(MOTOR_LIN2, 0);  _gpio_output(MOTOR_LIN2);
    gpio_set_level(MOTOR_RIN1, 0);  _gpio_output(MOTOR_RIN1);
    gpio_set_level(MOTOR_RIN2, 0);  _gpio_output(MOTOR_RIN2);

    // MCPWM Timer (shared between both operators)
    mcpwm_timer_config_t timer_cfg = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MOTOR_PWM_RES_HZ,
        .period_ticks  = PWM_PERIOD_TICKS,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &s_timer));

    // Operators (one per motor channel)
    mcpwm_operator_config_t oper_cfg = { .group_id = 0 };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &s_oper_l));
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &s_oper_r));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(s_oper_l, s_timer));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(s_oper_r, s_timer));

    // Comparators
    mcpwm_comparator_config_t cmp_cfg = { .flags.update_cmp_on_tez = true };
    ESP_ERROR_CHECK(mcpwm_new_comparator(s_oper_l, &cmp_cfg, &s_cmp_l));
    ESP_ERROR_CHECK(mcpwm_new_comparator(s_oper_r, &cmp_cfg, &s_cmp_r));

    // Start at 0% duty
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_cmp_l, 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_cmp_r, 0));

    // Generators (waveform on actual GPIO pins)
    _setup_generator(s_oper_l, s_cmp_l, &s_gen_l, MOTOR_PWM_L);
    _setup_generator(s_oper_r, s_cmp_r, &s_gen_r, MOTOR_PWM_R);

    ESP_ERROR_CHECK(mcpwm_timer_enable(s_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "MCPWM init OK – %d Hz, %d ticks/period", MOTOR_PWM_FREQ_HZ, PWM_PERIOD_TICKS);
}

void motors_set_speed(motor_side_t side, float duty)
{
    // Invertir dirección si el motor está montado al revés
    if (side == MOTOR_LEFT  && MOTOR_LEFT_INVERTED)  duty = -duty;
    if (side == MOTOR_RIGHT && MOTOR_RIGHT_INVERTED) duty = -duty;

    // Clamp
    if (duty >  100.0f) duty =  100.0f;
    if (duty < -100.0f) duty = -100.0f;

    uint32_t ticks = (uint32_t)(fabsf(duty) / 100.0f * PWM_PERIOD_TICKS);

    if (side == MOTOR_LEFT) {
        if (duty > 0.0f) {
            gpio_set_level(MOTOR_LIN1, 1);
            gpio_set_level(MOTOR_LIN2, 0);
        } else if (duty < 0.0f) {
            gpio_set_level(MOTOR_LIN1, 0);
            gpio_set_level(MOTOR_LIN2, 1);
        } else {
            gpio_set_level(MOTOR_LIN1, 0);
            gpio_set_level(MOTOR_LIN2, 0);
        }
        mcpwm_comparator_set_compare_value(s_cmp_l, ticks);
    } else {
        if (duty > 0.0f) {
            gpio_set_level(MOTOR_RIN1, 1);
            gpio_set_level(MOTOR_RIN2, 0);
        } else if (duty < 0.0f) {
            gpio_set_level(MOTOR_RIN1, 0);
            gpio_set_level(MOTOR_RIN2, 1);
        } else {
            gpio_set_level(MOTOR_RIN1, 0);
            gpio_set_level(MOTOR_RIN2, 0);
        }
        mcpwm_comparator_set_compare_value(s_cmp_r, ticks);
    }
}

void motors_drive(float left, float right)
{
    motors_set_speed(MOTOR_LEFT,  left);
    motors_set_speed(MOTOR_RIGHT, right);
}

void motors_brake(void)
{
    // TB6612FNG short brake: IN1=HIGH IN2=HIGH on both channels
    gpio_set_level(MOTOR_LIN1, 1);
    gpio_set_level(MOTOR_LIN2, 1);
    gpio_set_level(MOTOR_RIN1, 1);
    gpio_set_level(MOTOR_RIN2, 1);
    mcpwm_comparator_set_compare_value(s_cmp_l, PWM_PERIOD_TICKS); // 100% duty
    mcpwm_comparator_set_compare_value(s_cmp_r, PWM_PERIOD_TICKS);
}

void motors_coast(void)
{
    gpio_set_level(MOTOR_LIN1, 0);
    gpio_set_level(MOTOR_LIN2, 0);
    gpio_set_level(MOTOR_RIN1, 0);
    gpio_set_level(MOTOR_RIN2, 0);
    mcpwm_comparator_set_compare_value(s_cmp_l, 0);
    mcpwm_comparator_set_compare_value(s_cmp_r, 0);
}

void motors_enable(bool en)
{
    gpio_set_level(MOTOR_ENABLE, en ? 1 : 0);
}
