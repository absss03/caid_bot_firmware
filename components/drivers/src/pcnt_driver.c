#include "pcnt_driver.h"

#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "pins.h"

/*
 * Encoder IR de un solo canal por rueda: no entrega sentido de giro.
 * El PCNT cuenta subida y bajada (x2) siempre hacia arriba; el sentido lo
 * aplica motion_ctrl según el signo del PWM. Los pines ENC_*_AUX no se usan.
 */
#define ENC_LIMIT       30000   /* con accum_count el driver acumula los desbordes */
#define ENC_GLITCH_NS   1000    /* filtro de glitches (máx. ~12000 ns a 80 MHz APB) */

static pcnt_unit_handle_t s_unit_izq = NULL;
static pcnt_unit_handle_t s_unit_der = NULL;

static pcnt_unit_handle_t counter_unit_create(int gpio_pulse) {
    pcnt_unit_config_t unit_cfg = {
        .high_limit = ENC_LIMIT,
        .low_limit = -ENC_LIMIT,
        .flags.accum_count = true,
    };
    pcnt_unit_handle_t unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &unit));

    pcnt_glitch_filter_config_t filter_cfg = {.max_glitch_ns = ENC_GLITCH_NS};
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter_cfg));

    pcnt_chan_config_t chan_cfg = {.edge_gpio_num = gpio_pulse, .level_gpio_num = -1};
    pcnt_channel_handle_t chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_cfg, &chan));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, ENC_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, -ENC_LIMIT));

    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    ESP_ERROR_CHECK(pcnt_unit_start(unit));
    return unit;
}

void pcnt_driver_init(void) {
    s_unit_izq = counter_unit_create(ENC_IZQ);
    s_unit_der = counter_unit_create(ENC_DER);
}

void pcnt_reset_left(void) {
    if (s_unit_izq) pcnt_unit_clear_count(s_unit_izq);
}

void pcnt_reset_right(void) {
    if (s_unit_der) pcnt_unit_clear_count(s_unit_der);
}

/* Cuentas acumuladas, siempre crecientes (sin signo de sentido). */
int32_t pcnt_get_left_ticks(void) {
    int count = 0;
    if (s_unit_izq) pcnt_unit_get_count(s_unit_izq, &count);
    return (int32_t)count;
}

int32_t pcnt_get_right_ticks(void) {
    int count = 0;
    if (s_unit_der) pcnt_unit_get_count(s_unit_der, &count);
    return (int32_t)count;
}