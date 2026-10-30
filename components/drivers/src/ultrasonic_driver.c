#include "ultrasonic_driver.h"

#include <math.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "pins.h"

#define ULTRASONIC_SLOT_MS 30

#define US_NUM            3  // comentado para pruebas
#define US_MAX_RANGE_M    4.0f
#define METROS_POR_US     0.0001715f    /* 343 m/s / 2 (ida y vuelta) */

#define RMT_RES_HZ        1000000       /* 1 tick = 1 us */
#define RMT_MEM_SYMBOLS   64
#define ECHO_MIN_NS       1000          /* filtro de glitches (máx. ~3100 ns) */
#define ECHO_MAX_NS       30000000      /* fin de recepción; debe superar el eco más largo útil (~23 ms) */

typedef struct {
    gpio_num_t trig;
    gpio_num_t echo;
    rmt_channel_handle_t chan;
    rmt_symbol_word_t symbols[RMT_MEM_SYMBOLS];
    volatile int32_t width_us;   /* -1 = sin eco */
    volatile bool armed;         /* hay un rmt_receive en curso */
    volatile bool fresh;         /* llegó una medida desde el último ciclo */
} us_sensor_t;

/* 0 = frente, 1 = izquierda, 2 = derecha */
static us_sensor_t s_us[US_NUM] = {
    {.trig = US_FRONTAL_TRIG, .echo = US_FRONTAL_ECHO, .width_us = -1},
    {.trig = US_IZQ_TRIG,     .echo = US_IZQ_ECHO,     .width_us = -1},
    {.trig = US_DER_TRIG,     .echo = US_DER_ECHO,     .width_us = -1},
};

/* Corre en ISR (sin float): el hardware ya midió el ancho del pulso de eco. */
static bool IRAM_ATTR echo_done_cb(rmt_channel_handle_t ch,
                                   const rmt_rx_done_event_data_t *edata, void *user_ctx) {
    (void)ch;
    us_sensor_t *s = (us_sensor_t *)user_ctx;
    int32_t w = -1;

    for (size_t i = 0; i < edata->num_symbols; ++i) {
        const rmt_symbol_word_t sym = edata->received_symbols[i];
        if (sym.level0) { w = (int32_t)sym.duration0; break; }
        if (sym.level1) { w = (int32_t)sym.duration1; break; }
    }

    s->width_us = w;
    s->fresh = true;
    s->armed = false;
    return false;   /* no despierta ninguna tarea */
}

static void arm_receive(us_sensor_t *s) {
    static const rmt_receive_config_t cfg = {
        .signal_range_min_ns = ECHO_MIN_NS,
        .signal_range_max_ns = ECHO_MAX_NS,
    };
    s->armed = true;
    if (rmt_receive(s->chan, s->symbols, sizeof(s->symbols), &cfg) != ESP_OK) {
        s->armed = false;
    }
}

/* Dispara un sensor por vez, rotando, para evitar interferencia entre ellos. */
static void us_timer_cb(void *arg) {
    static uint8_t idx = 0;
    (void)arg;

    us_sensor_t *s = &s_us[idx];

    if (!s->fresh) {
        s->width_us = -1;   /* el ciclo anterior no recibió eco */
    }
    s->fresh = false;

    if (!s->armed) {
        arm_receive(s);     /* si sigue armado, espera el próximo eco */
    }

    gpio_set_level(s->trig, 1);
    esp_rom_delay_us(10);
    gpio_set_level(s->trig, 0);

    idx = (uint8_t)((idx + 1) % US_NUM);
}

static float us_get(int i) {
    const int32_t w = s_us[i].width_us;
    if (w < 0) {
        return INFINITY;
    }
    const float d = (float)w * METROS_POR_US;
    return (d <= US_MAX_RANGE_M) ? d : INFINITY;
}

void ultrasonic_init(void) {
    for (int i = 0; i < US_NUM; ++i) {
        us_sensor_t *s = &s_us[i];

        gpio_config_t trig = {
            .pin_bit_mask = 1ULL << s->trig,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&trig));
        gpio_set_level(s->trig, 0);

        /* RMT configura el pin de eco como entrada. */
        rmt_rx_channel_config_t rx_cfg = {
            .gpio_num = s->echo,
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = RMT_RES_HZ,
            .mem_block_symbols = RMT_MEM_SYMBOLS,
        };
        ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &s->chan));

        rmt_rx_event_callbacks_t cbs = {.on_recv_done = echo_done_cb};
        ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(s->chan, &cbs, s));
        ESP_ERROR_CHECK(rmt_enable(s->chan));
    }

    const esp_timer_create_args_t args = {
        .callback = us_timer_cb,
        .name = "us_trigger",
        .dispatch_method = ESP_TIMER_TASK,
    };
    esp_timer_handle_t timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, ULTRASONIC_SLOT_MS * 1000ULL));
}

float ultrasonic_read_front(void) { return us_get(0); }

// comentado para pruebas 
float ultrasonic_read_left(void)  { return us_get(1); }
float ultrasonic_read_right(void) { return us_get(2); }