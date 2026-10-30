#include "ledc_driver.h"

#include <math.h>

#include "driver/ledc.h"
#include "pins.h"

#define MOTOR_RES_BITS   LEDC_TIMER_10_BIT
#define MOTOR_DUTY_MAX   1023u
#define SERVO_RES_BITS   LEDC_TIMER_14_BIT
#define SERVO_DUTY_STEPS 16384.0f
#define SERVO_PERIOD_US  (1000000.0f / SERVO_PWM_HZ)

/* Ancho de pulso del servo: ajustar a cada modelo (SG90 ~500-2400, MG996R ~500-2500). */
#define SERVO_MIN_US     500.0f
#define SERVO_MAX_US     2500.0f

#define ARM_JOINTS 4
static const ledc_channel_t ARM_CH[ARM_JOINTS] = {
    LEDC_CHANNEL_4, LEDC_CHANNEL_5, LEDC_CHANNEL_6, LEDC_CHANNEL_7,
};

static void configure_channel(ledc_mode_t speed_mode, ledc_channel_t channel,
                              gpio_num_t pin, ledc_timer_t timer) {
    ledc_channel_config_t cfg = {
        .gpio_num = pin,
        .speed_mode = speed_mode,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = timer,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&cfg));
}

static void set_duty(ledc_mode_t mode, ledc_channel_t ch, uint32_t duty) {
    ledc_set_duty(mode, ch, duty);
    ledc_update_duty(mode, ch);
}

void ledc_init(void) {
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = MOTOR_RES_BITS,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = MOTOR_PWM_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    configure_channel(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, MOTOR_IZQ_A, LEDC_TIMER_0);
    configure_channel(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, MOTOR_IZQ_B, LEDC_TIMER_0);
    configure_channel(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, MOTOR_DER_A, LEDC_TIMER_0);
    configure_channel(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, MOTOR_DER_B, LEDC_TIMER_0);

    /* Servos: 50 Hz exige 14 bits (con 10 el divisor excede el máximo del LEDC). */
    timer_cfg.timer_num = LEDC_TIMER_1;
    timer_cfg.freq_hz = SERVO_PWM_HZ;
    timer_cfg.duty_resolution = SERVO_RES_BITS;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));
    configure_channel(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, SERVO_BASE, LEDC_TIMER_1);
    configure_channel(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5, SERVO_HOMBRO, LEDC_TIMER_1);
    configure_channel(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6, SERVO_CODO, LEDC_TIMER_1);
    configure_channel(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_7, SERVO_PINZA, LEDC_TIMER_1);

    /* Los 8 canales de low-speed ya están usados: la cámara va en high-speed (solo ESP32 clásico). */
    timer_cfg.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_cfg.timer_num = LEDC_TIMER_0;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));
    configure_channel(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, SERVO_CAMARA, LEDC_TIMER_0);
}

/* Puente H con dos pines PWM: un pin lleva el duty, el otro queda en 0. */
static void set_motor(ledc_channel_t ch_a, ledc_channel_t ch_b, float cmd) {
    if (!isfinite(cmd)) {
        cmd = 0.0f;
    }
    if (cmd > 1.0f) cmd = 1.0f;
    if (cmd < -1.0f) cmd = -1.0f;

    const uint32_t duty = (uint32_t)(fabsf(cmd) * (float)MOTOR_DUTY_MAX);
    const ledc_channel_t on = (cmd >= 0.0f) ? ch_a : ch_b;
    const ledc_channel_t off = (cmd >= 0.0f) ? ch_b : ch_a;

    /* Primero apagar el lado opuesto para no cruzar ambos pines en alto. */
    set_duty(LEDC_LOW_SPEED_MODE, off, 0);
    set_duty(LEDC_LOW_SPEED_MODE, on, duty);
}

void ledc_set_motor_left(float duty_normalized) {
    set_motor(LEDC_CHANNEL_0, LEDC_CHANNEL_1, duty_normalized);
}

void ledc_set_motor_right(float duty_normalized) {
    set_motor(LEDC_CHANNEL_2, LEDC_CHANNEL_3, duty_normalized);
}

static uint32_t servo_angle_to_duty(float deg) {
    if (!isfinite(deg)) deg = 90.0f;
    if (deg < 0.0f) deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    const float us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (deg / 180.0f);
    return (uint32_t)(us * SERVO_DUTY_STEPS / SERVO_PERIOD_US + 0.5f);
}

void ledc_set_servo_camera(float angle_deg) {
    set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, servo_angle_to_duty(angle_deg));
}

void ledc_set_servo_arm(uint8_t index, float angle_deg) {
    if (index >= ARM_JOINTS) {
        return;
    }
    set_duty(LEDC_LOW_SPEED_MODE, ARM_CH[index], servo_angle_to_duty(angle_deg));
}