#include "hardware_init.h"

#include "driver/gpio.h"
#include "pins.h"

/*
 * Solo deja los pines de potencia en estado seguro (bajo) antes de que LEDC
 * los tome. Cada driver configura sus propios pines:
 *   ledc_driver       -> motores y servos
 *   pcnt_driver       -> encoders
 *   ultrasonic_driver -> trigger / echo
 */
static void salida_en_bajo(gpio_num_t pin) {
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
}

void init_hardware(void) {
    salida_en_bajo(MOTOR_IZQ_A);
    salida_en_bajo(MOTOR_IZQ_B);
    salida_en_bajo(MOTOR_DER_A);
    salida_en_bajo(MOTOR_DER_B);
    salida_en_bajo(LED_STATUS);
}