#ifndef LEDC_DRIVER_H
#define LEDC_DRIVER_H

#include <stdint.h>

void ledc_init(void);
void ledc_set_motor_left(float duty_normalized);   /* -1.0 .. 1.0 */
void ledc_set_motor_right(float duty_normalized);  /* -1.0 .. 1.0 */
void ledc_set_servo_camera(float angle_deg);       /* 0 .. 180 */
void ledc_set_servo_arm(uint8_t index, float angle_deg);

#endif /* LEDC_DRIVER_H */