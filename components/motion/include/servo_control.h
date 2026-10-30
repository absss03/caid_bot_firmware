#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <stdbool.h>
#include <stddef.h>

#define SERVO_ARM_JOINTS 4

/* Objetivos: se pueden escribir desde cualquier core (mailbox con spinlock).
 * servo_ctrl_update() y servo_ctrl_arm_busy() se llaman solo desde el lazo del Core 1. */
void servo_ctrl_init(void);
void servo_ctrl_set_arm_target(const float *deg, size_t count);  /* base, hombro, codo, pinza */
void servo_ctrl_set_camera_target(float deg);
void servo_ctrl_update(float dt_s);
bool servo_ctrl_arm_busy(void);

#endif /* SERVO_CONTROL_H */