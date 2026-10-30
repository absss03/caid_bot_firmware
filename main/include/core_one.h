#ifndef CORE_ONE_H
#define CORE_ONE_H

#include <stddef.h>

#include "shared_data.h"

/* Lazo de control (corre en el Core 1). */
void vTaskCore1_PID(void *pvParameters);

/* Cuentas crudas del encoder (crecientes, sin sentido). */
void leer_encoders(int32_t *raw_izq, int32_t *raw_der);
void leer_ultrasonicos(float *dist_frente_m, float *dist_izq_m, float *dist_der_m);
void escribir_motores(float pwm_izq, float pwm_der);

#endif /* CORE_ONE_H */