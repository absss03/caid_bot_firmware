#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float   pwm_izq;       /* -1..1, ya con zona muerta compensada */
    float   pwm_der;
    int32_t ticks_izq;     /* acumulados con signo */
    int32_t ticks_der;
    float   v_izq_ms;      /* velocidad medida y filtrada */
    float   v_der_ms;
    bool    fallo;         /* rueda atascada / encoder sin señal: motores cortados */
} motion_out_t;

/*
 * Un paso del lazo de velocidad (llamar cada dt_s).
 *  raw_izq/raw_der : cuentas del encoder, siempre crecientes (sin sentido)
 *  parada          : true = frenar sin rampa (stop local, watchdog)
 * El sentido de giro se toma del signo del último PWM aplicado.
 */
void motion_ctrl_step(float v_cmd, float w_cmd, bool parada,
                      int32_t raw_izq, int32_t raw_der, float dt_s,
                      motion_out_t *out);

#endif /* MOTION_CTRL_H */