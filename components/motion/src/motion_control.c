#include "motion_control.h"

#include <math.h>

#include "robot_params.h"

typedef struct {
    float   integ;
    float   v_filt;     /* velocidad medida, filtrada [m/s] */
    float   v_ramp;     /* objetivo después de la rampa [m/s] */
    int32_t raw_prev;
    int32_t ticks;      /* acumulados con signo */
    int8_t  dir;        /* sentido del último PWM no nulo */
    float   stall_s;
} rueda_t;

static rueda_t s_izq = {.dir = 1};
static rueda_t s_der = {.dir = 1};
static bool  s_primed = false;
static bool  s_fallo = false;
static float s_cmd_nulo_s = 0.0f;

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static float step_toward(float cur, float tgt, float max_step) {
    const float d = tgt - cur;
    if (fabsf(d) <= max_step) {
        return tgt;
    }
    return cur + copysignf(max_step, d);
}

static void estimar(rueda_t *r, int32_t raw, float dt) {
    const int32_t delta = raw - r->raw_prev;
    r->raw_prev = raw;
    r->ticks += r->dir * delta;
    const float v_inst = r->dir * ((float)delta / TICKS_POR_METRO) / dt;
    r->v_filt += VEL_FILTRO_ALPHA * (v_inst - r->v_filt);
}

/* PI con feedforward. Devuelve u en -1..1 (fracción de V_MAX). */
static float pi_rueda(rueda_t *r, float v_obj, float dt) {
    if (fabsf(v_obj) < V_OBJ_MINIMA_MS) {
        r->integ = 0.0f;
        return 0.0f;
    }

    const float ff = v_obj / V_MAX_MS;
    const float err = (v_obj - r->v_filt) / V_MAX_MS;

    const float integ_new = clampf(r->integ + PID_KI * err * dt, -PID_INTEG_MAX, PID_INTEG_MAX);
    const float u_try = ff + PID_KP * err + integ_new;

    /* Anti-windup: no integrar si satura y el error empuja más hacia la saturación. */
    const bool satura = (fabsf(u_try) >= 1.0f) && ((u_try > 0.0f) == (err > 0.0f));
    if (!satura) {
        r->integ = integ_new;
    }

    float u = clampf(ff + PID_KP * err + r->integ, -1.0f, 1.0f);
    if (u * v_obj < 0.0f) {
        u = 0.0f;   /* nunca empujar contra el sentido pedido */
    }
    return u;
}

/* u (fracción de velocidad) -> PWM que salta la zona muerta del motor. */
static float zona_muerta(float u) {
    if (u == 0.0f) {
        return 0.0f;
    }
    return copysignf(PWM_ZONA_MUERTA + (1.0f - PWM_ZONA_MUERTA) * fabsf(u), u);
}

static void chequear_stall(rueda_t *r, float pwm, float dt) {
    if (fabsf(pwm) >= STALL_PWM_MIN && fabsf(r->v_filt) < STALL_VEL_MAX_MS) {
        r->stall_s += dt;
    } else {
        r->stall_s = 0.0f;
    }
}

static void resetear_rueda(rueda_t *r) {
    r->integ = 0.0f;
    r->v_ramp = 0.0f;
    r->stall_s = 0.0f;
}

void motion_ctrl_step(float v_cmd, float w_cmd, bool parada,
                      int32_t raw_izq, int32_t raw_der, float dt_s,
                      motion_out_t *out) {
    if (!s_primed) {
        s_izq.raw_prev = raw_izq;
        s_der.raw_prev = raw_der;
        s_primed = true;
    }

    estimar(&s_izq, raw_izq, dt_s);
    estimar(&s_der, raw_der, dt_s);

    /* Cinemática diferencial. */
    const float v_izq_t = clampf(v_cmd - w_cmd * ANCHO_EJE_M * 0.5f, -V_MAX_MS, V_MAX_MS);
    const float v_der_t = clampf(v_cmd + w_cmd * ANCHO_EJE_M * 0.5f, -V_MAX_MS, V_MAX_MS);

    if (parada) {
        s_izq.v_ramp = v_izq_t;   /* sin rampa */
        s_der.v_ramp = v_der_t;
    } else {
        const float paso = ACEL_MAX_MS2 * dt_s;
        s_izq.v_ramp = step_toward(s_izq.v_ramp, v_izq_t, paso);
        s_der.v_ramp = step_toward(s_der.v_ramp, v_der_t, paso);
    }

    /* Rearme del fallo: comando nulo sostenido. */
    if (fabsf(v_cmd) < V_OBJ_MINIMA_MS && fabsf(w_cmd) < V_OBJ_MINIMA_MS) {
        s_cmd_nulo_s += dt_s;
        if (s_fallo && s_cmd_nulo_s >= FALLO_REARME_S) {
            s_fallo = false;
            resetear_rueda(&s_izq);
            resetear_rueda(&s_der);
        }
    } else {
        s_cmd_nulo_s = 0.0f;
    }

    float pwm_i = 0.0f;
    float pwm_d = 0.0f;
    if (!s_fallo) {
        pwm_i = zona_muerta(pi_rueda(&s_izq, s_izq.v_ramp, dt_s));
        pwm_d = zona_muerta(pi_rueda(&s_der, s_der.v_ramp, dt_s));

        chequear_stall(&s_izq, pwm_i, dt_s);
        chequear_stall(&s_der, pwm_d, dt_s);
        if (s_izq.stall_s > STALL_TIEMPO_S || s_der.stall_s > STALL_TIEMPO_S) {
            s_fallo = true;
            pwm_i = 0.0f;
            pwm_d = 0.0f;
        }
    }

    if (pwm_i != 0.0f) s_izq.dir = (pwm_i > 0.0f) ? 1 : -1;
    if (pwm_d != 0.0f) s_der.dir = (pwm_d > 0.0f) ? 1 : -1;

    out->pwm_izq = pwm_i;
    out->pwm_der = pwm_d;
    out->ticks_izq = s_izq.ticks;
    out->ticks_der = s_der.ticks;
    out->v_izq_ms = s_izq.v_filt;
    out->v_der_ms = s_der.v_filt;
    out->fallo = s_fallo;
}