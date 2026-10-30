#include "core_one.h"

#include <math.h>

#include "esp_log.h"
#include "ledc_driver.h"
#include "motion_control.h"
#include "pcnt_driver.h"
#include "servo_control.h"
#include "ultrasonic_driver.h"

#define TAG "control"

/*
 * Stop local por ultrasónicos. Cancela el avance y el giro, pero deja pasar
 * marcha atrás (v < 0) para que el robot pueda salir de un bloqueo.
 */
static bool aplicar_stop_local(float dist_frente_m, float dist_izq_m, float dist_der_m,
                               cmd_vel_t *cmd) {
    const bool bloqueo = (dist_frente_m < UMBRAL_STOP_FRONTAL_M) ||
                         (dist_izq_m < UMBRAL_STOP_LATERAL_M) ||
                         (dist_der_m < UMBRAL_STOP_LATERAL_M);
    if (bloqueo) {
        cmd->v = (cmd->v < 0.0f) ? cmd->v : 0.0f;
        cmd->w = 0.0f;
    }
    return bloqueo;
}

static void publicar_snapshot_telemetria(const motion_out_t *mo,
                                         float dist_frente_m, float dist_izq_m, float dist_der_m,
                                         bool robot_bloqueado, bool brazo_ocupado) {
    telemetria_t snapshot = {0};
    static uint32_t seq = 0;

    snapshot.seq = ++seq;
    snapshot.stamp_us = esp_timer_get_time();
    snapshot.ticks_izq = mo->ticks_izq;
    snapshot.ticks_der = mo->ticks_der;
    snapshot.dist_frente_m = dist_frente_m;
    snapshot.dist_izq_m = dist_izq_m;
    snapshot.dist_der_m = dist_der_m;
    snapshot.modo = brazo_ocupado ? MODO_BRAZO : MODO_CRUCERO;
    snapshot.maniobra_id = 0;
    snapshot.maniobra_estado = MANIOBRA_TERMINADA;
    snapshot.maniobra_motivo = mo->fallo ? MOTIVO_BLOQUEO
                              : (robot_bloqueado ? MOTIVO_OBSTACULO : MOTIVO_NINGUNO);
    snapshot.brazo_ocupado = brazo_ocupado;

    xQueueOverwrite(cola_telemetria, &snapshot);
}

void vTaskCore1_PID(void *pvParameters) {
    (void)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrecuencia = pdMS_TO_TICKS(PERIODO_CONTROL_MS);
    const float dt_s = PERIODO_CONTROL_MS / 1000.0f;

    cmd_vel_t orden_actual = {0.0f, 0.0f, 0, false};
    int32_t raw_izq = 0;
    int32_t raw_der = 0;
    float dist_frente_m = INFINITY;
    float dist_izq_m = INFINITY;
    float dist_der_m = INFINITY;
    bool fallo_prev = false;

    while (1) {
        const int64_t ahora_us = esp_timer_get_time();

        if (!setpoint_leer(&orden_actual, pdMS_TO_TICKS(1))) {
            orden_actual.v = 0.0f;
            orden_actual.w = 0.0f;
            orden_actual.valida = false;
        }

        /* Watchdog: sin cmd_vel reciente, frenar. */
        if ((ahora_us - orden_actual.stamp_us) > (WATCHDOG_CMD_MS * 1000LL)) {
            orden_actual.v = 0.0f;
            orden_actual.w = 0.0f;
            orden_actual.valida = false;
        }

        leer_encoders(&raw_izq, &raw_der);
        leer_ultrasonicos(&dist_frente_m, &dist_izq_m, &dist_der_m);

        const bool robot_bloqueado =
            aplicar_stop_local(dist_frente_m, dist_izq_m, dist_der_m, &orden_actual);

        /* Stop local o watchdog: frenar sin rampa. */
        const bool parada = robot_bloqueado || !orden_actual.valida;

        motion_out_t mo = {0};
        motion_ctrl_step(orden_actual.v, orden_actual.w, parada, raw_izq, raw_der, dt_s, &mo);
        escribir_motores(mo.pwm_izq, mo.pwm_der);

        if (mo.fallo && !fallo_prev) {
            ESP_LOGE(TAG, "rueda atascada o encoder sin señal: motores cortados");
        }
        fallo_prev = mo.fallo;

        /* Servos: avance con límite de velocidad hacia el último objetivo recibido. */
        servo_ctrl_update(dt_s);

        publicar_snapshot_telemetria(&mo, dist_frente_m, dist_izq_m, dist_der_m,
                                     robot_bloqueado, servo_ctrl_arm_busy());

        vTaskDelayUntil(&xLastWakeTime, xFrecuencia);
    }
}

void leer_encoders(int32_t *raw_izq, int32_t *raw_der) {
    if (raw_izq == NULL || raw_der == NULL) {
        return;
    }
    *raw_izq = pcnt_get_left_ticks();
    *raw_der = pcnt_get_right_ticks();
}

void leer_ultrasonicos(float *dist_frente_m, float *dist_izq_m, float *dist_der_m) {
    if (dist_frente_m == NULL || dist_izq_m == NULL || dist_der_m == NULL) {
        return;
    }
    *dist_frente_m = ultrasonic_read_front();
    *dist_izq_m = ultrasonic_read_left();
    *dist_der_m = ultrasonic_read_right();
}

void escribir_motores(float pwm_izq, float pwm_der) {
    ledc_set_motor_left(pwm_izq);
    ledc_set_motor_right(pwm_der);
}