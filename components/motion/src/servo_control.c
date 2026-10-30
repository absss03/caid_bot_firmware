#include "servo_control.h"

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ledc_driver.h"


#define N SERVO_ARM_JOINTS

/* ---- AJUSTAR A TU BRAZO (orden: base, hombro, codo, pinza) ---- */
static const float ARM_HOME_DEG[N]      = {90.0f, 90.0f, 90.0f, 90.0f};
static const float ARM_MIN_DEG[N]       = { 0.0f,  0.0f,  0.0f,  0.0f};   /* límites mecánicos */
static const float ARM_MAX_DEG[N]       = {180.0f, 180.0f, 180.0f, 180.0f};
static const float ARM_MAX_SPEED_DPS[N] = {60.0f, 60.0f, 60.0f, 120.0f}; /* grados/s */

#define CAM_HOME_DEG      90.0f
#define CAM_MIN_DEG       0.0f
#define CAM_MAX_DEG       180.0f
#define CAM_MAX_SPEED_DPS 120.0f

#define BUSY_TOL_DEG      0.5f

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static float s_arm_tgt[N];
static float s_cam_tgt;

/* Posición actual: solo la toca el Core 1. */
static float s_arm_cur[N];
static float s_cam_cur;

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

void servo_ctrl_init(void) {
    portENTER_CRITICAL(&s_lock);
    for (int i = 0; i < N; ++i) {
        s_arm_tgt[i] = ARM_HOME_DEG[i];
    }
    s_cam_tgt = CAM_HOME_DEG;
    portEXIT_CRITICAL(&s_lock);

    for (int i = 0; i < N; ++i) {
        s_arm_cur[i] = ARM_HOME_DEG[i];
        ledc_set_servo_arm((uint8_t)i, s_arm_cur[i]);
    }
    s_cam_cur = CAM_HOME_DEG;
    ledc_set_servo_camera(s_cam_cur);
}

void servo_ctrl_set_arm_target(const float *deg, size_t count) {
    if (deg == NULL) {
        return;
    }
    if (count > N) {
        count = N;
    }

    float tmp[N];
    for (size_t i = 0; i < count; ++i) {
        if (!isfinite(deg[i])) {
            return;   /* se descarta el mensaje completo */
        }
        tmp[i] = clampf(deg[i], ARM_MIN_DEG[i], ARM_MAX_DEG[i]);
    }

    portENTER_CRITICAL(&s_lock);
    for (size_t i = 0; i < count; ++i) {
        s_arm_tgt[i] = tmp[i];
    }
    portEXIT_CRITICAL(&s_lock);
}

void servo_ctrl_set_camera_target(float deg) {
    if (!isfinite(deg)) {
        return;
    }
    const float v = clampf(deg, CAM_MIN_DEG, CAM_MAX_DEG);
    portENTER_CRITICAL(&s_lock);
    s_cam_tgt = v;
    portEXIT_CRITICAL(&s_lock);
}

void servo_ctrl_update(float dt_s) {
    float tgt[N];
    float cam;

    portENTER_CRITICAL(&s_lock);
    for (int i = 0; i < N; ++i) {
        tgt[i] = s_arm_tgt[i];
    }
    cam = s_cam_tgt;
    portEXIT_CRITICAL(&s_lock);

    for (int i = 0; i < N; ++i) {
        const float next = step_toward(s_arm_cur[i], tgt[i], ARM_MAX_SPEED_DPS[i] * dt_s);
        if (next != s_arm_cur[i]) {
            s_arm_cur[i] = next;
            ledc_set_servo_arm((uint8_t)i, next);
        }
    }

    const float next_cam = step_toward(s_cam_cur, cam, CAM_MAX_SPEED_DPS * dt_s);
    if (next_cam != s_cam_cur) {
        s_cam_cur = next_cam;
        ledc_set_servo_camera(next_cam);
    }
}

bool servo_ctrl_arm_busy(void) {
    for (int i = 0; i < N; ++i) {
        if (fabsf(s_arm_tgt[i] - s_arm_cur[i]) > BUSY_TOL_DEG) {
            return true;
        }
    }
    return false;
}