#ifndef ROBOT_PARAMS_H
#define ROBOT_PARAMS_H

/*
 * Parámetros físicos y de control. Todo lo marcado [MEDIR] arranca con un valor
 * razonable para motores TT 48:1 y se reemplaza al calibrar el robot real.
 */

/* ---- Encoder (disco IR en el eje del motor, antes de la reductora) ---- */
#define ENC_RANURAS_POR_VUELTA   20
#define ENC_FLANCOS_POR_RANURA   2       /* PCNT cuenta subida y bajada */
#define MOTOR_REDUCCION          48

/* ---- Geometría ---- */
#define RUEDA_DIAMETRO_M         0.065f
#define ANCHO_EJE_M              0.15f   /* [MEDIR] centro a centro de las ruedas */

#define PI_F                     3.14159265f
#define TICKS_POR_VUELTA_RUEDA   (ENC_RANURAS_POR_VUELTA * ENC_FLANCOS_POR_RANURA * MOTOR_REDUCCION)
#define TICKS_POR_METRO          ((float)TICKS_POR_VUELTA_RUEDA / (PI_F * RUEDA_DIAMETRO_M))

/* ---- Motor ---- */
#define V_MAX_MS                 0.50f   /* [MEDIR] m/s con PWM 100 % en el piso (sin carga ~0.68) */
#define PWM_ZONA_MUERTA          0.30f   /* [MEDIR] PWM mínimo para que la rueda arranque */

/* ---- Estimación de velocidad ---- */
#define VEL_FILTRO_ALPHA         0.4f    /* 1.0 = sin filtro; menor = más suave y más lento */

/* ---- Perfil de movimiento ---- */
#define ACEL_MAX_MS2             1.0f    /* rampa de velocidad de cada rueda */

/* ---- PI de velocidad por rueda (en unidades normalizadas por V_MAX_MS) ---- */
#define PID_KP                   0.5f    /* [AFINAR] */
#define PID_KI                   1.0f    /* [AFINAR] */
#define PID_INTEG_MAX            0.3f
#define V_OBJ_MINIMA_MS          0.01f   /* por debajo de esto, la rueda se considera parada */

/* ---- Seguridad: rueda atascada o encoder sin señal ---- */
#define STALL_PWM_MIN            0.5f
#define STALL_VEL_MAX_MS         0.02f
#define STALL_TIEMPO_S           1.0f
#define FALLO_REARME_S           0.5f    /* tiempo con comando nulo para limpiar el fallo */

#endif /* ROBOT_PARAMS_H */