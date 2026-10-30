#ifndef ULTRASONIC_DRIVER_H
#define ULTRASONIC_DRIVER_H

#define ULTRASONIC_SLOT_MS 30

/* Lecturas no bloqueantes: devuelven el último valor medido en metros,
 * o INFINITY si no hay eco / está fuera de rango (4 m). */
void ultrasonic_init(void);
float ultrasonic_read_front(void);
float ultrasonic_read_left(void);
float ultrasonic_read_right(void);

#endif /* ULTRASONIC_DRIVER_H */