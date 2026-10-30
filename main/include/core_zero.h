#ifndef CORE_ZERO_H
#define CORE_ZERO_H

#include <stddef.h>

#include "shared_data.h"

/* Comunicaciones micro-ROS (corre en el Core 0). */
void vTaskCore0_MicroROS(void *pvParameters);

void iniciar_wifi(void);
void publicar_telemetria(const telemetria_t *snapshot);

#endif /* CORE_ZERO_H */