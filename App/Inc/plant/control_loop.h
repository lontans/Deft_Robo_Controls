#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "app.h"

extern volatile uint32_t g_control_tick_count;

void control_loop_init(void);
void control_loop_start(void);
void control_loop_service(void);
void control_loop_tick(void);
uint8_t control_loop_pending_get(void);

#if USE_FREERTOS_SCHEDULER
/* Set from MX_FREERTOS_Init after PlantTask create; TIM6 ISR notifies this. */
void control_loop_set_plant_task(void *task_handle);
#endif
