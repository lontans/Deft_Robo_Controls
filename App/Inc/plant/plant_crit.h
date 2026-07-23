#pragma once

#include "app.h"
#include "main.h"

#if USE_FREERTOS_SCHEDULER
#include "FreeRTOS.h"
#include "task.h"

/* taskENTER_CRITICAL before osKernelStart leaves BASEPRI raised forever
 * (uxCriticalNesting starts at 0xaaaaaaaa). Use PRIMASK until scheduler runs. */
static inline void plant_crit_enter(void)
{
	if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
		__disable_irq();
	else
		taskENTER_CRITICAL();
}

static inline void plant_crit_exit(void)
{
	if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
		__enable_irq();
	else
		taskEXIT_CRITICAL();
}
#else
#define plant_crit_enter() __disable_irq()
#define plant_crit_exit()  __enable_irq()
#endif
