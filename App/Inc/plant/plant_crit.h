#pragma once

#include "app.h"

#if USE_FREERTOS_SCHEDULER
#include "FreeRTOS.h"
#include "task.h"
#define plant_crit_enter() taskENTER_CRITICAL()
#define plant_crit_exit()  taskEXIT_CRITICAL()
#else
#include "main.h"
#define plant_crit_enter() __disable_irq()
#define plant_crit_exit()  __enable_irq()
#endif
