// app.h

#pragma once

/* 0 = pre-RTOS superloop (app_run in main) — host link + control_loop_service.
 * 1 = osKernelStart(); app_run in StartDefaultTask, plant loop in ControlTask. */
#ifndef USE_FREERTOS_SCHEDULER
#define USE_FREERTOS_SCHEDULER 0
#endif

void app_init(void);
void app_run(void);
