// app.h

#pragma once

/* 0 = pre-RTOS superloop (app_run in main) — host + plant + peripheral sequential.
 * 1 = osKernelStart(); Host/Plant/Peripheral tasks own their services. */
#ifndef USE_FREERTOS_SCHEDULER
#define USE_FREERTOS_SCHEDULER 1
#endif

void app_init(void);
void app_run(void);

/* Role helpers — bare-metal app_run calls all three; RTOS tasks call one each. */
void app_host_service(void);
void app_plant_service(void);
void app_peripheral_service(void);
