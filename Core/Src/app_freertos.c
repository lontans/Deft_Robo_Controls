/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "app.h"
#include "plant/control_loop.h"
#include "host/host_link.h"

/* Private variables ---------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
void StartDefaultTask(void const *argument);
void ControlTask(void const *argument);

void MX_FREERTOS_Init(void)
{
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
	if (defaultTaskHandle == NULL)
		Error_Handler();

	osThreadDef(Control, ControlTask, osPriorityAboveNormal, 0, 512);
	if (osThreadCreate(osThread(Control), NULL) == NULL)
		Error_Handler();
}

/* configCHECK_FOR_STACK_OVERFLOW=2 / configUSE_MALLOC_FAILED_HOOK=1
 * (FreeRTOSConfig.h) call these on detection instead of continuing to run
 * with a corrupted stack or a task that silently never got created. Reuse
 * the existing fault convention (main.c Error_Handler(): disable IRQs, spin
 * toggling PC2/PC3 at a distinct rate) rather than inventing a new one --
 * it needs almost no stack itself, which matters here since the stack may
 * already be the thing that's damaged. */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	(void)xTask;
	(void)pcTaskName;
	Error_Handler();
}

void vApplicationMallocFailedHook(void)
{
	Error_Handler();
}

void ControlTask(void const *argument)
{
	(void)argument;

	/* TIM6 + heartbeat run from main() before the scheduler; this task drains
	 * pending ticks then pushes USB feedback (same order as pre-RTOS app_run). */
	for (;;) {
		control_loop_service();
		host_link_poll_tx();
		osDelay(1);
	}
}

void StartDefaultTask(void const *argument)
{
	(void)argument;

	HAL_ResumeTick();

	/* USB now initialized unconditionally from main() (before osKernelStart()),
	 * decoupled from whether the scheduler ever successfully starts a task. */
	for (;;) {
		app_run();
		osDelay(1);
	}
}
