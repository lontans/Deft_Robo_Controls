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
