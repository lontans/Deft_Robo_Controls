/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications (CMSIS-RTOS v2)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app.h"
#include "plant/control_loop.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask (Host) */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  /* Same band as Plant: time-slice keeps USB RX/ack alive under plant load. */
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 1024 * 4
};
/* Definitions for PlantTask */
osThreadId_t plantTaskHandle;
const osThreadAttr_t plantTask_attributes = {
  .name = "PlantTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 1024 * 4
};
/* Definitions for PeripheralTask */
osThreadId_t peripheralTaskHandle;
const osThreadAttr_t peripheralTask_attributes = {
  .name = "PeripheralTask",
  /* Same band as Host/Plant: BelowNormal starved under ×25 plant load so
   * DXL/LED never ran. dxl_port_bus_lock() covers polled UART critical. */
  .priority = (osPriority_t) osPriorityAboveNormal,
  /* DXL/LED paths are deep; 1024–1536 words overflowed into Error_Handler. */
  .stack_size = 2048 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void PlantTask(void *argument);
void PeripheralTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if (defaultTaskHandle == NULL)
    Error_Handler();

  plantTaskHandle = osThreadNew(PlantTask, NULL, &plantTask_attributes);
  if (plantTaskHandle == NULL)
    Error_Handler();

  peripheralTaskHandle = osThreadNew(PeripheralTask, NULL, &peripheralTask_attributes);
  if (peripheralTaskHandle == NULL)
    Error_Handler();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread (Host).
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;
  /* MX_USB_Device_Init() runs in main() before osKernelStart. */

  for (;;) {
    app_host_service();
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void PlantTask(void *argument)
{
  (void)argument;

  /* Register after scheduler is running — avoid FromISR between
   * MX_FREERTOS_Init and osKernelStart. */
  control_loop_set_plant_task(xTaskGetCurrentTaskHandle());

  /* Wake on TIM6 notify — no osDelay so Plant can preempt Peripheral. */
  for (;;) {
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    app_plant_service();
  }
}

void PeripheralTask(void *argument)
{
  (void)argument;

  for (;;) {
    app_peripheral_service();
    osDelay(1);
  }
}

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

/* USER CODE END Application */
