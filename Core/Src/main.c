/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app.h"
#include "plant/control_loop.h"
#include "usb_device.h"
#include "host/soft_dfu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CPU_ACTIVITY_PORT            GPIOC
#define CPU_ACTIVITY_PIN             GPIO_PIN_2
#define CPU_BOOT_PULSE_DELAY_LOOPS   4000000u   /* ~visible flash @ 170 MHz PLL */
#define CPU_ACTIVITY_DELAY_LOOPS     4000000u   /* Error_Handler blink period */
#define CPU_ACTIVITY_BOOT_PULSES     3u
/* Bring-up milestone on PC1 (unused LED): HIGH = past USB init, LOW = past app_init. */
#define BRINGUP_DIAG_PORT            GPIOC
#define BRINGUP_DIAG_PIN             GPIO_PIN_1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
static void cpu_activity_delay(uint32_t loops);
static void cpu_activity_boot_pulses(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void cpu_activity_delay(uint32_t loops)
{
	for (volatile uint32_t d = 0; d < loops; d++) {
	}
}

static void cpu_activity_boot_pulses(void)
{
	for (uint32_t n = 0; n < CPU_ACTIVITY_BOOT_PULSES; n++) {
		HAL_GPIO_WritePin(CPU_ACTIVITY_PORT, CPU_ACTIVITY_PIN, GPIO_PIN_SET);
		cpu_activity_delay(CPU_BOOT_PULSE_DELAY_LOOPS);
		HAL_GPIO_WritePin(CPU_ACTIVITY_PORT, CPU_ACTIVITY_PIN, GPIO_PIN_RESET);
		cpu_activity_delay(CPU_BOOT_PULSE_DELAY_LOOPS);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6)
		control_loop_tick();
	else if (htim->Instance == TIM7)
		HAL_IncTick();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* Checks a RAM signature left by a prior soft_dfu_on_command() call and,
   * if present, jumps into the ROM bootloader instead of continuing boot.
   * Must run before HAL_Init()/SystemClock_Config() -- see soft_dfu.h. */
  soft_dfu_check_and_jump();

  /* AN3156: ROM DFU Leave jumps into the app without a system reset, so the
   * USB FS core / IRQs can still be live from the bootloader. Tear that down
   * before HAL/USB re-init or CDC may never re-enumerate. */
  __HAL_RCC_USB_CLK_ENABLE();
  __HAL_RCC_USB_FORCE_RESET();
  __HAL_RCC_USB_RELEASE_RESET();
  NVIC_DisableIRQ(USB_HP_IRQn);
  NVIC_DisableIRQ(USB_LP_IRQn);
  NVIC_DisableIRQ(USBWakeUp_IRQn);
  NVIC_ClearPendingIRQ(USB_HP_IRQn);
  NVIC_ClearPendingIRQ(USB_LP_IRQn);
  NVIC_ClearPendingIRQ(USBWakeUp_IRQn);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  cpu_activity_boot_pulses();

  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_SPI3_Init();
  MX_TIM6_Init();
  control_loop_start();

  /* Match pre-RTOS baseline (14eb426): enumerate USB before blocking app_init(). */
  MX_USB_Device_Init();
  HAL_GPIO_WritePin(BRINGUP_DIAG_PORT, BRINGUP_DIAG_PIN, GPIO_PIN_SET);

  /* USER CODE BEGIN 2 */
  app_init();
  /* USER CODE END 2 */

  HAL_GPIO_WritePin(BRINGUP_DIAG_PORT, BRINGUP_DIAG_PIN, GPIO_PIN_RESET);

#if USE_FREERTOS_SCHEDULER
  MX_FREERTOS_Init();
  HAL_SuspendTick();
  osKernelStart();
  /* not reached */
#endif

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    app_run();
    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV5;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* NOTE: CubeMX's own HAL_TIM_PeriodElapsedCallback (TIM7 -> HAL_IncTick) used
 * to be generated here. Merged into the hand-written TIM6 callback above
 * (USER CODE 0 block) since C doesn't allow two definitions of the same
 * function -- a future full CubeMX regeneration will re-emit this block and
 * cause the same redefinition error again; re-merge it the same way. */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
    cpu_activity_delay(CPU_ACTIVITY_DELAY_LOOPS);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
