/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
#include "spi.h"

/* USER CODE BEGIN 0 */
/*
 * SUPERSEDED — do not merge this MX_SPI4_Init()/hspi4 addition. The live
 * tree's thermocouple support rides the existing SPI3 (shared with SK9822
 * via spi3_role.h exclusive arbitration) instead of a dedicated SPI4 — no
 * spi.c/spi.h changes were needed there at all. Only merge the CubeMars
 * pieces of this workstream (cubemars.h/.c, plugin_table.c) — see
 * ../../README.md. Kept below for reference only.
 *
 * 2026-07-10 workstreams: SPI4 added for the MAX31855 K-type thermocouple
 * (MW BA017 breakout). SPI1=MCP2518 CAN (CH4-6), SPI3=SK9822 LEDs — both
 * already claimed, so SPI4 (unconfigured/free in the live tree) was picked
 * as the placeholder target. TODO CONFIRM before merging:
 *   - Is the breakout actually wired to SPI4, or SPI2 (also free)? If SPI2,
 *     swap this whole block to MX_SPI2_Init()/hspi2 and the matching
 *     GPIOx/AFx/pins for SPI2 instead of SPI4.
 *   - THERMO_SPI4_SCK_PIN / THERMO_SPI4_MOSI_PIN below are PLACEHOLDERS,
 *     not verified against the schematic/harness. Replace with the real
 *     pins before this compiles against real hardware.
 * The breakout only exposes SO (no separate MISO), wired to what the
 * harness calls MOSI — hence SPI_DIRECTION_1LINE (half-duplex): the
 * MOSI-designated pin is reused as the receive line (BIDIOE cleared by
 * HAL_SPI_Receive() in 1-line mode), no MISO pin needed at all.
 */
#define THERMO_SPI4_GPIO_PORT   GPIOE                 /* TODO CONFIRM */
#define THERMO_SPI4_SCK_PIN     GPIO_PIN_2             /* TODO CONFIRM */
#define THERMO_SPI4_MOSI_PIN    GPIO_PIN_6             /* TODO CONFIRM — this is the SO line */
#define THERMO_SPI4_AF          GPIO_AF5_SPI4          /* TODO CONFIRM against the chosen pins */
/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}
/* SPI3 init function */
void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/* SPI4 init function — 2026-07-10 workstreams (MAX31855 thermocouple) */
void MX_SPI4_Init(void)
{
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  /* Half-duplex 1-line: MOSI-designated pin doubles as the receive line
   * (MAX31855 SO), since MISO isn't broken out on this breakout. */
  hspi4.Init.Direction = SPI_DIRECTION_1LINE;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;   /* HAL has no 32-bit frame mode; read 4 bytes, reassemble MSB-first in max31855.c */
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;   /* MAX31855: SPI Mode 0 (TODO CONFIRM against the MW BA017's exact datasheet page) */
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;               /* software CS, same convention as spi_can_port.c */
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;  /* conservative; MAX31855 max SCK ~5 MHz, tighten once confirmed */
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
  else if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspInit 0 */

  /* USER CODE END SPI3_MspInit 0 */
    /* SPI3 clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI3 GPIO Configuration
    PB3     ------> SPI3_SCK
    PB4     ------> SPI3_MISO
    PB5     ------> SPI3_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI3_MspInit 1 */

  /* USER CODE END SPI3_MspInit 1 */
  }
  else if(spiHandle->Instance==SPI4)
  {
    /* 2026-07-10 workstreams — TODO CONFIRM all pins/AF before merging. */
    __HAL_RCC_SPI4_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();   /* TODO CONFIRM port matches THERMO_SPI4_GPIO_PORT above */

    /* SCK: alternate-function push-pull, same style as SPI1/SPI3. */
    GPIO_InitStruct.Pin = THERMO_SPI4_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = THERMO_SPI4_AF;
    HAL_GPIO_Init(THERMO_SPI4_GPIO_PORT, &GPIO_InitStruct);

    /* MOSI-designated pin, reused as the receive line in 1-line mode.
     * Still an AF pin (not a plain GPIO input) — HAL's 1-line RX drives
     * this through the SPI peripheral, not manual bit-banging. */
    GPIO_InitStruct.Pin = THERMO_SPI4_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = THERMO_SPI4_AF;
    HAL_GPIO_Init(THERMO_SPI4_GPIO_PORT, &GPIO_InitStruct);

    /* CS is a plain software-driven GPIO output — configured in
     * thermo_init()/max31855.c, not here (matches spi_can_port.c's CS
     * handling, which lives in the CAN driver rather than spi.c). */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
  else if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspDeInit 0 */

  /* USER CODE END SPI3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    /**SPI3 GPIO Configuration
    PB3     ------> SPI3_SCK
    PB4     ------> SPI3_MISO
    PB5     ------> SPI3_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);

  /* USER CODE BEGIN SPI3_MspDeInit 1 */

  /* USER CODE END SPI3_MspDeInit 1 */
  }
  else if(spiHandle->Instance==SPI4)
  {
    /* 2026-07-10 workstreams */
    __HAL_RCC_SPI4_CLK_DISABLE();
    HAL_GPIO_DeInit(THERMO_SPI4_GPIO_PORT, THERMO_SPI4_SCK_PIN|THERMO_SPI4_MOSI_PIN);
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
