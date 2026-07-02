#include "plant/can/spi_can_port.h"
#include "plant/can/mcp2518fd.h"

#include "spi.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

static volatile bool g_spi1_locked;

void spi_can_port_init(void)
{
	/* CS idle high — rail0=PB11, rail1=PB1, rail2=PA4. */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11 | GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	g_spi1_locked = false;
}

void spi_can_port_irq_init(void)
{
	GPIO_InitTypeDef gpio = {0};

	/* MCP INT is open-drain from the chip; no external pull-up on board — use STM32 pull-up. */
	gpio.Mode = GPIO_MODE_IT_FALLING;
	gpio.Pull = GPIO_PULLUP;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;

	gpio.Pin = MCP_INT_RAIL0_PIN;
	HAL_GPIO_Init(MCP_INT_RAIL0_PORT, &gpio);

	gpio.Pin = MCP_INT_RAIL1_PIN;
	HAL_GPIO_Init(MCP_INT_RAIL1_PORT, &gpio);

	gpio.Pin = MCP_INT_RAIL2_PIN;
	HAL_GPIO_Init(MCP_INT_RAIL2_PORT, &gpio);

	HAL_NVIC_SetPriority(EXTI0_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	HAL_NVIC_SetPriority(EXTI3_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

bool spi_can_port_int_active(uint8_t rail)
{
	switch (rail) {
	case 0:
		return HAL_GPIO_ReadPin(MCP_INT_RAIL0_PORT, MCP_INT_RAIL0_PIN) == GPIO_PIN_RESET;
	case 1:
		return HAL_GPIO_ReadPin(MCP_INT_RAIL1_PORT, MCP_INT_RAIL1_PIN) == GPIO_PIN_RESET;
	case 2:
		return HAL_GPIO_ReadPin(MCP_INT_RAIL2_PORT, MCP_INT_RAIL2_PIN) == GPIO_PIN_RESET;
	default:
		return false;
	}
}

static void spi1_lock(void)
{
	while (1) {
		__disable_irq();
		if (!g_spi1_locked) {
			g_spi1_locked = true;
			__enable_irq();
			return;
		}
		__enable_irq();
	}
}

static void spi1_unlock(void)
{
	g_spi1_locked = false;
}

static void cs_assert(uint8_t rail)
{
	switch (rail) {
	case 0:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		break;
	default:
		break;
	}
}

static void cs_deassert(uint8_t rail)
{
	switch (rail) {
	case 0:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		break;
	default:
		break;
	}
}

bool spi_can_port_xfer(uint8_t rail, const uint8_t *tx, uint8_t *rx, uint16_t len)
{
	if (rail >= MCP2518_RAIL_COUNT || tx == NULL || len == 0u)
		return false;

	spi1_lock();
	cs_assert(rail);

	HAL_StatusTypeDef st;
	if (rx != NULL)
		st = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, len, 10u);
	else
		st = HAL_SPI_Transmit(&hspi1, (uint8_t *)tx, len, 10u);

	cs_deassert(rail);
	spi1_unlock();

	return (st == HAL_OK);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin) {
	case MCP_INT_RAIL0_PIN:
		mcp2518_isr_rx_pending(0);
		break;
	case MCP_INT_RAIL1_PIN:
		mcp2518_isr_rx_pending(1);
		break;
	case MCP_INT_RAIL2_PIN:
		mcp2518_isr_rx_pending(2);
		break;
	default:
		break;
	}
}
