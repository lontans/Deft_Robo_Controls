#include "plant/plugins/dynamixel.h"
#include "usart.h"
#include "main.h"

static uint8_t g_dbg_init_hal_st;
static uint8_t g_dbg_tx_hal_st;
static uint8_t g_dbg_gstate_before_tx;
static uint8_t g_dbg_lock_before_tx;

uint8_t dxl_port_debug_init_hal_st(void) { return g_dbg_init_hal_st; }
uint8_t dxl_port_debug_tx_hal_st(void)   { return g_dbg_tx_hal_st; }
uint8_t dxl_port_debug_gstate(void)      { return g_dbg_gstate_before_tx; }
uint8_t dxl_port_debug_lock(void)        { return g_dbg_lock_before_tx; }

static void dxl_dwt_init(void)
{
	static bool ready;

	if (ready)
		return;

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	ready = true;
}

static void dxl_port_delay_us(uint32_t us)
{
	uint32_t start;
	uint32_t ticks;

	if (us == 0u)
		return;

	dxl_dwt_init();
	ticks = (SystemCoreClock / 1000000u) * us;
	start = DWT->CYCCNT;

	while ((DWT->CYCCNT - start) < ticks) {
	}
}

static void dxl_bus_set_rx(void)
{
	DXL_GPIO_EN_PORT->BSRR = (uint32_t)DXL_GPIO_EN_PIN << 16;
}

static void dxl_bus_set_tx(void)
{
	DXL_GPIO_EN_PORT->BSRR = DXL_GPIO_EN_PIN;
}

static void dxl_bus_set_listen(void)
{
	dxl_bus_set_rx();
}

static void dxl_bus_set_transmit(void)
{
	dxl_bus_set_tx();
	dxl_port_delay_us(DXL_PRE_TX_SETTLE_US);
}

static void dxl_port_drain_rx(void)
{
	uint8_t dump;

	while (HAL_UART_Receive(&huart5, &dump, 1, 0) == HAL_OK) {
	}
}

static void dxl_bus_after_transmit(void)
{
	uint32_t baud;
	uint32_t release_us;
	uint32_t turnaround_us;

	/* HAL_UART_Transmit already waited for TC.  Hold TX gate one bit-period
	 * so the stop bit fully clears the wire before releasing the bus. */
	baud = huart5.Init.BaudRate;
	if (baud == 0u)
		baud = DXL_BAUD_RATE;

	release_us = (10000000u / baud) + DXL_TURNAROUND_MARGIN_US;
	if (release_us < DXL_BUS_RELEASE_MIN_US)
		release_us = DXL_BUS_RELEASE_MIN_US;

	dxl_port_delay_us(release_us);

	/* Switch to RX; servo's return-delay window starts from TC. */
	dxl_bus_set_rx();

	turnaround_us = (10000000u / baud) + DXL_TURNAROUND_MARGIN_US;
	if (turnaround_us < DXL_TURNAROUND_MIN_US)
		turnaround_us = DXL_TURNAROUND_MIN_US;

	dxl_port_delay_us(turnaround_us);
	dxl_port_drain_rx();
}

void dxl_port_init(void)
{
	dxl_bus_set_listen();
	dxl_port_flush_rx();
}

void dxl_port_flush_rx(void)
{
	dxl_bus_set_listen();
	dxl_port_drain_rx();
}

void dxl_port_delay_ms(uint32_t ms)
{
	HAL_Delay(ms);
}

static void dxl_uart_recover(void)
{
	(void)HAL_UART_Abort(&huart5);
	dxl_port_drain_rx();
}

bool dxl_port_set_baud(uint32_t baud)
{
	if (huart5.gState != HAL_UART_STATE_READY &&
	    huart5.gState != HAL_UART_STATE_RESET) {
		dxl_uart_recover();
	}

	huart5.Init.BaudRate = baud;
	g_dbg_init_hal_st = (uint8_t)HAL_UART_Init(&huart5);
	if (g_dbg_init_hal_st != HAL_OK)
		return false;

	dxl_bus_set_listen();
	dxl_port_delay_ms(DXL_BAUD_SETTLE_MS);
	return true;
}

int dxl_port_write(const uint8_t *buf, int len)
{
	HAL_StatusTypeDef st;

	if (buf == NULL || len <= 0)
		return 0;

	if (huart5.gState != HAL_UART_STATE_READY) {
		dxl_uart_recover();
	}

	g_dbg_gstate_before_tx = (uint8_t)huart5.gState;
	g_dbg_lock_before_tx   = (uint8_t)huart5.Lock;

	dxl_bus_set_transmit();

	st = HAL_UART_Transmit(&huart5, (uint8_t *)buf, (uint16_t)len,
	                       DXL_TX_TIMEOUT_MS);
	g_dbg_tx_hal_st = (uint8_t)st;
	if (st != HAL_OK) {
		dxl_bus_set_listen();
		return 0;
	}

	dxl_bus_after_transmit();
	return len;
}

int dxl_port_read_byte(uint8_t *byte, uint32_t timeout_ms)
{
	if (byte == NULL)
		return 0;

	dxl_bus_set_listen();

	if (HAL_UART_Receive(&huart5, byte, 1, timeout_ms) == HAL_OK)
		return 1;

	return 0;
}

uint32_t dxl_port_millis(void)
{
	return HAL_GetTick();
}
