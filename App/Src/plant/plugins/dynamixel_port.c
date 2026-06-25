#include "usart.h"

void dx1_port_flush_rx(void) // Must call at the beginning of each transaction to prevent stale bytes
{
	uint8_t dump;
	while (HAL_UART_RECEIVE(&huart, &dump, 1,0) == HAL_OK){}
}

int dx1_port_write(const uint8_t *buf, int len)
{
	if (HAL_UART_TRANSMIT(&huart5, (uint8_t *)buf, (uint16_t)len, DXL_TX_TIMEOUT_MS) != HAL_OK)
	{
		return 0;
	}
	return len;
}


void dx1_port_init(void);
int  dx1_port_write(const uint8_t *buf, int len);
int  dx1_port_read (uint8_t *buf, int len, uint32_t timeout_ms);
uint32_t dx1_port_millis(void);
