#include "host/host_transport.h"
#include "host/host_transport_uart.h"
#include "usart.h"
#include <string.h>

#define UART_TRANSPORT_RX_RING_SIZE 2048u
#define HOST_IMAGE_BYTES            562u

static uint8_t           rx_ring[UART_TRANSPORT_RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile bool     tx_busy;
static uint8_t           rx_byte;

void host_transport_uart_rx_push(const uint8_t *data, uint32_t len)
{
	if (data == NULL || len == 0)
		return;
	for (uint32_t i = 0; i < len; i++) {
		uint16_t next = (uint16_t)((rx_head + 1u) % UART_TRANSPORT_RX_RING_SIZE);
		if (next == rx_tail)
			break;
		rx_ring[rx_head] = data[i];
		rx_head = next;
	}
}

static void uart_init(void)
{
	rx_head = 0;
	rx_tail = 0;
	tx_busy = false;
	HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
}

static size_t uart_read(uint8_t *dst, size_t max_len)
{
	size_t n = 0;

	if (dst == NULL || max_len == 0)
		return 0;

	while (n < max_len && rx_tail != rx_head) {
		dst[n++] = rx_ring[rx_tail];
		rx_tail = (uint16_t)((rx_tail + 1u) % UART_TRANSPORT_RX_RING_SIZE);
	}

	return n;
}

static bool uart_write(const uint8_t *src, size_t len)
{
	if (src == NULL || len == 0 || len > HOST_IMAGE_BYTES)
		return false;
	if (tx_busy)
		return false;

	tx_busy = true;
	if (HAL_UART_Transmit(&huart4, (uint8_t *)src, (uint16_t)len, 100) != HAL_OK) {
		tx_busy = false;
		return false;
	}
	tx_busy = false;
	return true;
}

static bool uart_tx_ready(void)
{
	return !tx_busy;
}

void host_transport_uart_tx_complete(void)
{
	tx_busy = false;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance != UART4)
		return;

	host_transport_uart_rx_push(&rx_byte, 1);
	HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
}

const host_transport_ops_t host_transport_uart_ops = {
	.init = uart_init,
	.read = uart_read,
	.write = uart_write,
	.tx_ready = uart_tx_ready,
};
