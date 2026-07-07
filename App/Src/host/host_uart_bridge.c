#include "host/host_uart_bridge.h"
#include "host/uart4_mode.h"

#if UART4_MODE == UART4_MODE_DAMIAO_BRIDGE

#include "usart.h"
#include <string.h>

#define UB_TAG0          ((uint8_t)'U')
#define UB_TAG1          ((uint8_t)'B')
#define UB_FB_TAG        ((uint8_t)'u')
#define UB_CMD_MAX_BYTES (HOST_PDU_PAYLOAD_BYTES - 3u)
#define UB_FB_MAX_BYTES  (HOST_PDU_PAYLOAD_BYTES - 2u)
#define UB_RX_RING_SIZE  2048u

static uint8_t           rx_ring[UB_RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static uint8_t           rx_byte;

static void ub_rx_push(uint8_t b)
{
	uint16_t next = (uint16_t)((rx_head + 1u) % UB_RX_RING_SIZE);

	if (next == rx_tail)
		return; /* ring full: drop rather than block the debug console */

	rx_ring[rx_head] = b;
	rx_head = next;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance != UART4)
		return;

	ub_rx_push(rx_byte);
	(void)HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
}

void host_uart_bridge_init(void)
{
	rx_head = 0;
	rx_tail = 0;

	/* MX_UART4_Init() already ran at 115200; reconfigure for the Damiao
	 * debug UART (921600-8N1 per DM-J4310 manual), reusing the same
	 * GPIO/AF setup from HAL_UART_MspInit(). */
	huart4.Init.BaudRate = 921600u;
	(void)HAL_UART_Init(&huart4);

	(void)HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
}

bool host_uart_bridge_is_command(const host_command_image_t *cmd)
{
	return cmd != NULL &&
	       cmd->pdu.data[0] == UB_TAG0 &&
	       cmd->pdu.data[1] == UB_TAG1;
}

void host_uart_bridge_on_command(const host_command_image_t *cmd)
{
	uint8_t len;

	if (cmd == NULL)
		return;

	len = cmd->pdu.data[2];
	if (len > UB_CMD_MAX_BYTES)
		len = UB_CMD_MAX_BYTES;
	if (len == 0u)
		return;

	(void)HAL_UART_Transmit(&huart4, (uint8_t *)&cmd->pdu.data[3], len, 20u);
}

void host_uart_bridge_feedback_fill(host_pdu_feedback_t *pdu)
{
	uint8_t n = 0;

	if (pdu == NULL)
		return;

	while (n < UB_FB_MAX_BYTES && rx_tail != rx_head) {
		pdu->data[2 + n] = rx_ring[rx_tail];
		rx_tail = (uint16_t)((rx_tail + 1u) % UB_RX_RING_SIZE);
		n++;
	}

	if (n == 0u)
		return; /* nothing to report: leave plant_diag/servo feedback untouched */

	pdu->data[0] = UB_FB_TAG;
	pdu->data[1] = n;
}

#else /* UART4_MODE != UART4_MODE_DAMIAO_BRIDGE: UART4 owned elsewhere. */

void host_uart_bridge_init(void) { }

bool host_uart_bridge_is_command(const host_command_image_t *cmd)
{
	(void)cmd;
	return false;
}

void host_uart_bridge_on_command(const host_command_image_t *cmd)
{
	(void)cmd;
}

void host_uart_bridge_feedback_fill(host_pdu_feedback_t *pdu)
{
	(void)pdu;
}

#endif
