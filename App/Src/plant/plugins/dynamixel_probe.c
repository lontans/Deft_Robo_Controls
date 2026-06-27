#include "plant/plugins/dynamixel.h"
#include "plant/plant_diag.h"
#include "host/host_exchange_schema.h"
#include "main.h"
#include <string.h>

static dxl_probe_result_t g_dxl_probe;
static bool g_dxl_probe_valid;
static uint8_t g_dbg_init_st;
static uint8_t g_dbg_tx_st;
static uint8_t g_dbg_gstate;
static uint8_t g_dbg_lock;

void dynamixel_probe_run(uint8_t kind, uint8_t target_id,
                         uint8_t id_start, uint8_t id_end)
{
	uint32_t baud = 0;

	memset(&g_dxl_probe, 0, sizeof(g_dxl_probe));
	g_dxl_probe.probe_kind = kind;
	g_dxl_probe_valid = false;
	g_dbg_init_st = 0;
	g_dbg_tx_st   = 0;
	g_dbg_gstate  = 0;
	g_dbg_lock    = 0;

#if DXL_PROBE_EN_CANARY_MS > 0u
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_Delay(DXL_PROBE_EN_CANARY_MS);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_Delay(10);
#endif

	dxl_port_init();

	if (kind == PLANT_DXL_PROBE_TOGGLE_BAUD) {
		uint32_t new_baud = 0u;

		if (!dxl_toggle_ids_baud(id_start, id_end, &new_baud)) {
			g_dxl_probe.status = 2u;
			g_dxl_probe.baud_rate = 0u;
			g_dxl_probe_valid = true;
			return;
		}
		g_dxl_probe.baud_rate = new_baud;
		g_dxl_probe.status = 0u;
		g_dxl_probe.count = 0u;
		g_dxl_probe_valid = true;
		return;
	}

	if (!dxl_find_baud(&baud, id_start, id_end)) {
		g_dbg_init_st = dxl_port_debug_init_hal_st();
		g_dbg_tx_st   = dxl_port_debug_tx_hal_st();
		g_dbg_gstate  = dxl_port_debug_gstate();
		g_dbg_lock    = dxl_port_debug_lock();
		g_dxl_probe.status = 1u;
		g_dxl_probe_valid = true;
		return;
	}

	g_dxl_probe.baud_rate = baud;

	switch (kind) {
	case PLANT_DXL_PROBE_FIND_BAUD:
		g_dxl_probe.status = 0u;
		g_dxl_probe_valid = true;
		return;

	case PLANT_DXL_PROBE_PING:
		if (target_id == 0u) {
			g_dxl_probe.status = 2u;
			g_dxl_probe_valid = true;
			return;
		}
		if (!dxl_ping(target_id)) {
			g_dxl_probe.status = 2u;
			g_dxl_probe_valid = true;
			return;
		}
		g_dxl_probe.hits[0].id = target_id;
		g_dxl_probe.hits[0].model_number = dxl_ping_model_number(target_id);
		g_dxl_probe.count = 1u;
		g_dxl_probe.status = 0u;
		g_dxl_probe_valid = true;
		return;
	case PLANT_DXL_PROBE_TOGGLE_BAUD:
		return;

	case PLANT_DXL_PROBE_SCAN:
	default:
		g_dxl_probe.count = dxl_scan_ids(id_start, id_end,
		                                 g_dxl_probe.hits,
		                                 DXL_SCAN_RESULT_MAX);
		g_dxl_probe.status = (g_dxl_probe.count > 0u) ? 0u : 2u;
		g_dxl_probe_valid = true;
		return;
	}
}

bool dynamixel_probe_feedback_valid(void)
{
	return g_dxl_probe_valid;
}

void dynamixel_probe_feedback_fill(host_pdu_feedback_t *pdu)
{
	uint8_t i;

	if (pdu == NULL || !g_dxl_probe_valid)
		return;

	memset(pdu->data, 0, sizeof(pdu->data));
	pdu->data[0] = (uint8_t)'d';
	pdu->data[1] = g_dxl_probe.status;
	pdu->data[2] = g_dxl_probe.probe_kind;
	pdu->data[3] = g_dxl_probe.count;

	pdu->data[4] = (uint8_t)(g_dxl_probe.baud_rate);
	pdu->data[5] = (uint8_t)(g_dxl_probe.baud_rate >> 8);
	pdu->data[6] = (uint8_t)(g_dxl_probe.baud_rate >> 16);
	pdu->data[7] = (uint8_t)(g_dxl_probe.baud_rate >> 24);

	/* Debug bytes (only meaningful when status != 0, count == 0):
	 * [8]  HAL_UART_Init return (0=OK 1=ERR 2=BUSY 3=TIMEOUT)
	 * [9]  gState before HAL_UART_Transmit (0x20=READY expected)
	 * [10] HAL_UART_Transmit return
	 * [11] Lock before HAL_UART_Transmit (0=UNLOCKED 1=LOCKED) */
	pdu->data[8]  = g_dbg_init_st;
	pdu->data[9]  = g_dbg_gstate;
	pdu->data[10] = g_dbg_tx_st;
	pdu->data[11] = g_dbg_lock;

	for (i = 0; i < g_dxl_probe.count && i < DXL_SCAN_RESULT_MAX; i++) {
		uint8_t off = (uint8_t)(8u + i * 3u);

		if ((uint16_t)off + 3u > sizeof(pdu->data))
			break;

		pdu->data[off + 0] = g_dxl_probe.hits[i].id;
		pdu->data[off + 1] = (uint8_t)g_dxl_probe.hits[i].model_number;
		pdu->data[off + 2] = (uint8_t)(g_dxl_probe.hits[i].model_number >> 8);
	}
}
