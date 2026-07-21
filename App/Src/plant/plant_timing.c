#include "plant/plant_timing.h"
#include "plant/can/can_router.h"
#include "plant/can/mcp2518fd.h"
#include "main.h"
#include <string.h>

static uint32_t s_lap_start_ms;
static uint16_t s_lap_delta_ms;
static uint16_t s_lap_max_ms;
static uint8_t  s_ticks_serviced_lap;
static uint8_t  s_pending_at_lap_start;

void plant_timing_lap_begin(void)
{
	s_lap_start_ms = HAL_GetTick();
	s_ticks_serviced_lap = 0u;
}

void plant_timing_lap_end(void)
{
	uint32_t now = HAL_GetTick();
	uint32_t delta = now - s_lap_start_ms;

	if (delta > 0xFFFFu)
		delta = 0xFFFFu;
	s_lap_delta_ms = (uint16_t)delta;
	if (s_lap_delta_ms > s_lap_max_ms)
		s_lap_max_ms = s_lap_delta_ms;
}

void plant_timing_note_service(uint8_t ticks_serviced)
{
	s_ticks_serviced_lap = (uint8_t)(s_ticks_serviced_lap + ticks_serviced);
}

void plant_timing_note_pending_at_lap(uint8_t pending)
{
	s_pending_at_lap_start = pending;
}

static void plant_timing_pack6(uint8_t *dst)
{
	dst[0] = (uint8_t)(s_lap_delta_ms & 0xFFu);
	dst[1] = (uint8_t)((s_lap_delta_ms >> 8) & 0xFFu);
	dst[2] = s_ticks_serviced_lap;
	dst[3] = s_pending_at_lap_start;
	dst[4] = (uint8_t)(s_lap_max_ms & 0xFFu);
	dst[5] = (uint8_t)((s_lap_max_ms >> 8) & 0xFFu);
}

void plant_timing_svd_fill(host_pdu_feedback_t *pdu)
{
	if (pdu == NULL)
		return;
	if (pdu->data[0] != (uint8_t)'S' ||
	    pdu->data[1] != (uint8_t)'V' ||
	    pdu->data[2] != (uint8_t)'D')
		return;

	plant_timing_pack6(&pdu->data[23]);
}

void plant_timing_thermo_fill(host_pdu_feedback_t *pdu)
{
	uint8_t mark[CAN_BACKEND_COUNT];
	uint8_t toggle[CAN_BACKEND_COUNT];
	uint8_t try_ok = 0u;
	uint8_t try_busy = 0u;

	if (pdu == NULL)
		return;
	if (pdu->data[0] != (uint8_t)'t')
		return;

	/* Thermo payload uses 0..15; 16..21 carry the same 6 timing bytes as SVD.
	 * 22..31: MCP ACT LED / try_send debug (wrap 255):
	 *   22..24  mark_count[CH4..CH6]
	 *   25..27  toggle_count[CH4..CH6]
	 *   28..30  try_ok[CH4..CH6]
	 *   31      try_busy[CH4] (hold CH4 alone; CH5/6 ≈ stalled Δmark/Δtry_ok)
	 * mark is +2 per successful plant try_send (driver + spi_backend). */
	plant_timing_pack6(&pdu->data[16]);

	can_router_led_debug_counts(mark, toggle);
	pdu->data[22] = mark[CAN_BUS_CH4];
	pdu->data[23] = mark[CAN_BUS_CH5];
	pdu->data[24] = mark[CAN_BUS_CH6];
	pdu->data[25] = toggle[CAN_BUS_CH4];
	pdu->data[26] = toggle[CAN_BUS_CH5];
	pdu->data[27] = toggle[CAN_BUS_CH6];
	for (uint8_t r = 0; r < 3u; r++) {
		mcp2518_get_try_send_counts(r, &try_ok, &try_busy);
		pdu->data[28u + r] = try_ok;
	}
	mcp2518_get_try_send_counts(0u, &try_ok, &try_busy);
	pdu->data[31] = try_busy;
}
