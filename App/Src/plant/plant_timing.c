#include "plant/plant_timing.h"
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

void plant_timing_svd_fill(host_pdu_feedback_t *pdu)
{
	if (pdu == NULL)
		return;
	if (pdu->data[0] != (uint8_t)'S' ||
	    pdu->data[1] != (uint8_t)'V' ||
	    pdu->data[2] != (uint8_t)'D')
		return;

	pdu->data[23] = (uint8_t)(s_lap_delta_ms & 0xFFu);
	pdu->data[24] = (uint8_t)((s_lap_delta_ms >> 8) & 0xFFu);
	pdu->data[25] = s_ticks_serviced_lap;
	pdu->data[26] = s_pending_at_lap_start;
	pdu->data[27] = (uint8_t)(s_lap_max_ms & 0xFFu);
	pdu->data[28] = (uint8_t)((s_lap_max_ms >> 8) & 0xFFu);
}
