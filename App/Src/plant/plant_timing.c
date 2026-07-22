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

void plant_timing_system_fill(host_system_feedback_t *sys)
{
	if (sys == NULL)
		return;
	sys->lap_ms = s_lap_delta_ms;
	sys->lap_max_ms = s_lap_max_ms;
	sys->ticks_svc = s_ticks_serviced_lap;
	sys->ticks_pending = s_pending_at_lap_start;
}

void plant_timing_svd_fill(host_pdu_feedback_t *pdu)
{
	(void)pdu;
	/* Timing lives in system[] (v2). Do not cannibalize DEBUG mailbox. */
}

void plant_timing_thermo_fill(host_pdu_feedback_t *pdu)
{
	(void)pdu;
}
