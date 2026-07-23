#pragma once

#include <stdint.h>
#include "host/host_exchange_schema.h"

/* Dual-path timing — published in system feedback (layout v2).
 * act_lap_ms / act_lap_peak_ms / ticks_*: PlantTask (apply+TX).
 * periph_lap_ms / periph_lap_peak_ms: PeripheralTask (DXL/LED/SPI3).
 * *_peak_ms are sticky until plant_timing_reset_peaks(). */
void plant_timing_lap_begin(void);
void plant_timing_lap_end(void);
void plant_timing_periph_lap_begin(void);
void plant_timing_periph_lap_end(void);
void plant_timing_note_pending_at_lap(uint8_t pending);
void plant_timing_note_service(uint8_t ticks_serviced);
void plant_timing_system_fill(host_system_feedback_t *sys);
void plant_timing_svd_fill(host_pdu_feedback_t *pdu);
void plant_timing_thermo_fill(host_pdu_feedback_t *pdu);
/* Clear sticky lap peaks (e.g. after blocking bench/MCP reinit). */
void plant_timing_reset_peaks(void);
