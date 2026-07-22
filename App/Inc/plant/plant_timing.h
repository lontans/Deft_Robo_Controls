#pragma once

#include <stdint.h>
#include "host/host_exchange_schema.h"

/* Superloop / plant timing — primary: system feedback (layout v2).
 * SVD/thermo PDU overlays kept as no-ops for API stability. */
void plant_timing_lap_begin(void);
void plant_timing_lap_end(void);
void plant_timing_note_pending_at_lap(uint8_t pending);
void plant_timing_note_service(uint8_t ticks_serviced);
void plant_timing_system_fill(host_system_feedback_t *sys);
void plant_timing_svd_fill(host_pdu_feedback_t *pdu);
void plant_timing_thermo_fill(host_pdu_feedback_t *pdu);
