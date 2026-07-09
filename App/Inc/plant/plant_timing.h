#pragma once

#include <stdint.h>
#include "host/host_exchange_schema.h"

/* Superloop / plant timing — packed into SVD PDU bytes 23..28 on feedback. */
void plant_timing_lap_begin(void);
void plant_timing_lap_end(void);
void plant_timing_note_pending_at_lap(uint8_t pending);
void plant_timing_note_service(uint8_t ticks_serviced);
void plant_timing_svd_fill(host_pdu_feedback_t *pdu);
