#pragma once

#include <stdint.h>
#include "host/host_exchange_schema.h"

/* Superloop / plant timing — packed into SVD PDU bytes 23..28 on feedback.
 * Also overlaid on thermo 't' PDU bytes 16..21 so timing is visible while
 * SPI3_ROLE_THERMO owns the mailbox (otherwise lap_ms is stuck at n/a). */
void plant_timing_lap_begin(void);
void plant_timing_lap_end(void);
void plant_timing_note_pending_at_lap(uint8_t pending);
void plant_timing_note_service(uint8_t ticks_serviced);
void plant_timing_svd_fill(host_pdu_feedback_t *pdu);
void plant_timing_thermo_fill(host_pdu_feedback_t *pdu);
