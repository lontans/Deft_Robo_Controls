#pragma once
#include "host/host_exchange_schema.h"

/* Plant HBHF — actuators/servos/leds; pdb = power-board mirror only. */
void plant_feedback_image_fetch_plant(host_feedback_image_t *out);

/* DEBUG DBGF — fill pdb[0..31] mailbox from CFG/diag/thermo state (legacy). */
void plant_feedback_image_fetch_debug_mailbox(host_pdu_feedback_t *pdu);

/* DEBUG DBGF debug_lanes — stamp DL\x01 header + reply lanes; also fills legacy
 * mailbox for dual-path migration. */
void plant_feedback_image_fetch_debug_lanes(host_feedback_image_t *out);
