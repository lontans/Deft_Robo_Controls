#pragma once
#include "host/host_exchange_schema.h"

/* Plant HBHF — actuators/servos/leds; pdb left clear for power mirror. */
void plant_feedback_image_fetch_plant(host_feedback_image_t *out);

/* DEBUG DBGF — fill pdb[0..31] mailbox from CFG/diag/thermo state. */
void plant_feedback_image_fetch_debug_mailbox(host_pdu_feedback_t *pdu);
