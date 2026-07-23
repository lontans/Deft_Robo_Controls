#include "plant/plant_feedback.h"
#include "plant/actuator.h"
#include "plant/servo.h"
#include "plant/led.h"
#include "plant/plant_diag.h"
#include "plant/plant_config_nvm.h"
#include "plant/thermo.h"
#include "host/host_uart_bridge.h"
#include "host/pdb_link.h"
#include <string.h>

void plant_feedback_image_fetch_plant(host_feedback_image_t *out)
{
	if (out == NULL)
		return;

	actuator_capture_state();
	actuator_feedback_snapshot(out->actuator_feedback, HOST_EXCHANGE_ACTUATOR_SLOTS);
	servo_feedback_snapshot(out->servos, HOST_EXCHANGE_SERVO_SLOTS);
	led_feedback_snapshot(&out->leds[0]);
	/* Plant path: pdb[] is the PDB power-board mirror (ADR-001) — no DEBUG tags. */
	pdb_link_fill_mirror(out->pdb);
}

void plant_feedback_image_fetch_debug_mailbox(host_pdu_feedback_t *pdu)
{
	if (pdu == NULL)
		return;

	memset(pdu->data, 0, sizeof(pdu->data));
	plant_diag_feedback_fill(pdu);
	host_uart_bridge_feedback_fill(pdu);
	plant_config_feedback_fill(pdu);
	thermo_feedback_fill(pdu);
	if (pdu->data[0] != (uint8_t)'d' &&
	    pdu->data[0] != (uint8_t)'u' &&
	    pdu->data[0] != (uint8_t)PLANT_CFG_PDU_RESP_TAG0 &&
	    pdu->data[0] != (uint8_t)PLANT_DIAG_DM_RESP_TAG &&
	    pdu->data[0] != (uint8_t)PLANT_DIAG_PDU_RESP_TAG &&
	    pdu->data[0] != (uint8_t)PLANT_THERMO_RESP_TAG) {
		servo_diag_feedback_fill(pdu);
		plant_diag_feedback_stamp_fw_marker(pdu);
	}
}
