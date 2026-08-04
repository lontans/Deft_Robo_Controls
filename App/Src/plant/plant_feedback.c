#include "plant/plant_feedback.h"
#include "plant/actuator.h"
#include "plant/servo.h"
#include "plant/led.h"
#include "plant/plant_command.h"
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
	    pdu->data[0] != (uint8_t)PLANT_DIAG_CM_RESP_TAG &&
	    pdu->data[0] != (uint8_t)PLANT_DIAG_ZE_RESP_TAG &&
	    pdu->data[0] != (uint8_t)PLANT_DIAG_PDU_RESP_TAG &&
	    pdu->data[0] != (uint8_t)PLANT_THERMO_RESP_TAG) {
		servo_diag_feedback_fill(pdu);
		plant_diag_feedback_stamp_fw_marker(pdu);
	}
}

void plant_feedback_image_fetch_debug_lanes(host_feedback_image_t *out)
{
	uint8_t *raw;
	host_debug_lanes_header_t *hdr;
	uint16_t arm;
	uint8_t *lane;
	const uint8_t *mbox;

	if (out == NULL)
		return;

	/* Fill legacy mailbox once (destructive on diag state), then mirror
	 * the reply into the matching debug lane. */
	plant_feedback_image_fetch_debug_mailbox(&out->pdu);
	mbox = out->pdu.data;

	raw = (uint8_t *)out;
	hdr = (host_debug_lanes_header_t *)(raw + HOST_DEBUG_LANES_HDR_OFF);
	arm = plant_command_debug_lanes_arm_mask();

	hdr->tag0 = (uint8_t)HOST_DEBUG_LANES_TAG0;
	hdr->tag1 = (uint8_t)HOST_DEBUG_LANES_TAG1;
	hdr->ver = (uint8_t)HOST_DEBUG_LANES_VER;
	hdr->flags = 0u;
	hdr->arm_mask = arm;

	memset(raw + HOST_DEBUG_LANE0_OFF, 0,
	       (size_t)HOST_DEBUG_LANE_COUNT *
		       (size_t)HOST_DEBUG_LANE_BYTES);

	if (mbox[0] == (uint8_t)PLANT_DIAG_PDU_RESP_TAG) {
		lane = raw + HOST_DEBUG_LANE0_OFF +
		       ((size_t)HOST_DEBUG_LANE_RS *
			(size_t)HOST_DEBUG_LANE_BYTES);
		memcpy(lane, mbox, HOST_PDU_PAYLOAD_BYTES);
	} else if (mbox[0] == (uint8_t)PLANT_DIAG_DM_RESP_TAG) {
		lane = raw + HOST_DEBUG_LANE0_OFF +
		       ((size_t)HOST_DEBUG_LANE_DM *
			(size_t)HOST_DEBUG_LANE_BYTES);
		memcpy(lane, mbox, HOST_PDU_PAYLOAD_BYTES);
	} else if (mbox[0] == (uint8_t)PLANT_DIAG_CM_RESP_TAG) {
		lane = raw + HOST_DEBUG_LANE0_OFF +
		       ((size_t)HOST_DEBUG_LANE_CM *
			(size_t)HOST_DEBUG_LANE_BYTES);
		memcpy(lane, mbox, HOST_PDU_PAYLOAD_BYTES);
	} else if (mbox[0] == (uint8_t)PLANT_DIAG_ZE_RESP_TAG) {
		lane = raw + HOST_DEBUG_LANE0_OFF +
		       ((size_t)HOST_DEBUG_LANE_ZE *
			(size_t)HOST_DEBUG_LANE_BYTES);
		memcpy(lane, mbox, HOST_PDU_PAYLOAD_BYTES);
	} else if (mbox[0] == (uint8_t)PLANT_CFG_PDU_RESP_TAG0) {
		lane = raw + HOST_DEBUG_LANE0_OFF +
		       ((size_t)HOST_DEBUG_LANE_CFG *
			(size_t)HOST_DEBUG_LANE_BYTES);
		memcpy(lane, mbox, HOST_PDU_PAYLOAD_BYTES);
	} else if (mbox[0] == (uint8_t)'d') {
		lane = raw + HOST_DEBUG_LANE0_OFF +
		       ((size_t)HOST_DEBUG_LANE_SERVO *
			(size_t)HOST_DEBUG_LANE_BYTES);
		memcpy(lane, mbox, HOST_PDU_PAYLOAD_BYTES);
	} else if (mbox[0] == (uint8_t)'u' ||
		   mbox[0] == (uint8_t)PLANT_THERMO_RESP_TAG) {
		lane = raw + HOST_DEBUG_LANE0_OFF +
		       ((size_t)HOST_DEBUG_LANE_PDU *
			(size_t)HOST_DEBUG_LANE_BYTES);
		memcpy(lane, mbox, HOST_PDU_PAYLOAD_BYTES);
	}
}
