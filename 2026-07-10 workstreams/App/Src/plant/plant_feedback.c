#include "plant/plant_feedback.h"
#include "plant/actuator.h"
#include "plant/servo.h"
#include "plant/led.h"
#include "plant/plant_diag.h"
#include "plant/plant_config_nvm.h"
#include "plant/thermo.h"
#include "host/host_uart_bridge.h"

void plant_feedback_image_fetch(host_feedback_image_t *out)
{
	if (out == NULL)
		return;

	actuator_capture_state();
	actuator_feedback_snapshot(out->actuator_feedback, HOST_EXCHANGE_ACTUATOR_SLOTS);
	servo_feedback_snapshot(out->servos, HOST_EXCHANGE_SERVO_SLOTS);
	led_feedback_snapshot(&out->leds[0]);
	plant_diag_feedback_fill(&out->pdu);
	/* UART4 bridge (host/uart4_mode.h) takes priority when built in; no-op otherwise. */
	host_uart_bridge_feedback_fill(&out->pdu);
	plant_config_feedback_fill(&out->pdu);
	/* 2026-07-10 workstreams: mirror the latest thermocouple reading iff the
	 * last command tagged a TMP probe (no-op otherwise). Must run before the
	 * whitelist check below, same as the other *_feedback_fill calls. */
	thermo_feedback_fill(&out->pdu);
	/* Keep RS2 ('r'), Dynamixel ('d'), UART bridge ('u'), CFG ('c'), and
	 * thermo ('t') PDUs — 2026-07-10 workstreams added PLANT_THERMO_RESP_TAG
	 * to this whitelist; without it servo_diag_feedback_fill() below
	 * silently stomps pdu.data[0] on every cycle a thermo probe wasn't the
	 * only thing tagged, which would make the reading disappear. */
	if (out->pdu.data[0] != (uint8_t)'d' &&
	    out->pdu.data[0] != (uint8_t)'u' &&
	    out->pdu.data[0] != (uint8_t)PLANT_CFG_PDU_RESP_TAG0 &&
	    out->pdu.data[0] != (uint8_t)PLANT_DIAG_DM_RESP_TAG &&
	    out->pdu.data[0] != (uint8_t)PLANT_DIAG_PDU_RESP_TAG &&
	    out->pdu.data[0] != (uint8_t)PLANT_THERMO_RESP_TAG) {
		servo_diag_feedback_fill(&out->pdu);
		plant_diag_feedback_stamp_fw_marker(&out->pdu);
	}
}
