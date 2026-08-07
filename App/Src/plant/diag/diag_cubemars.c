#include "plant/diag/diag.h"
#include "plant/diag/diag_internal.h"
#include "plant/actuator.h"
#include "plant/can/can_router.h"
#include "plant/can/mcp2518fd.h"
#include "plant/plugins/cubemars.h"
#include "host/host_link.h"
#include "main.h"
#include <string.h>

/*
 * CubeMars bench DEBUG RPC — mirrors diag_dm.c's tag/session shape (Damiao-
 * shaped wire), but probes go through cubemars_probe_id[_range] (MIT-frame
 * discover, no register-read scheme). Never touches cubemars_apply_cycle's
 * enable latch or the hot plant path.
 */

static void diag_cm_clear_actuator_mirror(void)
{
	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		if (!actuator_table[i].enabled)
			continue;
		if (actuator_table[i].protocol != PROTO_CUBEMARS)
			continue;

		actuator_state_live[i].fault = 0u;
	}

	actuator_capture_state();
}

bool plant_diag_is_cm_command(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return false;

	return cmd->pdu.data[0] == (uint8_t)PLANT_DIAG_CM_TAG0 &&
	       cmd->pdu.data[1] == (uint8_t)PLANT_DIAG_CM_TAG1 &&
	       cmd->pdu.data[2] == (uint8_t)PLANT_DIAG_CM_TAG2;
}

void plant_diag_on_cm_command(const host_command_image_t *cmd)
{
	if (cmd == NULL || !plant_diag_is_cm_command(cmd))
		return;

	uint8_t kind = cmd->pdu.data[4];
	can_bus_id_t bus = diag_pdu_can_bus(&cmd->pdu);

	if (kind == PLANT_DIAG_SESSION_BEGIN) {
		g_cm_session_active = true;
		g_cm_can_bus = bus;
		g_cm_bus_mask = diag_pdu_bus_mask_at(&cmd->pdu, PLANT_DIAG_CM_PDU_BUS_MASK, bus);
		diag_session_prepare_buses(g_cm_bus_mask, bus);
		actuator_desire_clear();
		diag_cm_clear_actuator_mirror();
		memset(&g_last_cm_probe, 0, sizeof(g_last_cm_probe));
		g_last_cm_probe.probe_kind = kind;
		g_last_cm_probe.motor_id = cmd->pdu.data[3];
		g_last_cm_probe.found = true;
		g_cm_feedback_active = true;
		g_cm_feedback_ttl = 2u;
		diag_flush_usb();
		return;
	}

	if (kind == PLANT_DIAG_SESSION_END) {
		g_cm_session_active = false;
		g_cm_bus_mask = 0u;
		g_cm_quiet_until_ms = HAL_GetTick() + PLANT_DIAG_CM_QUIET_MS;
		memset(&g_last_cm_probe, 0, sizeof(g_last_cm_probe));
		g_last_cm_probe.probe_kind = kind;
		g_last_cm_probe.motor_id = cmd->pdu.data[3];
		g_last_cm_probe.found = true;
		g_cm_feedback_active = true;
		g_cm_feedback_ttl = 2u;
		diag_flush_usb();
		plant_diag_release_actuator_can();
		return;
	}

	if (g_probe_in_progress)
		return;

	g_cm_pending_motor_id = cmd->pdu.data[3];
	g_cm_pending_kind = kind;
	g_cm_pending_listen_ms = cmd->pdu.data[PLANT_DIAG_CM_PDU_LISTEN_MS];
	if (g_cm_pending_listen_ms == 0u)
		g_cm_pending_listen_ms = 15u;
	g_cm_pending_bus = bus;

	diag_cm_clear_actuator_mirror();
	memset(&g_last_cm_probe, 0, sizeof(g_last_cm_probe));
	g_last_cm_probe.motor_id = g_cm_pending_motor_id;
	g_last_cm_probe.probe_kind = g_cm_pending_kind;
	g_cm_feedback_active = false;
	g_cm_feedback_ttl = 0u;

	g_probe_in_progress = true;

	if (kind == CUBEMARS_PROBE_ID_SWEEP) {
		uint8_t end_id = cmd->pdu.data[PLANT_DIAG_CM_PDU_END_ID];

		if (!cubemars_probe_id_range(g_cm_pending_bus, g_cm_pending_motor_id,
		                             end_id, g_cm_pending_listen_ms,
		                             &g_last_cm_probe))
			g_last_cm_probe.found = false;
	} else if (!cubemars_probe_id(g_cm_pending_bus, g_cm_pending_motor_id, kind,
	                              g_cm_pending_listen_ms, &g_last_cm_probe)) {
		g_last_cm_probe.found = false;
	}
	g_probe_in_progress = false;

	can_rx_drain(g_cm_pending_bus);

	/* Same TTL-hold pattern as DM — survives until the host matches the reply
	 * without leaving a sticky fault in actuator_state_live[]. */
	g_cm_feedback_active = true;
	g_cm_feedback_ttl = 12u;
	diag_flush_usb();
	diag_cm_clear_actuator_mirror();
}
