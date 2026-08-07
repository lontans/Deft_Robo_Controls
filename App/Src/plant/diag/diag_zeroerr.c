#include "plant/diag/diag.h"
#include "plant/diag/diag_internal.h"
#include "plant/actuator.h"
#include "plant/can/can_router.h"
#include "plant/plugins/zeroerr.h"
#include "host/host_link.h"
#include "main.h"
#include <string.h>

/*
 * ZeroErr bench DEBUG RPC — CANopen node discover via SDO-read 0x1018
 * (zeroerr_probe_node[_range]), not MIT-shaped like DM/CM. Never touches
 * zeroerr_apply_cycle's boot FSM or the hot plant path.
 */

static void diag_ze_clear_actuator_mirror(void)
{
	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		if (!actuator_table[i].enabled)
			continue;
		if (actuator_table[i].protocol != PROTO_ZEROERR)
			continue;

		actuator_state_live[i].fault = 0u;
	}

	actuator_capture_state();
}

bool plant_diag_is_ze_command(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return false;

	return cmd->pdu.data[0] == (uint8_t)PLANT_DIAG_ZE_TAG0 &&
	       cmd->pdu.data[1] == (uint8_t)PLANT_DIAG_ZE_TAG1 &&
	       cmd->pdu.data[2] == (uint8_t)PLANT_DIAG_ZE_TAG2;
}

void plant_diag_on_ze_command(const host_command_image_t *cmd)
{
	if (cmd == NULL || !plant_diag_is_ze_command(cmd))
		return;

	uint8_t kind = cmd->pdu.data[4];
	can_bus_id_t bus = diag_pdu_can_bus(&cmd->pdu);

	if (kind == PLANT_DIAG_SESSION_BEGIN) {
		g_ze_session_active = true;
		g_ze_can_bus = bus;
		g_ze_bus_mask = diag_pdu_bus_mask_at(&cmd->pdu, PLANT_DIAG_ZE_PDU_BUS_MASK, bus);
		diag_session_prepare_buses(g_ze_bus_mask, bus);
		actuator_desire_clear();
		diag_ze_clear_actuator_mirror();
		memset(&g_last_ze_probe, 0, sizeof(g_last_ze_probe));
		g_last_ze_probe.node_id = cmd->pdu.data[3];
		g_last_ze_probe.found = true;
		g_ze_feedback_active = true;
		g_ze_feedback_ttl = 2u;
		diag_flush_usb();
		return;
	}

	if (kind == PLANT_DIAG_SESSION_END) {
		g_ze_session_active = false;
		g_ze_bus_mask = 0u;
		g_ze_quiet_until_ms = HAL_GetTick() + PLANT_DIAG_ZE_QUIET_MS;
		memset(&g_last_ze_probe, 0, sizeof(g_last_ze_probe));
		g_last_ze_probe.node_id = cmd->pdu.data[3];
		g_last_ze_probe.found = true;
		g_ze_feedback_active = true;
		g_ze_feedback_ttl = 2u;
		diag_flush_usb();
		plant_diag_release_actuator_can();
		return;
	}

	if (g_probe_in_progress)
		return;

	g_ze_pending_node_id = cmd->pdu.data[3];
	g_ze_pending_kind = kind;
	g_ze_pending_sdo_timeout_ms = cmd->pdu.data[PLANT_DIAG_ZE_PDU_TIMEOUT_MS];
	if (g_ze_pending_sdo_timeout_ms == 0u)
		g_ze_pending_sdo_timeout_ms = 30u;
	g_ze_pending_bus = bus;

	diag_ze_clear_actuator_mirror();
	memset(&g_last_ze_probe, 0, sizeof(g_last_ze_probe));
	g_last_ze_probe.node_id = g_ze_pending_node_id;
	g_ze_feedback_active = false;
	g_ze_feedback_ttl = 0u;

	g_probe_in_progress = true;

	if (kind == PLANT_DIAG_ZE_PROBE_SWEEP) {
		uint8_t end_id = cmd->pdu.data[PLANT_DIAG_ZE_PDU_END_ID];

		(void)zeroerr_probe_node_range(g_ze_pending_bus, g_ze_pending_node_id,
		                               end_id, g_ze_pending_sdo_timeout_ms,
		                               &g_last_ze_probe);
	} else if (kind == PLANT_DIAG_ZE_PROBE_BOOT) {
		/* NMT + PDO1 remap only — no controlword frame is ever sent here,
		 * so this cannot release the brake or move the shaft (see
		 * zeroerr_boot_blocking). Reuses zeroerr_probe_result_t: found =
		 * whether the whole SDO/NMT sequence completed; vendor/product/
		 * revision are not applicable and left zero. */
		bool ok = zeroerr_boot_blocking(g_ze_pending_bus, g_ze_pending_node_id,
		                                g_ze_pending_sdo_timeout_ms);

		g_last_ze_probe.found = ok;
		g_last_ze_probe.node_id = g_ze_pending_node_id;
		g_last_ze_probe.discovered_id = ok ? g_ze_pending_node_id : 0u;
	} else if (kind == PLANT_DIAG_ZE_PROBE_POSITION) {
		/* Plain SDO read of 0x6064 — no enable, no PDO/NMT requirement. */
		float pos_rad = 0.0f;
		bool ok = zeroerr_read_position(g_ze_pending_bus, g_ze_pending_node_id,
		                                &pos_rad, g_ze_pending_sdo_timeout_ms);

		g_last_ze_probe.found = ok;
		g_last_ze_probe.node_id = g_ze_pending_node_id;
		g_last_ze_probe.discovered_id = ok ? g_ze_pending_node_id : 0u;
		g_last_ze_probe.position_valid = ok;
		g_last_ze_probe.position_rad = ok ? pos_rad : 0.0f;
	} else {
		(void)zeroerr_probe_node(g_ze_pending_bus, g_ze_pending_node_id,
		                         g_ze_pending_sdo_timeout_ms, &g_last_ze_probe);
	}
	g_probe_in_progress = false;

	can_rx_drain(g_ze_pending_bus);

	g_ze_feedback_active = true;
	g_ze_feedback_ttl = 12u;
	diag_flush_usb();
	diag_ze_clear_actuator_mirror();
}
