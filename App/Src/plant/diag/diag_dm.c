#include "plant/diag/diag.h"
#include "plant/diag/diag_internal.h"
#include "plant/actuator.h"
#include "plant/can/can_router.h"
#include "plant/plugins/damiao.h"
#include "host/host_link.h"
#include "main.h"
#include <string.h>

void diag_dm_clear_actuator_mirror(void)
{
	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		if (!actuator_table[i].enabled)
			continue;
		if (actuator_table[i].protocol != PROTO_DAMIAO)
			continue;

		actuator_state_live[i].fault = 0u;
	}

	actuator_capture_state();
}

void diag_dm_publish_actuator_state(void)
{
	uint8_t slot = ACTUATOR_COUNT;

	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		if (!actuator_table[i].enabled)
			continue;
		if (actuator_table[i].protocol != PROTO_DAMIAO)
			continue;
		if (actuator_table[i].bus != g_dm_pending_bus)
			continue;
		slot = i;
		break;
	}

	if (slot >= ACTUATOR_COUNT) {
		for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
			if (actuator_table[i].enabled &&
			    actuator_table[i].protocol == PROTO_DAMIAO) {
				slot = i;
				break;
			}
		}
	}

	if (slot >= ACTUATOR_COUNT)
		return;

	actuator_state_t *st = &actuator_state_live[slot];

	st->position    = g_last_dm_probe.position;
	st->velocity    = g_last_dm_probe.velocity;
	st->torque      = g_last_dm_probe.torque;
	st->temperature = g_last_dm_probe.temperature;
	if (g_last_dm_probe.probe_kind != 0u) {
		st->velocity    = (float)g_last_dm_probe.motor_id;
		st->torque      = (float)g_last_dm_probe.discovered_id;
		st->temperature = (float)g_last_dm_probe.master_id;
	}
	st->fault = PLANT_DM_FB_MAGIC |
	            ((uint32_t)(g_last_dm_probe.found ? 1u : 0u) << 23) |
	            ((uint32_t)g_last_dm_probe.tx_frames_sent << 16) |
	            ((uint32_t)g_last_dm_probe.raw_frames_seen << 8) |
	            (uint32_t)g_last_dm_probe.err;

	actuator_capture_state();
}

bool plant_diag_is_dm_command(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return false;

	return cmd->pdu.data[0] == (uint8_t)PLANT_DIAG_DM_TAG0 &&
	       cmd->pdu.data[1] == (uint8_t)PLANT_DIAG_DM_TAG1 &&
	       cmd->pdu.data[2] == (uint8_t)PLANT_DIAG_DM_TAG2;
}

void plant_diag_on_dm_command(const host_command_image_t *cmd)
{
	if (cmd == NULL || !plant_diag_is_dm_command(cmd))
		return;

	uint8_t kind = cmd->pdu.data[4];
	can_bus_id_t bus = diag_pdu_can_bus(&cmd->pdu);

	if (kind == PLANT_DIAG_SESSION_BEGIN) {
		g_rs2_probe_pending = false;
		g_probe_in_progress = false;
		g_dm_session_active = true;
		g_dm_can_bus = bus;
		can_router_discard_pending_tx();
		if (bus < CAN_BUS_CH4)
			can_rx_drain(bus);
		actuator_desire_clear();
		diag_dm_clear_actuator_mirror();
		memset(&g_last_dm_probe, 0, sizeof(g_last_dm_probe));
		g_last_dm_probe.probe_kind = kind;
		g_last_dm_probe.motor_id = cmd->pdu.data[3];
		g_last_dm_probe.found = true;
		g_dm_feedback_active = true;
		g_dm_feedback_ttl = 2u;
		diag_flush_usb();
		return;
	}

	if (kind == PLANT_DIAG_SESSION_END) {
		g_rs2_probe_pending = false;
		g_probe_in_progress = false;
		g_dm_session_active = false;
		g_dm_quiet_until_ms = HAL_GetTick() + PLANT_DIAG_DM_QUIET_MS;
		memset(&g_last_dm_probe, 0, sizeof(g_last_dm_probe));
		g_last_dm_probe.probe_kind = kind;
		g_last_dm_probe.motor_id = cmd->pdu.data[3];
		g_last_dm_probe.found = true;
		g_dm_feedback_active = true;
		g_dm_feedback_ttl = 2u;
		diag_flush_usb();
		plant_diag_release_actuator_can();
		return;
	}

	if (g_probe_in_progress)
		return;

	g_dm_pending_motor_id = cmd->pdu.data[3];
	g_dm_pending_kind = kind;
	g_dm_pending_master_id = cmd->pdu.data[PLANT_DIAG_DM_PDU_MASTER_ID];
	g_dm_pending_listen_ms = cmd->pdu.data[PLANT_DIAG_DM_PDU_LISTEN_MS];
	g_dm_pending_param_rid = cmd->pdu.data[PLANT_DIAG_DM_PDU_PARAM_RID];
	if (g_dm_pending_listen_ms == 0u)
		g_dm_pending_listen_ms = 15u;
	g_dm_pending_bus = bus;

	diag_dm_clear_actuator_mirror();
	memset(&g_last_dm_probe, 0, sizeof(g_last_dm_probe));
	g_last_dm_probe.motor_id = g_dm_pending_motor_id;
	g_last_dm_probe.probe_kind = g_dm_pending_kind;
	g_dm_feedback_active = false;
	g_dm_feedback_ttl = 0u;

	g_probe_in_progress = false;

	if (g_dm_pending_kind == DM_PROBE_ID_SWEEP) {
		uint8_t end_id = cmd->pdu.data[PLANT_DIAG_DM_PDU_END_ID];

		if (!damiao_probe_id_range(g_dm_pending_bus,
		                          g_dm_pending_motor_id,
		                          end_id,
		                          g_dm_pending_param_rid,
		                          g_dm_pending_listen_ms,
		                          &g_last_dm_probe))
			g_last_dm_probe.found = false;
	} else if (!damiao_probe_id(g_dm_pending_bus,
	                     g_dm_pending_motor_id,
	                     g_dm_pending_kind,
	                     g_dm_pending_master_id,
	                     g_dm_pending_param_rid,
	                     g_dm_pending_listen_ms,
	                     &g_last_dm_probe))
		g_last_dm_probe.found = false;
	g_probe_in_progress = false;

	diag_dm_publish_actuator_state();
	g_dm_feedback_active = true;
	g_dm_feedback_ttl = 4u;
	diag_flush_usb();
	memset(&g_last_dm_probe, 0, sizeof(g_last_dm_probe));
	g_dm_feedback_active = false;
	g_dm_feedback_ttl = 0u;
}
