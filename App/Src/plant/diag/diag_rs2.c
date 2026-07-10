#include "plant/diag/diag.h"
#include "plant/diag/diag_internal.h"
#include "plant/actuator.h"
#include "plant/can/can_router.h"
#include "plant/can/mcp2518fd.h"
#include "plant/plugins/robstride.h"
#include "main.h"
#include <string.h>

static bool pdu_is_scan_request(const host_pdu_command_t *pdu)
{
	if (pdu == NULL)
		return false;

	return pdu->data[0] == (uint8_t)PLANT_DIAG_PDU_TAG0 &&
	       pdu->data[1] == (uint8_t)PLANT_DIAG_PDU_TAG1 &&
	       pdu->data[2] == (uint8_t)PLANT_DIAG_PDU_TAG2;
}

static void diag_reset_motor(uint8_t motor_id, can_bus_id_t bus)
{
	robstride_probe_result_t tmp;

	if (motor_id == 0u)
		return;

	(void)robstride_probe_id(bus, motor_id, PLANT_DIAG_PROBE_RESET, NULL, 0u, 0u, &tmp);
}

static void diag_queue_probe(const host_command_image_t *cmd,
                             uint8_t motor_id,
                             uint8_t kind,
                             can_bus_id_t bus)
{
	g_rs2_pending_kind = kind;
	g_rs2_pending_motor_id = motor_id;
	g_rs2_pending_bus = bus;
	g_rs2_pending_param_index = (uint16_t)cmd->pdu.data[5] |
	                            ((uint16_t)cmd->pdu.data[6] << 8);
	g_rs2_pending_param_raw = (uint32_t)cmd->pdu.data[7] |
	                          ((uint32_t)cmd->pdu.data[8] << 8) |
	                          ((uint32_t)cmd->pdu.data[9] << 16) |
	                          ((uint32_t)cmd->pdu.data[10] << 24);
	if (rs02_probe_kind_mounts_desire(kind)) {
		g_rs2_pending_desire = cmd->actuator_commands[0];
		const actuator_desire_t *d = &cmd->actuator_commands[0];
		g_rs2_pending_has_desire =
			!(d->position == 0.0f && d->velocity == 0.0f && d->kp == 0.0f &&
			  d->kd == 0.0f && d->torque == 0.0f);
	} else {
		g_rs2_pending_has_desire = false;
	}

	if (kind != PLANT_DIAG_PROBE_CTRL_FAST) {
		memset(&g_last_probe, 0, sizeof(g_last_probe));
		g_last_probe.probe_kind = kind;
		g_last_probe.motor_id = motor_id;
	} else {
		g_last_probe.probe_kind = kind;
		g_last_probe.motor_id = motor_id;
	}

	g_probe_in_progress = true;
	g_rs2_probe_pending = true;
	g_probe_started_ms = HAL_GetTick();
}

bool plant_diag_is_rs2_command(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return false;

	return pdu_is_scan_request(&cmd->pdu);
}

void plant_diag_on_command(const host_command_image_t *cmd)
{
	if (cmd == NULL || !pdu_is_scan_request(&cmd->pdu))
		return;

	uint8_t motor_id = cmd->pdu.data[3];
	uint8_t kind = cmd->pdu.data[4];
	can_bus_id_t bus = diag_pdu_can_bus(&cmd->pdu);

	if (kind == PLANT_DIAG_SESSION_BEGIN) {
		g_rs2_session_active = true;
		g_rs2_quiet_until_ms = 0u;
		if (motor_id != 0u)
			g_rs2_motor_id = motor_id;
		g_rs2_can_bus = bus;
		can_router_discard_pending_tx();
		if (bus < CAN_BUS_CH4)
			can_router_restart_fdcan(bus);
		for (uint8_t b = 0; b < (uint8_t)CAN_BUS_CH4; b++)
			can_rx_drain((can_bus_id_t)b);
		if (bus >= CAN_BUS_CH4) {
			uint8_t rail = (uint8_t)(bus - CAN_BUS_CH4);
			if ((mcp2518_init_mask() & (1u << rail)) != 0u) {
				mcp2518_prepare_tx(bus);
			} else {
				(void)mcp2518_reinit_rail(bus);
			}
			mcp2518_reset_tx_stats(bus);
		}
		actuator_desire_clear();
		memset(&g_last_probe, 0, sizeof(g_last_probe));
		g_last_probe.probe_kind = kind;
		g_last_probe.found = true;
		diag_flush_usb();
		return;
	}

	if (kind == PLANT_DIAG_SESSION_END) {
		uint8_t last_kind = g_last_probe.probe_kind;

		g_rs2_session_active = false;
		g_rs2_quiet_until_ms = HAL_GetTick() + PLANT_DIAG_RS2_QUIET_MS;
		/* Plant teleop uses ENABLE_ONLY then SESSION_END — do not reset the drive
		 * we just armed; plant maintain_enable + MIT stream keeps it running. */
		if (g_rs2_can_bus < CAN_BUS_CH4 &&
		    last_kind != PLANT_DIAG_PROBE_ENABLE_ONLY)
			diag_reset_motor(g_rs2_motor_id, g_rs2_can_bus);
		actuator_desire_clear();
		memset(&g_last_probe, 0, sizeof(g_last_probe));
		g_last_probe.probe_kind = kind;
		g_last_probe.found = true;
		diag_flush_usb();
		return;
	}

	if (motor_id != 0u)
		g_rs2_motor_id = motor_id;
	g_rs2_can_bus = bus;

	if (kind == PLANT_DIAG_PROBE_MCP_SMOKE || kind == PLANT_DIAG_PROBE_MCP_WAKE ||
	    kind == PLANT_DIAG_PROBE_MCP_DISABLE) {
		memset(&g_last_mcp_smoke, 0, sizeof(g_last_mcp_smoke));
		memset(&g_last_probe, 0, sizeof(g_last_probe));
		g_last_probe.probe_kind = kind;
		g_last_probe.motor_id = motor_id;
		g_rs2_can_bus = bus;
		diag_queue_probe(cmd, motor_id, kind, bus);
		return;
	}

	diag_queue_probe(cmd, motor_id, kind, bus);
}

void diag_run_rs2_pending(void)
{
	if (!g_rs2_probe_pending)
		return;

	g_rs2_probe_pending = false;

	uint8_t kind = g_rs2_pending_kind;
	uint8_t motor_id = g_rs2_pending_motor_id;
	can_bus_id_t bus = g_rs2_pending_bus;

	if (kind == PLANT_DIAG_PROBE_MCP_SMOKE) {
		diag_mcp_smoke_sync(motor_id, bus);
		g_probe_in_progress = false;
		diag_flush_usb();
		return;
	}

	if (kind == PLANT_DIAG_PROBE_MCP_WAKE) {
		diag_mcp_wake_sync(motor_id, bus);
		g_probe_in_progress = false;
		diag_flush_usb();
		return;
	}

	if (kind == PLANT_DIAG_PROBE_MCP_DISABLE) {
		diag_mcp_disable_sync(motor_id, bus);
		g_probe_in_progress = false;
		diag_flush_usb();
		return;
	}

	if (bus >= CAN_BUS_CH4)
		mcp2518_prepare_tx(bus);

	const actuator_desire_t *desire = g_rs2_pending_has_desire ?
	                                  &g_rs2_pending_desire : NULL;
	bool got = robstride_probe_id(bus, motor_id, kind, desire,
	                              g_rs2_pending_param_index,
	                              g_rs2_pending_param_raw,
	                              &g_last_probe);
	diag_finalize_probe(kind, motor_id, got);

	g_probe_in_progress = false;
	g_probe_started_ms = 0u;
	diag_flush_usb();

	/* Drop stale bench PDU so plant runtime feedback is not stuck on tag 'r'. */
	if (kind != PLANT_DIAG_SESSION_BEGIN && kind != PLANT_DIAG_SESSION_END)
		memset(&g_last_probe, 0, sizeof(g_last_probe));
}
