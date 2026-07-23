#include "plant/diag/diag.h"
#include "plant/diag/diag_internal.h"
#include "plant/servo.h"
#include "plant/actuator.h"
#include "plant/can/can_router.h"
#include "plant/plugins/dynamixel.h"
#include "host/host_link.h"
#include "main.h"
#include <string.h>

can_bus_id_t diag_pdu_can_bus(const host_pdu_command_t *pdu)
{
	if (pdu == NULL)
		return CAN_BUS_CH1;

	uint8_t host_bus = pdu->data[PLANT_DIAG_PDU_CAN_BUS];
	if (host_bus >= 1u && host_bus <= (uint8_t)CAN_BACKEND_COUNT)
		return (can_bus_id_t)(host_bus - 1u);

	return CAN_BUS_CH1;
}

void diag_flush_usb(void)
{
	for (uint8_t i = 0; i < 32u; i++) {
		if (g_dm_session_active && g_dm_can_bus < CAN_BACKEND_COUNT)
			can_router_poll_bus_rx(g_dm_can_bus);
		else if (g_rs2_session_active && g_rs2_can_bus < CAN_BACKEND_COUNT)
			can_router_poll_bus_rx(g_rs2_can_bus);
		else
			plant_diag_can_router_poll();

		(void)host_link_poll_tx_once();
		HAL_Delay(1);
	}
}

void diag_finalize_probe(uint8_t kind, uint8_t motor_id, bool got)
{
	if (kind == PLANT_DIAG_PROBE_PARAREAD && !got) {
		if (g_last_probe.raw_frames_seen > 0u &&
		    (g_last_probe.comm_mode == RS02_COMM_PARAREAD ||
		     g_last_probe.comm_mode == RS02_COMM_PARAWRITE))
			g_last_probe.found = true;
		else
			g_last_probe.found = false;
		g_last_probe.probe_kind = kind;
		g_last_probe.motor_id = motor_id;
	} else if (kind == PLANT_DIAG_PROBE_CTRL_FAST) {
		g_last_probe.found = got || g_last_probe.found;
	} else if (kind == PLANT_DIAG_PROBE_CALI) {
		if (got)
			g_last_probe.found = true;
		/* else keep feedback captured during listen */
	} else {
		g_last_probe.found = got;
	}
}

void diag_stale_host_watchdog(void)
{
	uint32_t now = HAL_GetTick();

	if (g_probe_in_progress && g_probe_started_ms != 0u &&
	    (now - g_probe_started_ms) > DIAG_PROBE_STUCK_MS) {
		plant_diag_release_actuator_can();
		g_probe_started_ms = 0u;
		return;
	}

	if (g_probe_in_progress || g_rs2_probe_pending)
		return;

	if (!host_link_command_is_fresh(DIAG_HOST_STALE_MS)) {
		if (g_dm_session_active)
			plant_diag_release_actuator_can();
	}
}

bool plant_diag_is_dxl_command(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return false;
	return cmd -> pdu.data[0] == (uint8_t)PLANT_DIAG_DXL_TAG0 &&
		   cmd -> pdu.data[1] == (uint8_t)PLANT_DIAG_DXL_TAG1 &&
		   cmd -> pdu.data[2] == (uint8_t)PLANT_DIAG_DXL_TAG2;
}

void plant_diag_on_dxl_command(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return;

	g_dxl_pending_kind = cmd->pdu.data[4];
	g_dxl_pending_target_id = cmd->pdu.data[3];
	g_dxl_pending_id_start = cmd->pdu.data[5];
	g_dxl_pending_id_end = cmd->pdu.data[6];
	g_dxl_probe_pending = true;
	g_dxl_feedback_active = true;
}

void plant_diag_can_router_poll(void)
{
	if (g_dm_session_active && g_dm_can_bus < CAN_BACKEND_COUNT)
		can_router_poll_bus(g_dm_can_bus);
	else if (g_rs2_session_active && g_rs2_can_bus < CAN_BACKEND_COUNT)
		can_router_poll_bus(g_rs2_can_bus);
	else {
		/* Plant already polls commanded buses (incl. MCP). End-of-lap:
		 * FDCAN only, and skip buses the last apply already drained so
		 * all×25 does not pay a second CH1–3 flush every lap. Idle /
		 * uncommanded FDCAN still get coverage here. */
		uint32_t already = actuator_last_apply_poll_buses();

		for (can_bus_id_t bus = 0; bus < CAN_FDCAN_COUNT; bus++) {
			if ((already & (1u << (unsigned)bus)) != 0u)
				continue;
			can_router_poll_bus(bus);
		}
	}
}

void plant_diag_yield_usb(void)
{
	if (g_dm_session_active && g_dm_can_bus < CAN_BACKEND_COUNT)
		can_router_poll_bus_rx(g_dm_can_bus);
	else if (g_rs2_session_active && g_rs2_can_bus < CAN_BACKEND_COUNT)
		can_router_poll_bus_rx(g_rs2_can_bus);
	else
		plant_diag_can_router_poll();

	if (!g_probe_in_progress)
		(void)host_link_poll_tx_once();
}

bool plant_diag_blocks_usb_feedback(void)
{
	return g_probe_in_progress && !g_probe_progress_push;
}

void plant_diag_probe_progress(const robstride_probe_result_t *partial)
{
	if (partial == NULL)
		return;

	g_last_probe = *partial;
	g_probe_progress_push = true;
	(void)host_link_poll_tx_once();
	g_probe_progress_push = false;
}

void plant_diag_service(void)
{
	diag_stale_host_watchdog();
	diag_run_rs2_pending();

	if (!g_dxl_probe_pending)
		return;

	g_dxl_probe_pending = false;
	g_probe_in_progress = true;

	dynamixel_probe_run(g_dxl_pending_kind,
	                    g_dxl_pending_target_id,
	                    g_dxl_pending_id_start,
	                    g_dxl_pending_id_end);

	g_probe_in_progress = false;
}

void plant_diag_release_actuator_can(void)
{
	g_rs2_session_active = false;
	g_rs2_quiet_until_ms = 0u;
	g_dm_session_active = false;
	g_dm_quiet_until_ms = 0u;
	g_rs2_probe_pending = false;
	g_probe_in_progress = false;
	g_probe_started_ms = 0u;

	/* Drop bench PDU stamps ('r'/'m'/'d') so plant runtime feedback is clean. */
	g_dm_feedback_active = false;
	g_dm_feedback_ttl = 0u;
	memset(&g_last_dm_probe, 0, sizeof(g_last_dm_probe));
	memset(&g_last_probe, 0, sizeof(g_last_probe));
	g_dxl_feedback_active = false;
}

bool plant_diag_bench_session_active(void)
{
	return g_rs2_session_active || g_dm_session_active;
}

bool plant_diag_probe_busy(void)
{
	return g_probe_in_progress || g_rs2_probe_pending;
}

bool plant_diag_quiet_period_active(void)
{
	if (g_rs2_quiet_until_ms != 0u &&
	    (int32_t)(HAL_GetTick() - g_rs2_quiet_until_ms) < 0)
		return true;
	if (g_dm_quiet_until_ms != 0u &&
	    (int32_t)(HAL_GetTick() - g_dm_quiet_until_ms) < 0)
		return true;
	return false;
}

bool plant_diag_skip_actuator_can(void)
{
	if (plant_diag_bench_session_active() || plant_diag_probe_busy())
		return true;

	if (plant_diag_quiet_period_active())
		return true;

	/* Servo host session used to skip actuator CAN (DXL 50 ms miss timeout
	 * pegged the lap). Real DXLs answer quickly — allow RS02 + DXL together. */
	return false;
}

bool plant_diag_skip_servo_bus(void)
{
	return g_rs2_session_active || g_dm_session_active ||
	       g_probe_in_progress;
}
