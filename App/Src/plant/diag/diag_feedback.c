#include "plant/diag/diag.h"
#include "plant/diag/diag_internal.h"
#include "plant/can/mcp2518fd.h"
#include "plant/plugins/dynamixel.h"
#include <string.h>

void plant_diag_feedback_stamp_fw_marker(host_pdu_feedback_t *pdu)
{
	if (pdu == NULL)
		return;

	pdu->data[29] = (uint8_t)PLANT_DM_FW_MARKER0;
	pdu->data[30] = (uint8_t)PLANT_DM_FW_MARKER1;
}

void plant_diag_feedback_sent(uint8_t probe_kind)
{
	if (g_rs2_clear_after_send_kind != 0u &&
	    g_rs2_clear_after_send_kind == probe_kind) {
		memset(&g_last_probe, 0, sizeof(g_last_probe));
		g_rs2_clear_after_send_kind = 0u;
	}
}

void plant_diag_feedback_fill(host_pdu_feedback_t *pdu)
{
	if (pdu == NULL)
		return;

	if (g_dxl_feedback_active) {
		dynamixel_probe_feedback_fill(pdu);
		g_dxl_feedback_active = false;
		return;
	}

	if (g_dm_feedback_active || g_dm_feedback_ttl > 0u) {
		memset(pdu->data, 0, sizeof(pdu->data));
		pdu->data[0] = (uint8_t)PLANT_DIAG_DM_RESP_TAG;
		pdu->data[1] = g_last_dm_probe.motor_id;
		pdu->data[2] = g_last_dm_probe.found ? 1u : 0u;
		pdu->data[3] = g_last_dm_probe.probe_kind;
		memcpy(&pdu->data[4], &g_last_dm_probe.rx_can_id, sizeof(uint32_t));
		memcpy(&pdu->data[8], g_last_dm_probe.data, 8u);
		memcpy(&pdu->data[16], &g_last_dm_probe.param_value, sizeof(uint32_t));
		memcpy(&pdu->data[20], &g_last_dm_probe.position, sizeof(float));
		pdu->data[24] = g_last_dm_probe.discovered_id;
		pdu->data[25] = g_last_dm_probe.master_id;
		pdu->data[26] = g_last_dm_probe.raw_frames_seen;
		pdu->data[27] = g_last_dm_probe.err;
		pdu->data[28] = g_last_dm_probe.tx_frames_sent;
		pdu->data[31] = g_last_dm_probe.param_rid;
		plant_diag_feedback_stamp_fw_marker(pdu);
		if (g_dm_feedback_ttl > 0u)
			g_dm_feedback_ttl--;
		else
			g_dm_feedback_active = false;
		return;
	}

	if (g_cm_feedback_active || g_cm_feedback_ttl > 0u) {
		memset(pdu->data, 0, sizeof(pdu->data));
		pdu->data[0] = (uint8_t)PLANT_DIAG_CM_RESP_TAG;
		pdu->data[1] = g_last_cm_probe.motor_id;
		pdu->data[2] = g_last_cm_probe.found ? 1u : 0u;
		pdu->data[3] = g_last_cm_probe.probe_kind;
		memcpy(&pdu->data[4], &g_last_cm_probe.rx_can_id, sizeof(uint32_t));
		memcpy(&pdu->data[8], g_last_cm_probe.data, 8u);
		memcpy(&pdu->data[20], &g_last_cm_probe.position, sizeof(float));
		pdu->data[24] = g_last_cm_probe.discovered_id;
		pdu->data[26] = g_last_cm_probe.raw_frames_seen;
		pdu->data[27] = g_last_cm_probe.fault;
		pdu->data[28] = g_last_cm_probe.tx_frames_sent;
		plant_diag_feedback_stamp_fw_marker(pdu);
		if (g_cm_feedback_ttl > 0u)
			g_cm_feedback_ttl--;
		else
			g_cm_feedback_active = false;
		return;
	}

	if (g_ze_feedback_active || g_ze_feedback_ttl > 0u) {
		memset(pdu->data, 0, sizeof(pdu->data));
		pdu->data[0] = (uint8_t)PLANT_DIAG_ZE_RESP_TAG;
		pdu->data[1] = g_last_ze_probe.node_id;
		pdu->data[2] = g_last_ze_probe.found ? 1u : 0u;
		pdu->data[3] = g_last_ze_probe.vendor_match ? 1u : 0u;
		memcpy(&pdu->data[4], &g_last_ze_probe.vendor, sizeof(uint32_t));
		memcpy(&pdu->data[8], &g_last_ze_probe.product, sizeof(uint32_t));
		memcpy(&pdu->data[12], &g_last_ze_probe.revision, sizeof(uint32_t));
		memcpy(&pdu->data[16], &g_last_ze_probe.position_rad, sizeof(float));
		pdu->data[20] = g_last_ze_probe.position_valid ? 1u : 0u;
		pdu->data[24] = g_last_ze_probe.discovered_id;
		plant_diag_feedback_stamp_fw_marker(pdu);
		if (g_ze_feedback_ttl > 0u)
			g_ze_feedback_ttl--;
		else
			g_ze_feedback_active = false;
		return;
	}

	if (g_probe_in_progress && !g_probe_progress_push)
		return;

	if (g_last_probe.probe_kind == 0u && g_last_probe.motor_id == 0u &&
	    g_last_probe.raw_frames_seen == 0u && !g_last_probe.found)
		return;

	memset(pdu->data, 0, sizeof(pdu->data));
	pdu->data[0] = (uint8_t)PLANT_DIAG_PDU_RESP_TAG;
	pdu->data[1] = g_last_probe.motor_id;
	pdu->data[2] = g_last_probe.found ? 1u : 0u;
	pdu->data[3] = g_last_probe.comm_mode;
	memcpy(&pdu->data[4], &g_last_probe.ext_id, sizeof(uint32_t));
	memcpy(&pdu->data[8], g_last_probe.data, 8u);
	memcpy(&pdu->data[16], &g_last_probe.temperature, sizeof(float));
	memcpy(&pdu->data[20], &g_last_probe.position, sizeof(float));
	pdu->data[24] = g_last_probe.discovered_id;
	pdu->data[25] = g_last_probe.probe_kind;
	pdu->data[26] = g_last_probe.raw_frames_seen;
	/* Multi-bus discover: host bus 1..6. Else legacy mcp init mask. */
	if (g_last_probe.can_bus >= 1u &&
	    g_last_probe.can_bus <= (uint8_t)CAN_BACKEND_COUNT)
		pdu->data[PLANT_DIAG_RS2_PDU_RESP_BUS] = g_last_probe.can_bus;
	else
		pdu->data[PLANT_DIAG_RS2_PDU_RESP_BUS] = mcp2518_init_mask();
	{
		uint8_t mcp_rail = 0u;

		if (g_rs2_can_bus >= CAN_BUS_CH4)
			mcp_rail = (uint8_t)(g_rs2_can_bus - CAN_BUS_CH4);
		pdu->data[28] = mcp2518_rail_opmod(mcp_rail);
	}
	if (g_last_probe.probe_kind == PLANT_DIAG_PROBE_MCP_SMOKE ||
	    g_last_probe.probe_kind == PLANT_DIAG_PROBE_MCP_WAKE ||
	    g_last_probe.probe_kind == PLANT_DIAG_PROBE_MCP_DISABLE) {
		pdu->data[29] = g_last_mcp_smoke.tx_ok;
		pdu->data[30] = g_last_mcp_smoke.tx_fail;
		pdu->data[31] = g_last_mcp_smoke.tec;
		pdu->data[8] = g_last_mcp_smoke.tx_fifo_sta;
		pdu->data[9] = g_last_mcp_smoke.tx_fifo_con;
		pdu->data[10] = g_last_mcp_smoke.c1con_b2;
		pdu->data[11] = g_last_mcp_smoke.osc_b1;
		pdu->data[12] = g_last_mcp_smoke.osc_b0;
		pdu->data[13] = g_last_mcp_smoke.nbt_tseg1;
		pdu->data[14] = g_last_mcp_smoke.bdiag1_b0;
		pdu->data[15] = g_last_mcp_smoke.nbt_brp;
		pdu->data[24] = g_last_mcp_smoke.tx_nack;
		pdu->data[7] = g_last_mcp_smoke.bdiag1_b1;
		pdu->data[6] = g_last_mcp_smoke.tec_before;
		pdu->data[17] = g_last_mcp_smoke.rec;
		pdu->data[18] = g_last_mcp_smoke.ext_loopback_ok;
		if (!g_last_probe.found)
			pdu->data[3] = g_last_mcp_smoke.rx_fifo_sta;
	} else {
		uint8_t rail = 0u;

		if (g_rs2_can_bus >= CAN_BUS_CH4)
			rail = (uint8_t)(g_rs2_can_bus - CAN_BUS_CH4);

		/* RS2 bench on MCP: keep probe ext_id/can_data/discovered_id intact.
		 * MCP smoke fields live only in pdu[29..31] (and [27..28] above). */
		{
			uint8_t tx_ok = 0u;
			uint8_t tx_fail = 0u;
			uint8_t tx_nack = 0u;
			uint8_t tec = 0u;
			uint8_t rec = 0u;

			mcp2518_get_tx_stats(rail, &tx_ok, &tx_fail, &tx_nack);
			mcp2518_rail_trec(rail, &tec, &rec);
			(void)tx_nack;
			(void)rec;
			pdu->data[29] = tx_ok;
			pdu->data[30] = tx_fail;
			pdu->data[31] = tec;
		}
	}

	if (g_last_probe.probe_kind == PLANT_DIAG_SESSION_BEGIN ||
	    g_last_probe.probe_kind == PLANT_DIAG_SESSION_END)
		g_rs2_clear_after_send_kind = g_last_probe.probe_kind;
}
