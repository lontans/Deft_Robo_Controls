#include "plant/plant_diag.h"
#include "plant/plugins/robstride.h"
#include <string.h>

static robstride_probe_result_t g_last_probe;
static volatile bool g_rs2_session_active;
static volatile bool g_probe_in_progress;
static uint8_t g_rs2_motor_id;

bool plant_diag_skip_actuator_can(void)
{
	return g_rs2_session_active || g_probe_in_progress;
}

static bool pdu_is_scan_request(const host_pdu_command_t *pdu)
{
	if (pdu == NULL)
		return false;

	return pdu->data[0] == (uint8_t)PLANT_DIAG_PDU_TAG0 &&
	       pdu->data[1] == (uint8_t)PLANT_DIAG_PDU_TAG1 &&
	       pdu->data[2] == (uint8_t)PLANT_DIAG_PDU_TAG2;
}

static void plant_diag_reset_motor(uint8_t motor_id)
{
	robstride_probe_result_t tmp;

	if (motor_id == 0u)
		return;

	(void)robstride_probe_id(motor_id, PLANT_DIAG_PROBE_RESET, NULL, 0u, &tmp);
}

void plant_diag_on_command(const host_command_image_t *cmd)
{
	if (cmd == NULL || !pdu_is_scan_request(&cmd->pdu))
		return;

	uint8_t motor_id = cmd->pdu.data[3];
	uint8_t kind = cmd->pdu.data[4];

	if (kind == PLANT_DIAG_SESSION_BEGIN) {
		g_rs2_session_active = true;
		if (motor_id != 0u)
			g_rs2_motor_id = motor_id;
		memset(&g_last_probe, 0, sizeof(g_last_probe));
		g_last_probe.probe_kind = kind;
		return;
	}

	if (kind == PLANT_DIAG_SESSION_END) {
		g_rs2_session_active = false;
		plant_diag_reset_motor(g_rs2_motor_id);
		memset(&g_last_probe, 0, sizeof(g_last_probe));
		g_last_probe.probe_kind = kind;
		return;
	}

	if (motor_id != 0u)
		g_rs2_motor_id = motor_id;

	if (kind != PLANT_DIAG_PROBE_CTRL_FAST) {
		memset(&g_last_probe, 0, sizeof(g_last_probe));
		g_last_probe.probe_kind = kind;
		g_last_probe.motor_id = motor_id;
	} else {
		g_last_probe.probe_kind = kind;
		g_last_probe.motor_id = motor_id;
	}

	g_probe_in_progress = true;
	uint16_t param_index = (uint16_t)cmd->pdu.data[5] |
	                       ((uint16_t)cmd->pdu.data[6] << 8);
	bool got = robstride_probe_id(motor_id, kind,
	                              &cmd->actuator_commands[0],
	                              param_index,
	                              &g_last_probe);
	g_probe_in_progress = false;

	if (kind == PLANT_DIAG_PROBE_PARAREAD && !got) {
		g_last_probe.found = false;
		g_last_probe.probe_kind = kind;
		g_last_probe.motor_id = motor_id;
	} else if (kind == PLANT_DIAG_PROBE_CTRL_FAST)
		g_last_probe.found = got || g_last_probe.found;
	else
		g_last_probe.found = got;
}

void plant_diag_feedback_fill(host_pdu_feedback_t *pdu)
{
	if (pdu == NULL)
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
}
