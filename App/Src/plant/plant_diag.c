#include "plant/plant_diag.h"
#include "plant/actuator.h"
#include "plant/plugins/robstride.h"
#include "plant/plugins/dynamixel.h"
#include "plant/can/can_router.h"
#include <string.h>

static robstride_probe_result_t g_last_probe;
static volatile bool g_rs2_session_active;
static volatile bool g_probe_in_progress;
static uint8_t g_rs2_motor_id;
static can_bus_id_t g_rs2_can_bus;
static bool g_dxl_feedback_active;
static volatile bool g_dxl_probe_pending;
static uint8_t g_dxl_pending_kind;
static uint8_t g_dxl_pending_target_id;
static uint8_t g_dxl_pending_id_start;
static uint8_t g_dxl_pending_id_end;

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

void plant_diag_service(void)
{
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

static can_bus_id_t plant_diag_pdu_can_bus(const host_pdu_command_t *pdu)
{
	if (pdu == NULL)
		return CAN_BUS_CH1;

	uint8_t host_bus = pdu->data[PLANT_DIAG_PDU_CAN_BUS];
	if (host_bus >= 1u && host_bus <= (uint8_t)CAN_BACKEND_COUNT)
		return (can_bus_id_t)(host_bus - 1u);

	return CAN_BUS_CH1;
}

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

bool plant_diag_is_rs2_command(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return false;

	return pdu_is_scan_request(&cmd->pdu);
}

static void plant_diag_reset_motor(uint8_t motor_id, can_bus_id_t bus)
{
	robstride_probe_result_t tmp;

	if (motor_id == 0u)
		return;

	(void)robstride_probe_id(bus, motor_id, PLANT_DIAG_PROBE_RESET, NULL, 0u, 0u, &tmp);
}

void plant_diag_on_command(const host_command_image_t *cmd)
{
	if (cmd == NULL || !pdu_is_scan_request(&cmd->pdu))
		return;

	uint8_t motor_id = cmd->pdu.data[3];
	uint8_t kind = cmd->pdu.data[4];
	can_bus_id_t bus = plant_diag_pdu_can_bus(&cmd->pdu);

	if (kind == PLANT_DIAG_SESSION_BEGIN) {
		g_rs2_session_active = true;
		if (motor_id != 0u)
			g_rs2_motor_id = motor_id;
		g_rs2_can_bus = bus;
		actuator_desire_clear();
		memset(&g_last_probe, 0, sizeof(g_last_probe));
		g_last_probe.probe_kind = kind;
		return;
	}

	if (kind == PLANT_DIAG_SESSION_END) {
		g_rs2_session_active = false;
		plant_diag_reset_motor(g_rs2_motor_id, g_rs2_can_bus);
		actuator_desire_clear();
		memset(&g_last_probe, 0, sizeof(g_last_probe));
		g_last_probe.probe_kind = kind;
		return;
	}

	if (motor_id != 0u)
		g_rs2_motor_id = motor_id;
	g_rs2_can_bus = bus;

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
	uint32_t param_raw_value = (uint32_t)cmd->pdu.data[7] |
	                           ((uint32_t)cmd->pdu.data[8] << 8) |
	                           ((uint32_t)cmd->pdu.data[9] << 16) |
	                           ((uint32_t)cmd->pdu.data[10] << 24);
	bool got = robstride_probe_id(bus, motor_id, kind,
	                              &cmd->actuator_commands[0],
	                              param_index,
	                              param_raw_value,
	                              &g_last_probe);
	g_probe_in_progress = false;

	if (kind == PLANT_DIAG_PROBE_PARAREAD && !got) {
		if (g_last_probe.raw_frames_seen > 0u &&
		    (g_last_probe.comm_mode == RS02_COMM_PARAREAD ||
		     g_last_probe.comm_mode == RS02_COMM_PARAWRITE))
			g_last_probe.found = true;
		else
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

	if (g_dxl_feedback_active) { // exception for the dynamixel scripts
		dynamixel_probe_feedback_fill(pdu);
		g_dxl_feedback_active = false;
		return;
	}

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
