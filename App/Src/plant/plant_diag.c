#include "plant/plant_diag.h"
#include "plant/actuator.h"
#include "plant/servo.h"
#include "plant/can/can_router.h"
#include "plant/can/mcp2518fd.h"
#include "plant/can/spi_can_router.h"
#include "plant/plugins/dynamixel.h"
#include "plant/plugins/robstride.h"
#include "plant/plugins/damiao.h"
#include "plant/plugin_schema/plugin_table.h"
#include "host/host_link.h"
#include "main.h"
#include <string.h>

#define MCP_BENCH_LISTEN_MS_SMOKE 300u
#define MCP_BENCH_LISTEN_MS_WAKE  450u
#define MCP_BENCH_ZERO_GAIN_BURST 12u
#define MCP_BENCH_HOST_SYNC_FRAMES 4u

static robstride_probe_result_t g_last_probe;
static mcp2518_smoke_result_t g_last_mcp_smoke;
static volatile bool g_rs2_session_active;
static volatile bool g_probe_in_progress;
static uint8_t g_rs2_motor_id;
static can_bus_id_t g_rs2_can_bus;
static uint32_t g_rs2_quiet_until_ms;
static bool g_dxl_feedback_active;
static volatile bool g_dxl_probe_pending;
static uint8_t g_dxl_pending_kind;
static uint8_t g_dxl_pending_target_id;
static uint8_t g_dxl_pending_id_start;
static uint8_t g_dxl_pending_id_end;
static volatile bool g_rs2_probe_pending;
static uint8_t g_rs2_pending_kind;
static uint8_t g_rs2_pending_motor_id;
static can_bus_id_t g_rs2_pending_bus;
static actuator_desire_t g_rs2_pending_desire;
static bool g_rs2_pending_has_desire;
static uint16_t g_rs2_pending_param_index;
static uint32_t g_rs2_pending_param_raw;
static uint8_t g_rs2_clear_after_send_kind;
static damiao_probe_result_t g_last_dm_probe;
static volatile bool g_dm_feedback_active;
static uint8_t g_dm_pending_kind;
static uint8_t g_dm_pending_motor_id;
static uint8_t g_dm_pending_master_id;
static uint8_t g_dm_pending_param_rid;
static uint16_t g_dm_pending_listen_ms;
static can_bus_id_t g_dm_pending_bus;
static volatile bool g_dm_session_active;
static can_bus_id_t g_dm_can_bus;
static uint8_t g_dm_feedback_ttl;

static void plant_diag_dm_clear_actuator_mirror(void)
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

static void plant_diag_dm_publish_actuator_state(void)
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
	/* Bench mirror: velocity=probed ID, torque=ESC_ID, temperature=Master ID. */
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
	else
		can_router_poll();
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
	return g_probe_in_progress;
}

static bool rs2_probe_kind_needs_desire(uint8_t kind)
{
	switch (kind) {
	case PLANT_DIAG_PROBE_FULL:
	case PLANT_DIAG_PROBE_ENABLE_CTRL:
	case PLANT_DIAG_PROBE_CTRL_ONLY:
	case PLANT_DIAG_PROBE_CTRL_FAST:
		return true;
	default:
		return false;
	}
}

static void plant_diag_finalize_probe(uint8_t kind, uint8_t motor_id, bool got)
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
	} else {
		g_last_probe.found = got;
	}
}

static void plant_diag_flush_usb(void)
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

static void plant_diag_mcp_finalize_bench(uint8_t motor_id, can_bus_id_t bus,
                                          uint8_t probe_kind)
{
	uint8_t tec_before = g_last_mcp_smoke.tec_before;
	uint8_t rail = (uint8_t)(bus - CAN_BUS_CH4);

	mcp2518_rail_trec(rail, &g_last_mcp_smoke.tec, &g_last_mcp_smoke.rec);
	mcp2518_get_tx_stats(rail, NULL, NULL, &g_last_mcp_smoke.tx_nack);
	mcp2518_refresh_smoke_diag(bus, &g_last_mcp_smoke);
	g_last_mcp_smoke.tec_before = tec_before;

	if (g_last_mcp_smoke.tec > tec_before) {
		uint8_t nack_by_tec = (uint8_t)((g_last_mcp_smoke.tec - tec_before + 7u) / 8u);
		if (nack_by_tec > g_last_mcp_smoke.tx_nack)
			g_last_mcp_smoke.tx_nack = nack_by_tec;
	}

	g_last_probe.raw_frames_seen = g_last_mcp_smoke.rx_frames;
	g_last_probe.probe_kind = probe_kind;
	g_last_probe.motor_id = motor_id;

	if (probe_kind == PLANT_DIAG_PROBE_MCP_SMOKE) {
		g_last_probe.found = (g_last_mcp_smoke.tx_ok > 0u &&
		                      g_last_mcp_smoke.nbt_brp == MCP2518_NBT_BRP_EXPECT &&
		                      g_last_mcp_smoke.nbt_tseg1 == MCP2518_NBT_TSEG1_EXPECT);
	} else {
		g_last_probe.found = g_last_probe.found ||
		                     (g_last_mcp_smoke.rx_frames > 0u);
	}
}

static void plant_diag_mcp_soft_recover(can_bus_id_t bus)
{
	if (bus < CAN_BUS_CH4)
		return;

	/* Full reinit glitches TXQ when motor is on-bus — only reinit if not in normal. */
	if (mcp2518_rail_opmod((uint8_t)(bus - CAN_BUS_CH4)) != 6u) {
		(void)mcp2518_reinit_rail(bus);
	} else {
		(void)mcp2518_recover_if_busoff(bus);
		mcp2518_drain_rx(bus);
		mcp2518_reset_tx_stats(bus);
		mcp2518_prepare_tx(bus);
	}
}

static bool plant_diag_mcp_tx_frame(can_bus_id_t bus, const can_frame_t *frame)
{
	if (!mcp2518_send(bus, frame)) {
		if (g_last_mcp_smoke.tx_fail < 0xFFu)
			g_last_mcp_smoke.tx_fail++;
		mcp2518_refresh_smoke_diag(bus, &g_last_mcp_smoke);
		return false;
	}

	if (g_last_mcp_smoke.tx_ok < 0xFFu)
		g_last_mcp_smoke.tx_ok++;
	mcp2518_refresh_smoke_diag(bus, &g_last_mcp_smoke);
	can_router_poll_bus(bus);
	return true;
}

static void plant_diag_mcp_listen(can_bus_id_t bus, uint16_t listen_ms)
{
	can_frame_t rx;

	for (uint16_t i = 0; i < listen_ms; i++) {
		spi_can_router_poll_bus_rx(bus);
		while (spi_can_router_rx_pop(bus, &rx) == CAN_OK) {
			if (g_last_mcp_smoke.rx_frames < 0xFFu)
				g_last_mcp_smoke.rx_frames++;
			robstride_bench_note_rx(&rx, g_last_probe.motor_id, &g_last_probe);
		}
		plant_diag_yield_usb();
		HAL_Delay(1);
	}
}

static void plant_diag_mcp_host_sync(can_bus_id_t bus, const actuator_config_t *cfg)
{
	can_frame_t frame;

	/* Same reset → enable → run_mode → enable as mcp-wake. Motors at ~0.07 A from
	 * power-on or a prior session may ignore zero-gain/disable until this runs. */
	if (robstride_send_reset(cfg, &frame) == PLUGIN_OK) {
		(void)plant_diag_mcp_tx_frame(bus, &frame);
		HAL_Delay(200);
	}

	if (robstride_send_enable(cfg, &frame) == PLUGIN_OK) {
		(void)plant_diag_mcp_tx_frame(bus, &frame);
		HAL_Delay(50);
	}

	if (robstride_set_run_mode(cfg, RS02_RUN_MODE_MOVE, &frame) == PLUGIN_OK) {
		(void)plant_diag_mcp_tx_frame(bus, &frame);
		HAL_Delay(30);
	}

	if (robstride_send_enable(cfg, &frame) == PLUGIN_OK) {
		(void)plant_diag_mcp_tx_frame(bus, &frame);
		HAL_Delay(30);
	}
}

static void plant_diag_mcp_smoke_sync(uint8_t motor_id, can_bus_id_t bus)
{
	actuator_config_t cfg = {
		.bus = bus,
		.protocol = PROTO_ROBSTRIDE,
		.motor_id = motor_id,
		.enabled = true,
	};
	can_frame_t frame;
	uint8_t tec_delta = 0u;
	bool bus_activity = false;
	bool ok = false;

	memset(&g_last_mcp_smoke, 0, sizeof(g_last_mcp_smoke));
	g_last_probe.probe_kind = PLANT_DIAG_PROBE_MCP_SMOKE;
	g_last_probe.motor_id = motor_id;
	g_rs2_can_bus = bus;

	if (bus < CAN_BUS_CH4) {
		g_last_probe.found = false;
		return;
	}

	if (robstride_send_para_read(&cfg, RS02_PARAM_BUS_VOLT, &frame) != PLUGIN_OK) {
		g_last_probe.found = false;
		return;
	}

	/* Soft-recover — do not reinit when already in CAN2.0 normal (breaks TXQ). */
	plant_diag_mcp_soft_recover(bus);

	ok = mcp2518_bus_smoke(bus, &frame, MCP_BENCH_LISTEN_MS_SMOKE,
	                       &g_last_mcp_smoke);
	can_router_poll_bus(bus);
	g_last_probe.raw_frames_seen = g_last_mcp_smoke.rx_frames;

	if (g_last_mcp_smoke.tec > g_last_mcp_smoke.tec_before)
		tec_delta = (uint8_t)(g_last_mcp_smoke.tec - g_last_mcp_smoke.tec_before);

	bus_activity = (g_last_mcp_smoke.rx_frames > 0u ||
	                g_last_mcp_smoke.tx_nack > 0u ||
	                tec_delta > 0u);

	g_last_probe.found = ok && (g_last_mcp_smoke.tx_ok > 0u) &&
	                     (bus_activity ||
	                      g_last_mcp_smoke.tec == g_last_mcp_smoke.tec_before);
}

static void plant_diag_mcp_wake_sync(uint8_t motor_id, can_bus_id_t bus)
{
	actuator_config_t cfg = {
		.bus = bus,
		.protocol = PROTO_ROBSTRIDE,
		.motor_id = motor_id,
		.enabled = true,
	};

	memset(&g_last_mcp_smoke, 0, sizeof(g_last_mcp_smoke));
	g_last_probe.probe_kind = PLANT_DIAG_PROBE_MCP_WAKE;
	g_last_probe.motor_id = motor_id;
	g_rs2_can_bus = bus;

	if (bus < CAN_BUS_CH4) {
		g_last_probe.found = false;
		return;
	}

	plant_diag_mcp_soft_recover(bus);
	mcp2518_refresh_smoke_diag(bus, &g_last_mcp_smoke);
	g_last_mcp_smoke.tec_before = g_last_mcp_smoke.tec;

	plant_diag_mcp_host_sync(bus, &cfg);

	plant_diag_mcp_listen(bus, MCP_BENCH_LISTEN_MS_WAKE);
	plant_diag_mcp_finalize_bench(motor_id, bus, PLANT_DIAG_PROBE_MCP_WAKE);
}

static void plant_diag_mcp_disable_sync(uint8_t motor_id, can_bus_id_t bus)
{
	actuator_config_t cfg = {
		.bus = bus,
		.protocol = PROTO_ROBSTRIDE,
		.motor_id = motor_id,
		.enabled = true,
	};
	actuator_desire_t desire = {
		.position = 0.0f,
		.velocity = 0.0f,
		.kp = 0.0f,
		.kd = 0.0f,
		.torque = 0.0f,
	};
	can_frame_t frame;

	memset(&g_last_mcp_smoke, 0, sizeof(g_last_mcp_smoke));
	g_last_probe.probe_kind = PLANT_DIAG_PROBE_MCP_DISABLE;
	g_last_probe.motor_id = motor_id;
	g_rs2_can_bus = bus;

	if (bus < CAN_BUS_CH4) {
		g_last_probe.found = false;
		return;
	}

	plant_diag_mcp_soft_recover(bus);
	mcp2518_refresh_smoke_diag(bus, &g_last_mcp_smoke);
	g_last_mcp_smoke.tec_before = g_last_mcp_smoke.tec;

	plant_diag_mcp_host_sync(bus, &cfg);

	for (uint8_t i = 0; i < MCP_BENCH_ZERO_GAIN_BURST; i++) {
		if (plugin_pack_tx(&cfg, &desire, &frame) != PLUGIN_OK)
			break;
		(void)plant_diag_mcp_tx_frame(bus, &frame);
		HAL_Delay(8);
	}

	if (robstride_send_disable(&cfg, &frame) != PLUGIN_OK) {
		g_last_probe.found = false;
		return;
	}
	(void)plant_diag_mcp_tx_frame(bus, &frame);
	HAL_Delay(50);

	plant_diag_mcp_listen(bus, 120u);
	plant_diag_mcp_finalize_bench(motor_id, bus, PLANT_DIAG_PROBE_MCP_DISABLE);
}

static void plant_diag_run_rs2_pending(void)
{
	if (!g_rs2_probe_pending)
		return;

	g_rs2_probe_pending = false;

	uint8_t kind = g_rs2_pending_kind;
	uint8_t motor_id = g_rs2_pending_motor_id;
	can_bus_id_t bus = g_rs2_pending_bus;

	if (kind == PLANT_DIAG_PROBE_MCP_SMOKE) {
		plant_diag_mcp_smoke_sync(motor_id, bus);
		g_probe_in_progress = false;
		plant_diag_flush_usb();
		return;
	}

	if (kind == PLANT_DIAG_PROBE_MCP_WAKE) {
		plant_diag_mcp_wake_sync(motor_id, bus);
		g_probe_in_progress = false;
		plant_diag_flush_usb();
		return;
	}

	if (kind == PLANT_DIAG_PROBE_MCP_DISABLE) {
		plant_diag_mcp_disable_sync(motor_id, bus);
		g_probe_in_progress = false;
		plant_diag_flush_usb();
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
	plant_diag_finalize_probe(kind, motor_id, got);

	g_probe_in_progress = false;
	plant_diag_flush_usb();
}

void plant_diag_service(void)
{
	plant_diag_run_rs2_pending();

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
	if (g_rs2_session_active || g_dm_session_active ||
	    g_probe_in_progress)
		return true;

	if (g_rs2_quiet_until_ms != 0u &&
	    (int32_t)(HAL_GetTick() - g_rs2_quiet_until_ms) < 0)
		return true;

	return servo_host_session_active();
}

bool plant_diag_skip_servo_bus(void)
{
	return g_rs2_session_active || g_dm_session_active ||
	       g_probe_in_progress;
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
	can_bus_id_t bus = plant_diag_pdu_can_bus(&cmd->pdu);

	if (kind == PLANT_DIAG_SESSION_BEGIN) {
		g_dm_session_active = true;
		g_dm_can_bus = bus;
		can_router_discard_pending_tx();
		if (bus < CAN_BUS_CH4)
			can_rx_drain(bus);
		actuator_desire_clear();
		plant_diag_dm_clear_actuator_mirror();
		memset(&g_last_dm_probe, 0, sizeof(g_last_dm_probe));
		g_last_dm_probe.probe_kind = kind;
		g_last_dm_probe.motor_id = cmd->pdu.data[3];
		g_last_dm_probe.found = true;
		g_dm_feedback_active = true;
		g_dm_feedback_ttl = PLANT_DM_FB_TTL_SESSION;
		plant_diag_flush_usb();
		return;
	}

	if (kind == PLANT_DIAG_SESSION_END) {
		g_dm_session_active = false;
		memset(&g_last_dm_probe, 0, sizeof(g_last_dm_probe));
		g_last_dm_probe.probe_kind = kind;
		g_last_dm_probe.motor_id = cmd->pdu.data[3];
		g_last_dm_probe.found = true;
		g_dm_feedback_active = true;
		g_dm_feedback_ttl = PLANT_DM_FB_TTL_SESSION;
		plant_diag_flush_usb();
		return;
	}

	if (g_probe_in_progress || g_rs2_probe_pending)
		return;

	g_dm_pending_motor_id = cmd->pdu.data[3];
	g_dm_pending_kind = kind;
	g_dm_pending_master_id = cmd->pdu.data[PLANT_DIAG_DM_PDU_MASTER_ID];
	g_dm_pending_listen_ms = cmd->pdu.data[PLANT_DIAG_DM_PDU_LISTEN_MS];
	g_dm_pending_param_rid = cmd->pdu.data[PLANT_DIAG_DM_PDU_PARAM_RID];
	if (g_dm_pending_listen_ms == 0u)
		g_dm_pending_listen_ms = 15u;
	g_dm_pending_bus = bus;

	plant_diag_dm_clear_actuator_mirror();
	memset(&g_last_dm_probe, 0, sizeof(g_last_dm_probe));
	g_last_dm_probe.motor_id = g_dm_pending_motor_id;
	g_last_dm_probe.probe_kind = g_dm_pending_kind;

	if (!damiao_probe_id(g_dm_pending_bus,
	                     g_dm_pending_motor_id,
	                     g_dm_pending_kind,
	                     g_dm_pending_master_id,
	                     g_dm_pending_param_rid,
	                     g_dm_pending_listen_ms,
	                     &g_last_dm_probe))
		g_last_dm_probe.found = false;

	plant_diag_dm_publish_actuator_state();
	g_dm_feedback_active = true;
	g_dm_feedback_ttl = PLANT_DM_FB_TTL_PROBE;
	plant_diag_flush_usb();
}

static void plant_diag_reset_motor(uint8_t motor_id, can_bus_id_t bus)
{
	robstride_probe_result_t tmp;

	if (motor_id == 0u)
		return;

	(void)robstride_probe_id(bus, motor_id, PLANT_DIAG_PROBE_RESET, NULL, 0u, 0u, &tmp);
}

static void plant_diag_queue_probe(const host_command_image_t *cmd,
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
	if (rs2_probe_kind_needs_desire(kind)) {
		g_rs2_pending_desire = cmd->actuator_commands[0];
		g_rs2_pending_has_desire = true;
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
		g_rs2_quiet_until_ms = 0u;
		if (motor_id != 0u)
			g_rs2_motor_id = motor_id;
		g_rs2_can_bus = bus;
		can_router_discard_pending_tx();
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
		plant_diag_flush_usb();
		return;
	}

	if (kind == PLANT_DIAG_SESSION_END) {
		g_rs2_session_active = false;
		g_rs2_quiet_until_ms = HAL_GetTick() + PLANT_DIAG_RS2_QUIET_MS;
		if (g_rs2_can_bus < CAN_BUS_CH4)
			plant_diag_reset_motor(g_rs2_motor_id, g_rs2_can_bus);
		actuator_desire_clear();
		memset(&g_last_probe, 0, sizeof(g_last_probe));
		g_last_probe.probe_kind = kind;
		g_last_probe.found = true;
		plant_diag_flush_usb();
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
		plant_diag_queue_probe(cmd, motor_id, kind, bus);
		return;
	}

	plant_diag_queue_probe(cmd, motor_id, kind, bus);
}

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
		if (g_dm_feedback_ttl > 0u)
			g_dm_feedback_ttl--;
		else
			g_dm_feedback_active = false;
		return;
	}

	if (g_probe_in_progress)
		return;

	/* PLANT_DIAG_PROBE_FULL is 0 — only skip when nothing has been probed yet. */
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
	pdu->data[27] = mcp2518_init_mask();
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

		if (g_rs2_can_bus >= CAN_BUS_CH4) {
			mcp2518_smoke_result_t smoke;

			mcp2518_refresh_smoke_diag(g_rs2_can_bus, &smoke);
			pdu->data[6] = smoke.tec_before;
			pdu->data[7] = smoke.bdiag1_b1;
			pdu->data[8] = smoke.tx_fifo_sta;
			pdu->data[9] = smoke.tx_fifo_con;
			pdu->data[10] = smoke.c1con_b2;
			pdu->data[11] = smoke.osc_b1;
			pdu->data[12] = smoke.osc_b0;
			pdu->data[13] = smoke.nbt_tseg1;
			pdu->data[14] = smoke.bdiag1_b0;
			pdu->data[15] = smoke.nbt_brp;
			pdu->data[24] = smoke.tx_nack;
			pdu->data[29] = smoke.tx_ok;
			pdu->data[30] = smoke.tx_fail;
			pdu->data[31] = smoke.tec;
			pdu->data[17] = smoke.rec;
			pdu->data[18] = smoke.ext_loopback_ok;
		} else {
			uint8_t tx_ok = 0u;
			uint8_t tx_fail = 0u;
			uint8_t tx_nack = 0u;
			uint8_t tec = 0u;
			uint8_t rec = 0u;

			mcp2518_get_tx_stats(rail, &tx_ok, &tx_fail, &tx_nack);
			mcp2518_rail_trec(rail, &tec, &rec);
			(void)tx_nack;
			pdu->data[29] = tx_ok;
			pdu->data[30] = tx_fail;
			pdu->data[31] = tec;
			(void)rec;
		}
	}

	if (g_last_probe.probe_kind == PLANT_DIAG_SESSION_BEGIN ||
	    g_last_probe.probe_kind == PLANT_DIAG_SESSION_END)
		g_rs2_clear_after_send_kind = g_last_probe.probe_kind;
}
