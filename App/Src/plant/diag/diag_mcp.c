#include "plant/diag/diag.h"
#include "plant/diag/diag_internal.h"
#include "plant/actuator.h"
#include "plant/can/can_router.h"
#include "plant/can/mcp2518fd.h"
#include "plant/can/spi_can_router.h"
#include "plant/plugins/robstride.h"
#include "plant/plugin_schema/plugin_table.h"
#include "main.h"
#include <string.h>

static void diag_mcp_finalize_bench(uint8_t motor_id, can_bus_id_t bus,
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

static void diag_mcp_soft_recover(can_bus_id_t bus)
{
	if (bus < CAN_BUS_CH4)
		return;

	if (mcp2518_rail_opmod((uint8_t)(bus - CAN_BUS_CH4)) != 6u) {
		(void)mcp2518_reinit_rail(bus);
	} else {
		(void)mcp2518_recover_if_busoff(bus);
		mcp2518_drain_rx(bus);
		mcp2518_reset_tx_stats(bus);
		mcp2518_prepare_tx(bus);
	}
}

static bool diag_mcp_tx_frame(can_bus_id_t bus, const can_frame_t *frame)
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

static void diag_mcp_listen(can_bus_id_t bus, uint16_t listen_ms)
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

static void diag_mcp_host_sync(can_bus_id_t bus, const actuator_config_t *cfg)
{
	can_frame_t frame;

	if (robstride_send_reset(cfg, &frame) == PLUGIN_OK) {
		(void)diag_mcp_tx_frame(bus, &frame);
		HAL_Delay(200);
	}

	if (robstride_send_enable(cfg, &frame) == PLUGIN_OK) {
		(void)diag_mcp_tx_frame(bus, &frame);
		HAL_Delay(50);
	}

	if (robstride_set_run_mode(cfg, RS02_RUN_MODE_MOVE, &frame) == PLUGIN_OK) {
		(void)diag_mcp_tx_frame(bus, &frame);
		HAL_Delay(30);
	}

	if (robstride_send_enable(cfg, &frame) == PLUGIN_OK) {
		(void)diag_mcp_tx_frame(bus, &frame);
		HAL_Delay(30);
	}
}

void diag_mcp_smoke_sync(uint8_t motor_id, can_bus_id_t bus)
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

	diag_mcp_soft_recover(bus);

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

void diag_mcp_wake_sync(uint8_t motor_id, can_bus_id_t bus)
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

	diag_mcp_soft_recover(bus);
	mcp2518_refresh_smoke_diag(bus, &g_last_mcp_smoke);
	g_last_mcp_smoke.tec_before = g_last_mcp_smoke.tec;

	diag_mcp_host_sync(bus, &cfg);

	diag_mcp_listen(bus, MCP_BENCH_LISTEN_MS_WAKE);
	diag_mcp_finalize_bench(motor_id, bus, PLANT_DIAG_PROBE_MCP_WAKE);
}

void diag_mcp_disable_sync(uint8_t motor_id, can_bus_id_t bus)
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

	diag_mcp_soft_recover(bus);
	mcp2518_refresh_smoke_diag(bus, &g_last_mcp_smoke);
	g_last_mcp_smoke.tec_before = g_last_mcp_smoke.tec;

	diag_mcp_host_sync(bus, &cfg);

	for (uint8_t i = 0; i < MCP_BENCH_ZERO_GAIN_BURST; i++) {
		if (plugin_pack_tx(&cfg, &desire, &frame) != PLUGIN_OK)
			break;
		(void)diag_mcp_tx_frame(bus, &frame);
		HAL_Delay(8);
	}

	if (robstride_send_disable(&cfg, &frame) != PLUGIN_OK) {
		g_last_probe.found = false;
		return;
	}
	(void)diag_mcp_tx_frame(bus, &frame);
	HAL_Delay(50);

	diag_mcp_listen(bus, 120u);
	diag_mcp_finalize_bench(motor_id, bus, PLANT_DIAG_PROBE_MCP_DISABLE);
}
