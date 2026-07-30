#include "plant/actuator.h"
#include "plant/plugin_schema/plugin_table.h"
#include "plant/can/can_router.h"
#include "plant/can/mcp2518fd.h"
#include "plant/can/spi_can_router.h"
#include "plant/plugins/robstride.h"
#include "plant/plugins/damiao.h"
#include "plant/plugins/cubemars.h"
#include "plant/plugins/zeroerr.h"
#include "plant/plant_diag.h"
#include "plant/plant_command.h"
#include "plant/rx_sim/rx_sim.h"
#include "plant/rx_sim/rx_sim_actuator.h"
#include "host/host_link.h"
#include "plant/plant_crit.h"
#include <math.h>
#include <string.h>

static actuator_desire_t actuator_desire_stage[ACTUATOR_COUNT];
static actuator_state_t  actuator_state_stage[ACTUATOR_COUNT];
static volatile bool     actuator_desire_pending;
/* Buses polled by the last actuator_apply_desire(); end-of-lap diag skips
 * these so all×25 does not double-flush FDCAN every superloop lap. */
static uint32_t          s_last_apply_poll_buses;

actuator_config_t actuator_table[ACTUATOR_COUNT];
actuator_desire_t actuator_desire_live[ACTUATOR_COUNT];
actuator_state_t  actuator_state_live[ACTUATOR_COUNT];

/* Per-bus slot fan-out for RX dispatch. actuator_table[i].bus only changes on
 * CFG apply (init / factory defaults / NVM load / host CFG SET), never per
 * tick, so actuator_dispatch_bus_rx() has no business re-scanning all
 * ACTUATOR_COUNT slots for every RX frame on every bus. Rebuild is O(25) and
 * only needs to run on those CFG-apply edges, not in the hot path. */
static uint8_t s_bus_slot_idx[CAN_BACKEND_COUNT][ACTUATOR_COUNT];
static uint8_t s_bus_slot_count[CAN_BACKEND_COUNT];

void actuator_rebuild_bus_index(void)
{
	memset(s_bus_slot_count, 0, sizeof(s_bus_slot_count));

	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		can_bus_id_t bus;

		if (!actuator_table[i].enabled)
			continue;

		bus = actuator_table[i].bus;
		if ((uint8_t)bus >= CAN_BACKEND_COUNT)
			continue;

		s_bus_slot_idx[bus][s_bus_slot_count[bus]] = i;
		s_bus_slot_count[bus]++;
	}
}

void actuator_init(void)
{
	memset(actuator_table, 0, sizeof(actuator_table));
	memset(actuator_desire_live, 0, sizeof(actuator_desire_live));
	memset(actuator_state_live, 0, sizeof(actuator_state_live));
	memset(actuator_desire_stage, 0, sizeof(actuator_desire_stage));
	memset(actuator_state_stage, 0, sizeof(actuator_state_stage));
	actuator_desire_pending = false;
	actuator_rebuild_bus_index();
}

void actuator_command_mount(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return;

	plant_crit_enter();
	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++)
		actuator_desire_stage[i] = cmd->actuator_commands[i];
	actuator_desire_pending = true;
	plant_crit_exit();
}

void actuator_desire_clear(void)
{
	plant_crit_enter();
	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++)
		memset(&actuator_desire_live[i], 0, sizeof(actuator_desire_t));
	memset(actuator_desire_stage, 0, sizeof(actuator_desire_stage));
	actuator_desire_pending = false;
	plant_crit_exit();
}

bool actuator_any_non_idle_live(void)
{
	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		const actuator_desire_t *d = &actuator_desire_live[i];

		if (!actuator_table[i].enabled)
			continue;
		if (d->kp > 0.01f || d->kd > 0.01f)
			return true;
		if (fabsf(d->velocity) > 0.01f || fabsf(d->torque) > 0.01f)
			return true;
	}

	return false;
}

void plant_recovery_all(void)
{
	can_frame_t frame;
	uint8_t mcp_rails = 0u;

	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		if (!actuator_table[i].enabled)
			continue;

		if (actuator_table[i].protocol == PROTO_ROBSTRIDE) {
			if (robstride_send_reset(&actuator_table[i], &frame) != PLUGIN_OK)
				continue;
			/* Non-blocking on MCP — send_now/mcp2518_send uses HAL_Delay and
			 * pegs the superloop (multi-tick backlog under burst=1). Enqueue +
			 * bounded prepare/flush below; plant tick drains any remainder. */
			(void)can_tx_enqueue(actuator_table[i].bus, &frame);
			if (actuator_table[i].bus >= CAN_BUS_CH4)
				mcp_rails |= (uint8_t)(1u << (uint8_t)(actuator_table[i].bus - CAN_BUS_CH4));
			continue;
		}

		if (actuator_table[i].protocol == PROTO_DAMIAO) {
			damiao_reset_enable_latch(i);
			if (damiao_send_disable(&actuator_table[i], &frame) == PLUGIN_OK)
				(void)can_tx_enqueue(actuator_table[i].bus, &frame);
			continue;
		}

		if (actuator_table[i].protocol == PROTO_CUBEMARS) {
			cubemars_reset_enable_latch(i);
			if (cubemars_send_disable(&actuator_table[i], &frame) == PLUGIN_OK)
				(void)can_tx_enqueue(actuator_table[i].bus, &frame);
			continue;
		}

		if (actuator_table[i].protocol == PROTO_ZEROERR) {
			zeroerr_reset_slot(i);
			if (zeroerr_send_shutdown(&actuator_table[i], &frame))
				(void)can_tx_enqueue(actuator_table[i].bus, &frame);
		}
	}

	/* Push queued MCP resets onto the 1-deep TXQ without blocking waits. */
	for (uint8_t pass = 0u; pass < 4u; pass++) {
		for (uint8_t rail = 0u; rail < 3u; rail++) {
			can_bus_id_t bus;
			can_status_t st;

			if ((mcp_rails & (1u << rail)) == 0u)
				continue;
			bus = (can_bus_id_t)(CAN_BUS_CH4 + rail);
			mcp2518_prepare_tx(bus);
			st = spi_can_router_tx_flush(bus);
			if (st == CAN_ERR_EMPTY)
				mcp_rails &= (uint8_t)~(1u << rail);
		}
		if (mcp_rails == 0u)
			break;
	}

	can_router_poll();
	plant_diag_release_actuator_can();
	actuator_desire_clear();
	rx_sim_clear();
}

static bool actuator_desire_is_idle(const actuator_desire_t *d)
{
	if (d->kp > 0.01f || d->kd > 0.01f)
		return false;
	if (fabsf(d->velocity) > 0.01f || fabsf(d->torque) > 0.01f)
		return false;
	return true;
}

static bool actuator_desire_is_blank(const actuator_desire_t *d)
{
	return actuator_desire_is_idle(d) && d->position == 0.0f;
}

static void actuator_dispatch_bus_rx(can_bus_id_t bus)
{
	can_frame_t frame;
	bool damiao_had_rx[ACTUATOR_COUNT];
	uint8_t count;

	memset(damiao_had_rx, 0, sizeof(damiao_had_rx));

	if ((uint8_t)bus >= CAN_BACKEND_COUNT)
		return;
	count = s_bus_slot_count[bus];

	while (can_rx_pop(bus, &frame) == CAN_OK) {
		for (uint8_t k = 0; k < count; k++) {
			uint8_t i = s_bus_slot_idx[bus][k];

			/* enabled/bus already guaranteed by the index build; re-check
			 * enabled only in case a CFG SET landed between rebuild and this
			 * dispatch without a rebuild in between (defensive, not expected). */
			if (!actuator_table[i].enabled)
				continue;

			if (actuator_table[i].protocol == PROTO_ROBSTRIDE) {
				robstride_on_rx_frame(&actuator_table[i], i, &frame,
				                      &actuator_state_live[i]);
				continue;
			}

			if (actuator_table[i].protocol == PROTO_DAMIAO) {
				/* Only mark had_rx on a real match. Marking every Damiao slot on
				 * every CH1 frame clears enable-latch via post_rx when fault is
				 * still 0 (common for the far end of a daisy) and strands the
				 * slot in clear/enable instead of MIT tracking. */
				if (plugin_parse_rx(&actuator_table[i], &frame,
				                    &actuator_state_live[i]) == PLUGIN_OK)
					damiao_had_rx[i] = true;
				continue;
			}

			if (actuator_table[i].protocol == PROTO_ZEROERR) {
				zeroerr_on_rx_frame(&actuator_table[i], i, &frame,
				                    &actuator_state_live[i]);
				continue;
			}

			if (actuator_table[i].protocol == PROTO_CUBEMARS) {
				/* Direct/immediate, unlike Damiao's deferred had_rx array —
				 * the CubeMars enable latch is TX-driven only (see
				 * cubemars.c), so there is no daisy-chain hazard from
				 * reacting to RX content here. */
				cubemars_on_rx_frame(&actuator_table[i], i, &frame,
				                     &actuator_state_live[i]);
				continue;
			}

			(void)plugin_parse_rx(&actuator_table[i], &frame,
			                      &actuator_state_live[i]);
		}
	}

	for (uint8_t k = 0; k < count; k++) {
		uint8_t i = s_bus_slot_idx[bus][k];

		if (damiao_had_rx[i])
			damiao_post_rx_dispatch(i, true);
	}
}

void actuator_apply_desire(void)
{
	uint32_t poll_buses = 0u;

	plant_crit_enter();
	if (actuator_desire_pending) {
		for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
			actuator_desire_live[i] = actuator_desire_stage[i];
			if (actuator_table[i].enabled &&
			    actuator_table[i].protocol == PROTO_ROBSTRIDE)
				robstride_host_desire_updated(i, &actuator_desire_live[i]);
		}
		actuator_desire_pending = false;
	}
	plant_crit_exit();

	if (!plant_runtime_actuator_can_apply()) {
		s_last_apply_poll_buses = 0u;
		return;
	}

	uint32_t commanded_buses = 0u;

	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		const actuator_desire_t *d = &actuator_desire_live[i];

		if (!actuator_table[i].enabled)
			continue;
		if (!actuator_desire_is_blank(d))
			commanded_buses |= (1u << (unsigned)actuator_table[i].bus);
	}

	robstride_plant_tick_begin(commanded_buses);

	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		const actuator_desire_t *desire;
		can_bus_id_t bus;

		if (!actuator_table[i].enabled)
			continue;

		desire = &actuator_desire_live[i];
		bus = actuator_table[i].bus;

		/* Blank on a bus with no commanded slot — skip unless all-idle sync.
		 * Same policy for FDCAN (CH1–3) and MCP (CH4–6): true blank (p=0,
		 * idle gains) is not applied when another bus is active. Damiao /
		 * CubeMars stay exempt so enable-latch / MIT keep running while idle.
		 * Idle-anchor (p!=0, kp=0) is non-blank and keeps MCP in the path. */
		if (actuator_table[i].protocol != PROTO_DAMIAO &&
		    actuator_table[i].protocol != PROTO_CUBEMARS &&
		    actuator_desire_is_blank(desire) &&
		    commanded_buses != 0u &&
		    (commanded_buses & (1u << (unsigned)bus)) == 0u)
			continue;

		poll_buses |= (1u << (unsigned)bus);

		if (actuator_table[i].protocol == PROTO_ROBSTRIDE) {
			robstride_apply_cycle(&actuator_table[i], desire,
			                      &actuator_state_live[i], i);
			rx_sim_actuator_on_apply(&actuator_table[i], desire);
			continue;
		}

		if (actuator_table[i].protocol == PROTO_DAMIAO) {
			damiao_apply_cycle(&actuator_table[i], desire,
			                   &actuator_state_live[i]);
			continue;
		}

		if (actuator_table[i].protocol == PROTO_CUBEMARS) {
			cubemars_apply_cycle(&actuator_table[i], desire,
			                    &actuator_state_live[i]);
			continue;
		}

		if (actuator_table[i].protocol == PROTO_ZEROERR) {
			zeroerr_apply_cycle(&actuator_table[i], desire,
			                    &actuator_state_live[i]);
			continue;
		}

		can_frame_t tx;
		if (plugin_pack_tx(&actuator_table[i], desire, &tx) == PLUGIN_OK)
			(void)can_tx_enqueue(bus, &tx);
	}

	/* One prepare_tx + flush per MCP bus that enqueued this tick. */
	robstride_mcp_flush_pending();

	/* Poll every commanded bus each tick. MCP RX is INT-gated when idle;
	 * busy-path SPI was cut via batched RAM + TXQ STA dedupe — RR≤3 was a
	 * workaround when empty FIFOSTA + multi-rail SPI starved USB FB. */
	{
		uint8_t buses[CAN_BACKEND_COUNT];
		uint8_t n = 0u;

		for (can_bus_id_t bus = 0; bus < CAN_BACKEND_COUNT; bus++) {
			if ((poll_buses & (1u << (unsigned)bus)) != 0u)
				buses[n++] = (uint8_t)bus;
		}

		for (uint8_t i = 0; i < n; i++) {
			can_bus_id_t bus = (can_bus_id_t)buses[i];

			can_router_poll_bus(bus);
			actuator_dispatch_bus_rx(bus);
		}
	}

	s_last_apply_poll_buses = poll_buses;
}

void actuator_capture_state(void)
{
	plant_crit_enter();
	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++)
		actuator_state_stage[i] = actuator_state_live[i];
	plant_crit_exit();
}

void actuator_feedback_snapshot(host_actuator_feedback_t *dst, uint8_t count)
{
	if (dst == NULL || count == 0)
		return;

	uint8_t n = (count < ACTUATOR_COUNT) ? count : ACTUATOR_COUNT;

	plant_crit_enter();
	for (uint8_t i = 0; i < n; i++)
		dst[i] = actuator_state_stage[i];
	plant_crit_exit();

	for (uint8_t i = 0; i < n; i++) {
		const actuator_config_t *cfg = &actuator_table[i];
		bool fb_valid = (dst[i].fault == 0u) || cfg->enabled;
		dst[i].meta = HOST_ACT_META_PACK(
			(uint16_t)cfg->protocol,
			(uint16_t)cfg->bus,
			(uint16_t)(cfg->motor_id & 0xFFu),
			cfg->enabled,
			fb_valid);
	}

	for (uint8_t i = n; i < count; i++)
		memset(&dst[i], 0, sizeof(dst[i]));
}

uint32_t actuator_last_apply_poll_buses(void)
{
	return s_last_apply_poll_buses;
}
