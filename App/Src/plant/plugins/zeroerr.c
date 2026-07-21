#include "plant/plugins/zeroerr.h"
#include "plant/plugin_schema/plugin.h"
#include "plant/can/canopen.h"
#include "plant/can/can_router.h"
#include "plant/actuator.h"
#include "main.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ZEROERR_SDO_TIMEOUT_MS   30u
#define ZEROERR_BOOT_GAP_MS      5u
#define ZEROERR_RETRY_MS         200u

typedef enum {
	ZE_PHASE_IDLE = 0,
	ZE_PHASE_NMT_STOP,
	ZE_PHASE_NMT_RESET,
	ZE_PHASE_WAIT_RESET,
	ZE_PHASE_MODE_PP,
	ZE_PHASE_TXPDO_DISABLE,
	ZE_PHASE_TXPDO_CLEAR,
	ZE_PHASE_TXPDO_MAP1,
	ZE_PHASE_TXPDO_MAP2,
	ZE_PHASE_TXPDO_COUNT,
	ZE_PHASE_TXPDO_TYPE,
	ZE_PHASE_TXPDO_ENABLE,
	ZE_PHASE_RXPDO_DISABLE,
	ZE_PHASE_RXPDO_CLEAR,
	ZE_PHASE_RXPDO_MAP1,
	ZE_PHASE_RXPDO_MAP2,
	ZE_PHASE_RXPDO_COUNT,
	ZE_PHASE_RXPDO_TYPE,
	ZE_PHASE_RXPDO_ENABLE,
	ZE_PHASE_NMT_START,
	ZE_PHASE_ENABLE_06,
	ZE_PHASE_ENABLE_07,
	ZE_PHASE_ENABLE_0F,
	ZE_PHASE_OPERATIONAL,
	ZE_PHASE_FAULT,
} zeroerr_phase_t;

typedef struct {
	zeroerr_phase_t phase;
	uint32_t next_ms;
	uint16_t statusword;
	int32_t  actual_counts;
	int32_t  cmd_counts;
	int32_t  last_cmd_counts;
	uint16_t controlword;
	bool     fb_valid;
	bool     have_last_cmd;
} zeroerr_slot_t;

static zeroerr_slot_t s_ze[ACTUATOR_COUNT];

int32_t zeroerr_rad_to_counts(float rad)
{
	float counts = rad * ((float)ZEROERR_ENCODER_RES / (float)(2.0 * M_PI));

	if (counts >= 0.0f)
		return (int32_t)(counts + 0.5f);
	return (int32_t)(counts - 0.5f);
}

float zeroerr_counts_to_rad(int32_t counts)
{
	return ((float)counts) * ((float)(2.0 * M_PI) / (float)ZEROERR_ENCODER_RES);
}

static uint8_t zeroerr_node(const actuator_config_t *cfg)
{
	uint8_t n = (uint8_t)(cfg->motor_id & 0x7Fu);

	return (n == 0u) ? 1u : n;
}

static uint8_t zeroerr_actuator_slot(const actuator_config_t *cfg)
{
	if (cfg == NULL)
		return ACTUATOR_COUNT;

	if (cfg >= &actuator_table[0] && cfg < &actuator_table[ACTUATOR_COUNT])
		return (uint8_t)(cfg - actuator_table);

	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		if (actuator_table[i].bus == cfg->bus &&
		    actuator_table[i].motor_id == cfg->motor_id &&
		    actuator_table[i].protocol == PROTO_ZEROERR)
			return i;
	}
	return ACTUATOR_COUNT;
}

static bool zeroerr_desire_idle(const actuator_desire_t *d)
{
	if (d == NULL)
		return true;
	if (d->kp > 0.01f || d->kd > 0.01f)
		return false;
	if (fabsf(d->velocity) > 0.01f || fabsf(d->torque) > 0.01f)
		return false;
	return true;
}

void zeroerr_reset_slot(uint8_t slot)
{
	if (slot >= ACTUATOR_COUNT)
		return;
	memset(&s_ze[slot], 0, sizeof(s_ze[slot]));
}

bool zeroerr_send_shutdown(const actuator_config_t *cfg, can_frame_t *frame_out)
{
	if (cfg == NULL || frame_out == NULL || !cfg->enabled)
		return false;

	canopen_pack_rxpdo1_cw_pos(frame_out, zeroerr_node(cfg),
	                           ZEROERR_CW_SHUTDOWN, 0);
	return true;
}

bool zeroerr_read_identity(can_bus_id_t bus, uint8_t node_id,
                           uint32_t *vendor, uint32_t *product, uint32_t *revision,
                           uint32_t sdo_timeout_ms)
{
	uint32_t v = 0, p = 0, r = 0;

	if (!canopen_sdo_read_u32(bus, node_id, 0x1018u, 1u, &v, sdo_timeout_ms))
		return false;
	if (!canopen_sdo_read_u32(bus, node_id, 0x1018u, 2u, &p, sdo_timeout_ms))
		return false;
	if (!canopen_sdo_read_u32(bus, node_id, 0x1018u, 3u, &r, sdo_timeout_ms))
		return false;
	if (vendor != NULL)
		*vendor = v;
	if (product != NULL)
		*product = p;
	if (revision != NULL)
		*revision = r;
	return true;
}

/*
 * Blocking PDO1 remap + NMT start — mirrors eRobControl_PP.configure_pdo().
 * Call from DEBUG/bench only (uses SDO waits).
 */
bool zeroerr_boot_blocking(can_bus_id_t bus, uint8_t node_id, uint32_t sdo_timeout_ms)
{
	uint32_t txpdo_cob = (uint32_t)canopen_cob_txpdo1(node_id);
	uint32_t rxpdo_cob = (uint32_t)canopen_cob_rxpdo1(node_id);

	if (sdo_timeout_ms == 0u)
		sdo_timeout_ms = ZEROERR_SDO_TIMEOUT_MS;

	(void)canopen_nmt_send(bus, CANOPEN_NMT_STOP, node_id);
	(void)can_tx_flush(bus);
	(void)canopen_nmt_send(bus, CANOPEN_NMT_RESET_COMM, node_id);
	(void)can_tx_flush(bus);

	/* Brief settle — only allowed on blocking diag path. */
	{
		uint32_t t0 = HAL_GetTick();

		while ((HAL_GetTick() - t0) < 50u)
			can_router_poll_bus(bus);
	}

	(void)canopen_nmt_send(bus, CANOPEN_NMT_PREOP, node_id);
	(void)can_tx_flush(bus);

	if (!canopen_sdo_write_u8(bus, node_id, ZEROERR_IDX_MODE, 0u,
	                          (uint8_t)ZEROERR_MODE_PP, sdo_timeout_ms))
		return false;

	/* TxPDO1: statusword + actual position */
	if (!canopen_sdo_write_u32(bus, node_id, 0x1800u, 1u,
	                           txpdo_cob | CANOPEN_PDO_DISABLE_BIT, sdo_timeout_ms))
		return false;
	if (!canopen_sdo_write_u8(bus, node_id, 0x1A00u, 0u, 0u, sdo_timeout_ms))
		return false;
	if (!canopen_sdo_write_u32(bus, node_id, 0x1A00u, 1u, ZEROERR_MAP_SW, sdo_timeout_ms))
		return false;
	if (!canopen_sdo_write_u32(bus, node_id, 0x1A00u, 2u, ZEROERR_MAP_ACT_POS, sdo_timeout_ms))
		return false;
	if (!canopen_sdo_write_u8(bus, node_id, 0x1A00u, 0u, 2u, sdo_timeout_ms))
		return false;
	if (!canopen_sdo_write_u8(bus, node_id, 0x1800u, 2u, 0xFFu, sdo_timeout_ms))
		return false;
	if (!canopen_sdo_write_u32(bus, node_id, 0x1800u, 1u, txpdo_cob, sdo_timeout_ms))
		return false;

	/* RxPDO1: controlword + target position */
	if (!canopen_sdo_write_u32(bus, node_id, 0x1400u, 1u,
	                           rxpdo_cob | CANOPEN_PDO_DISABLE_BIT, sdo_timeout_ms))
		return false;
	if (!canopen_sdo_write_u8(bus, node_id, 0x1600u, 0u, 0u, sdo_timeout_ms))
		return false;
	if (!canopen_sdo_write_u32(bus, node_id, 0x1600u, 1u, ZEROERR_MAP_CW, sdo_timeout_ms))
		return false;
	if (!canopen_sdo_write_u32(bus, node_id, 0x1600u, 2u, ZEROERR_MAP_TARGET_POS, sdo_timeout_ms))
		return false;
	if (!canopen_sdo_write_u8(bus, node_id, 0x1600u, 0u, 2u, sdo_timeout_ms))
		return false;
	if (!canopen_sdo_write_u8(bus, node_id, 0x1400u, 2u, 0xFFu, sdo_timeout_ms))
		return false;
	if (!canopen_sdo_write_u32(bus, node_id, 0x1400u, 1u, rxpdo_cob, sdo_timeout_ms))
		return false;

	if (!canopen_nmt_send(bus, CANOPEN_NMT_START, node_id))
		return false;
	(void)can_tx_flush(bus);
	return true;
}

static void zeroerr_schedule(zeroerr_slot_t *st, zeroerr_phase_t next, uint32_t delay_ms)
{
	st->phase = next;
	st->next_ms = HAL_GetTick() + delay_ms;
}

static void zeroerr_boot_step(const actuator_config_t *cfg, zeroerr_slot_t *st)
{
	can_bus_id_t bus = cfg->bus;
	uint8_t node = zeroerr_node(cfg);
	uint32_t now = HAL_GetTick();
	uint32_t txpdo_cob = (uint32_t)canopen_cob_txpdo1(node);
	uint32_t rxpdo_cob = (uint32_t)canopen_cob_rxpdo1(node);
	const uint32_t tmo = ZEROERR_SDO_TIMEOUT_MS;

	if ((int32_t)(now - st->next_ms) < 0)
		return;

	/*
	 * One SDO/NMT action per plant service when not yet operational.
	 * SDO helper may wait up to ~30 ms — acceptable only during boot; once
	 * ZE_PHASE_OPERATIONAL, apply path is PDO-only.
	 */
	switch (st->phase) {
	case ZE_PHASE_IDLE:
		zeroerr_schedule(st, ZE_PHASE_NMT_STOP, 0u);
		break;
	case ZE_PHASE_NMT_STOP:
		(void)canopen_nmt_send(bus, CANOPEN_NMT_STOP, node);
		(void)can_tx_flush(bus);
		zeroerr_schedule(st, ZE_PHASE_NMT_RESET, ZEROERR_BOOT_GAP_MS);
		break;
	case ZE_PHASE_NMT_RESET:
		(void)canopen_nmt_send(bus, CANOPEN_NMT_RESET_COMM, node);
		(void)can_tx_flush(bus);
		zeroerr_schedule(st, ZE_PHASE_WAIT_RESET, 50u);
		break;
	case ZE_PHASE_WAIT_RESET:
		(void)canopen_nmt_send(bus, CANOPEN_NMT_PREOP, node);
		(void)can_tx_flush(bus);
		zeroerr_schedule(st, ZE_PHASE_MODE_PP, ZEROERR_BOOT_GAP_MS);
		break;
	case ZE_PHASE_MODE_PP:
		if (!canopen_sdo_write_u8(bus, node, ZEROERR_IDX_MODE, 0u,
		                         (uint8_t)ZEROERR_MODE_PP, tmo)) {
			zeroerr_schedule(st, ZE_PHASE_FAULT, ZEROERR_RETRY_MS);
			break;
		}
		zeroerr_schedule(st, ZE_PHASE_TXPDO_DISABLE, ZEROERR_BOOT_GAP_MS);
		break;
	case ZE_PHASE_TXPDO_DISABLE:
		if (!canopen_sdo_write_u32(bus, node, 0x1800u, 1u,
		                          txpdo_cob | CANOPEN_PDO_DISABLE_BIT, tmo)) {
			zeroerr_schedule(st, ZE_PHASE_FAULT, ZEROERR_RETRY_MS);
			break;
		}
		zeroerr_schedule(st, ZE_PHASE_TXPDO_CLEAR, ZEROERR_BOOT_GAP_MS);
		break;
	case ZE_PHASE_TXPDO_CLEAR:
		if (!canopen_sdo_write_u8(bus, node, 0x1A00u, 0u, 0u, tmo)) {
			zeroerr_schedule(st, ZE_PHASE_FAULT, ZEROERR_RETRY_MS);
			break;
		}
		zeroerr_schedule(st, ZE_PHASE_TXPDO_MAP1, ZEROERR_BOOT_GAP_MS);
		break;
	case ZE_PHASE_TXPDO_MAP1:
		if (!canopen_sdo_write_u32(bus, node, 0x1A00u, 1u, ZEROERR_MAP_SW, tmo)) {
			zeroerr_schedule(st, ZE_PHASE_FAULT, ZEROERR_RETRY_MS);
			break;
		}
		zeroerr_schedule(st, ZE_PHASE_TXPDO_MAP2, ZEROERR_BOOT_GAP_MS);
		break;
	case ZE_PHASE_TXPDO_MAP2:
		if (!canopen_sdo_write_u32(bus, node, 0x1A00u, 2u, ZEROERR_MAP_ACT_POS, tmo)) {
			zeroerr_schedule(st, ZE_PHASE_FAULT, ZEROERR_RETRY_MS);
			break;
		}
		zeroerr_schedule(st, ZE_PHASE_TXPDO_COUNT, ZEROERR_BOOT_GAP_MS);
		break;
	case ZE_PHASE_TXPDO_COUNT:
		if (!canopen_sdo_write_u8(bus, node, 0x1A00u, 0u, 2u, tmo)) {
			zeroerr_schedule(st, ZE_PHASE_FAULT, ZEROERR_RETRY_MS);
			break;
		}
		zeroerr_schedule(st, ZE_PHASE_TXPDO_TYPE, ZEROERR_BOOT_GAP_MS);
		break;
	case ZE_PHASE_TXPDO_TYPE:
		if (!canopen_sdo_write_u8(bus, node, 0x1800u, 2u, 0xFFu, tmo)) {
			zeroerr_schedule(st, ZE_PHASE_FAULT, ZEROERR_RETRY_MS);
			break;
		}
		zeroerr_schedule(st, ZE_PHASE_TXPDO_ENABLE, ZEROERR_BOOT_GAP_MS);
		break;
	case ZE_PHASE_TXPDO_ENABLE:
		if (!canopen_sdo_write_u32(bus, node, 0x1800u, 1u, txpdo_cob, tmo)) {
			zeroerr_schedule(st, ZE_PHASE_FAULT, ZEROERR_RETRY_MS);
			break;
		}
		zeroerr_schedule(st, ZE_PHASE_RXPDO_DISABLE, ZEROERR_BOOT_GAP_MS);
		break;
	case ZE_PHASE_RXPDO_DISABLE:
		if (!canopen_sdo_write_u32(bus, node, 0x1400u, 1u,
		                          rxpdo_cob | CANOPEN_PDO_DISABLE_BIT, tmo)) {
			zeroerr_schedule(st, ZE_PHASE_FAULT, ZEROERR_RETRY_MS);
			break;
		}
		zeroerr_schedule(st, ZE_PHASE_RXPDO_CLEAR, ZEROERR_BOOT_GAP_MS);
		break;
	case ZE_PHASE_RXPDO_CLEAR:
		if (!canopen_sdo_write_u8(bus, node, 0x1600u, 0u, 0u, tmo)) {
			zeroerr_schedule(st, ZE_PHASE_FAULT, ZEROERR_RETRY_MS);
			break;
		}
		zeroerr_schedule(st, ZE_PHASE_RXPDO_MAP1, ZEROERR_BOOT_GAP_MS);
		break;
	case ZE_PHASE_RXPDO_MAP1:
		if (!canopen_sdo_write_u32(bus, node, 0x1600u, 1u, ZEROERR_MAP_CW, tmo)) {
			zeroerr_schedule(st, ZE_PHASE_FAULT, ZEROERR_RETRY_MS);
			break;
		}
		zeroerr_schedule(st, ZE_PHASE_RXPDO_MAP2, ZEROERR_BOOT_GAP_MS);
		break;
	case ZE_PHASE_RXPDO_MAP2:
		if (!canopen_sdo_write_u32(bus, node, 0x1600u, 2u, ZEROERR_MAP_TARGET_POS, tmo)) {
			zeroerr_schedule(st, ZE_PHASE_FAULT, ZEROERR_RETRY_MS);
			break;
		}
		zeroerr_schedule(st, ZE_PHASE_RXPDO_COUNT, ZEROERR_BOOT_GAP_MS);
		break;
	case ZE_PHASE_RXPDO_COUNT:
		if (!canopen_sdo_write_u8(bus, node, 0x1600u, 0u, 2u, tmo)) {
			zeroerr_schedule(st, ZE_PHASE_FAULT, ZEROERR_RETRY_MS);
			break;
		}
		zeroerr_schedule(st, ZE_PHASE_RXPDO_TYPE, ZEROERR_BOOT_GAP_MS);
		break;
	case ZE_PHASE_RXPDO_TYPE:
		if (!canopen_sdo_write_u8(bus, node, 0x1400u, 2u, 0xFFu, tmo)) {
			zeroerr_schedule(st, ZE_PHASE_FAULT, ZEROERR_RETRY_MS);
			break;
		}
		zeroerr_schedule(st, ZE_PHASE_RXPDO_ENABLE, ZEROERR_BOOT_GAP_MS);
		break;
	case ZE_PHASE_RXPDO_ENABLE:
		if (!canopen_sdo_write_u32(bus, node, 0x1400u, 1u, rxpdo_cob, tmo)) {
			zeroerr_schedule(st, ZE_PHASE_FAULT, ZEROERR_RETRY_MS);
			break;
		}
		zeroerr_schedule(st, ZE_PHASE_NMT_START, ZEROERR_BOOT_GAP_MS);
		break;
	case ZE_PHASE_NMT_START:
		(void)canopen_nmt_send(bus, CANOPEN_NMT_START, node);
		(void)can_tx_flush(bus);
		zeroerr_schedule(st, ZE_PHASE_ENABLE_06, ZEROERR_BOOT_GAP_MS);
		break;
	case ZE_PHASE_ENABLE_06:
		st->controlword = ZEROERR_CW_SHUTDOWN;
		zeroerr_schedule(st, ZE_PHASE_ENABLE_07, 20u);
		break;
	case ZE_PHASE_ENABLE_07:
		st->controlword = ZEROERR_CW_SWITCH_ON;
		zeroerr_schedule(st, ZE_PHASE_ENABLE_0F, 20u);
		break;
	case ZE_PHASE_ENABLE_0F:
		st->controlword = ZEROERR_CW_ENABLE;
		st->have_last_cmd = false;
		zeroerr_schedule(st, ZE_PHASE_OPERATIONAL, 0u);
		break;
	case ZE_PHASE_FAULT:
		zeroerr_schedule(st, ZE_PHASE_IDLE, ZEROERR_RETRY_MS);
		break;
	case ZE_PHASE_OPERATIONAL:
	default:
		break;
	}
}

void zeroerr_apply_cycle(const actuator_config_t *cfg,
                         const actuator_desire_t *desire,
                         actuator_state_t *state_out)
{
	uint8_t slot;
	zeroerr_slot_t *st;
	can_frame_t frame;
	uint8_t node;

	if (cfg == NULL || desire == NULL || !cfg->enabled ||
	    cfg->protocol != PROTO_ZEROERR)
		return;

	slot = zeroerr_actuator_slot(cfg);
	if (slot >= ACTUATOR_COUNT)
		return;

	st = &s_ze[slot];
	node = zeroerr_node(cfg);

	/* kp≈0 → shutdown / no boot spam (plan blank policy). */
	if (zeroerr_desire_idle(desire)) {
		if (st->phase == ZE_PHASE_OPERATIONAL) {
			canopen_pack_rxpdo1_cw_pos(&frame, node, ZEROERR_CW_SHUTDOWN,
			                          st->fb_valid ? st->actual_counts : 0);
			(void)can_tx_enqueue(cfg->bus, &frame);
			st->phase = ZE_PHASE_IDLE;
			st->have_last_cmd = false;
		}
		return;
	}

	if (st->phase != ZE_PHASE_OPERATIONAL) {
		zeroerr_boot_step(cfg, st);
		/* During enable steps, still stream current cw if we have a target. */
		if (st->phase >= ZE_PHASE_ENABLE_06 && st->phase <= ZE_PHASE_ENABLE_0F) {
			st->cmd_counts = zeroerr_rad_to_counts(desire->position);
			canopen_pack_rxpdo1_cw_pos(&frame, node, st->controlword, st->cmd_counts);
			(void)can_tx_enqueue(cfg->bus, &frame);
		}
		return;
	}

	st->cmd_counts = zeroerr_rad_to_counts(desire->position);

	/*
	 * PP new-setpoint: rising edge on bit4 (0x0F -> 0x1F) when target changes,
	 * then hold 0x0F so the next move can edge again.
	 */
	if (!st->have_last_cmd || st->cmd_counts != st->last_cmd_counts) {
		st->controlword = ZEROERR_CW_ENABLE_NEW;
		st->last_cmd_counts = st->cmd_counts;
		st->have_last_cmd = true;
	} else {
		st->controlword = ZEROERR_CW_ENABLE;
	}

	canopen_pack_rxpdo1_cw_pos(&frame, node, st->controlword, st->cmd_counts);
	(void)can_tx_enqueue(cfg->bus, &frame);

	if (state_out != NULL && st->fb_valid) {
		state_out->position = zeroerr_counts_to_rad(st->actual_counts);
		state_out->fault = (uint32_t)st->statusword;
	}
}

void zeroerr_on_rx_frame(const actuator_config_t *cfg, uint8_t slot,
                         const can_frame_t *frame, actuator_state_t *state_out)
{
	uint16_t sw;
	int32_t pos;
	zeroerr_slot_t *st;

	if (cfg == NULL || frame == NULL || slot >= ACTUATOR_COUNT)
		return;

	st = &s_ze[slot];
	if (!canopen_parse_txpdo1_sw_pos(frame, zeroerr_node(cfg), &sw, &pos))
		return;

	st->statusword = sw;
	st->actual_counts = pos;
	st->fb_valid = true;

	if (state_out != NULL) {
		state_out->position = zeroerr_counts_to_rad(pos);
		state_out->velocity = 0.0f;
		state_out->torque = 0.0f;
		state_out->temperature = 0.0f;
		state_out->fault = (uint32_t)sw;
	}
}

static plugin_status_t zeroerr_pack_tx(const actuator_config_t *cfg,
                                       const actuator_desire_t *desire,
                                       can_frame_t *frame_out)
{
	/* Prefer zeroerr_apply_cycle — pack_tx only for generic enqueue fallback. */
	if (cfg == NULL || desire == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;
	if (!cfg->enabled || zeroerr_desire_idle(desire))
		return PLUGIN_ERR_UNSUPPORTED;

	canopen_pack_rxpdo1_cw_pos(frame_out, zeroerr_node(cfg),
	                           ZEROERR_CW_ENABLE,
	                           zeroerr_rad_to_counts(desire->position));
	return PLUGIN_OK;
}

static plugin_status_t zeroerr_parse_rx(const actuator_config_t *cfg,
                                        const can_frame_t *frame_in,
                                        actuator_state_t *state_out)
{
	uint16_t sw;
	int32_t pos;

	if (cfg == NULL || frame_in == NULL || state_out == NULL)
		return PLUGIN_ERR_PARAM;
	if (!canopen_parse_txpdo1_sw_pos(frame_in, zeroerr_node(cfg), &sw, &pos))
		return PLUGIN_ERR_UNSUPPORTED;

	state_out->position = zeroerr_counts_to_rad(pos);
	state_out->velocity = 0.0f;
	state_out->torque = 0.0f;
	state_out->temperature = 0.0f;
	state_out->fault = (uint32_t)sw;
	return PLUGIN_OK;
}

const plugin_ops_t zeroerr_ops = {
	.pack_tx = zeroerr_pack_tx,
	.parse_rx = zeroerr_parse_rx,
};
