#include "plant/plugins/zeroerr.h"
#include "plant/plugin_schema/plugin.h"
#include "plant/can/canopen.h"
#include "plant/can/can_router.h"
#include "plant/actuator.h"
#include "plant/plant_diag.h"
#include "main.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ZEROERR_SDO_TIMEOUT_MS   30u
#define ZEROERR_BOOT_GAP_MS      5u
#define ZEROERR_RETRY_MS         200u
/* eRob Rotary Actuator User Manual V3.42 Ch.7: brake activation ~150ms,
 * recommended wait after Enable Operation before motion commands >=500ms. */
#define ZEROERR_ENABLE_SETTLE_MS 500u
/* Statusword handshake on the 06/07/0F enable walk (CiA402 §6.3 state
 * table) — how often to re-check/re-send while waiting for the drive to
 * confirm a transition, and how long to wait before giving up and cold-
 * restarting via ZE_PHASE_FAULT rather than assuming the transition
 * silently worked. */
#define ZEROERR_STATE_POLL_MS    20u
#define ZEROERR_STATE_RESEND_MS  100u
#define ZEROERR_STATE_TIMEOUT_MS 1500u

typedef enum {
	ZE_PHASE_IDLE = 0,
	ZE_PHASE_NMT_STOP,
	ZE_PHASE_NMT_RESET,
	ZE_PHASE_WAIT_RESET,
	ZE_PHASE_MODE_SET,
	ZE_PHASE_PROF_VEL,
	ZE_PHASE_PROF_ACC,
	ZE_PHASE_PROF_DEC,
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
	ZE_PHASE_WAIT_RTSO,
	ZE_PHASE_ENABLE_07,
	ZE_PHASE_WAIT_SO,
	ZE_PHASE_ENABLE_0F,
	ZE_PHASE_WAIT_OE,
	ZE_PHASE_ENABLE_SETTLE,
	ZE_PHASE_OPERATIONAL,
	ZE_PHASE_FAULT,
} zeroerr_phase_t;

typedef struct {
	zeroerr_phase_t phase;
	uint32_t next_ms;
	uint16_t statusword;
	int32_t  actual_counts;
	int32_t  cmd_counts;
	uint16_t controlword;
	bool     fb_valid;
	/* ZE_PHASE_WAIT_* bookkeeping — see zeroerr_wait_state(). */
	uint32_t wait_deadline_ms;
	uint32_t wait_last_send_ms;
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

/* SDO read of 0x6064 (Position Actual Value, sub 0) — see zeroerr.h for the
 * "safe at any time, no enable needed" rationale. Byte-matches the vendor
 * manual §3.6.2 worked example exactly. */
bool zeroerr_read_position(can_bus_id_t bus, uint8_t node_id, float *position_rad,
                           uint32_t sdo_timeout_ms)
{
	uint32_t raw = 0u;

	if (position_rad == NULL)
		return false;
	if (sdo_timeout_ms == 0u)
		sdo_timeout_ms = ZEROERR_SDO_TIMEOUT_MS;

	if (!canopen_sdo_read_u32(bus, node_id, ZEROERR_IDX_POS_ACT, 0u, &raw, sdo_timeout_ms))
		return false;

	*position_rad = zeroerr_counts_to_rad((int32_t)raw);
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

	/* CSP (Cyclic Synchronous Position), not Profile Position — see
	 * zeroerr_apply_cycle's OPERATIONAL streaming path. */
	if (!canopen_sdo_write_u8(bus, node_id, ZEROERR_IDX_MODE, 0u,
	                          (uint8_t)ZEROERR_MODE_CSP, sdo_timeout_ms))
		return false;

	/* Profile velocity/accel/decel — previously left at whatever the drive's
	 * own NVM/factory default was; explicit placeholders here, tune on bench. */
	if (!canopen_sdo_write_u32(bus, node_id, ZEROERR_IDX_PROF_VEL, 0u,
	                           ZEROERR_DEFAULT_PROF_VEL, sdo_timeout_ms))
		return false;
	if (!canopen_sdo_write_u32(bus, node_id, ZEROERR_IDX_PROF_ACC, 0u,
	                           ZEROERR_DEFAULT_PROF_ACC, sdo_timeout_ms))
		return false;
	if (!canopen_sdo_write_u32(bus, node_id, ZEROERR_IDX_PROF_DEC, 0u,
	                           ZEROERR_DEFAULT_PROF_DEC, sdo_timeout_ms))
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

/* Sends the current controlword while holding target = last known actual
 * position (falls back to 0 if no feedback yet, same fallback as
 * zeroerr_send_shutdown). Used only during the enable walk (06/07/0F/settle)
 * so no motion is asserted before the drive is confirmed enabled and settled. */
static void zeroerr_send_hold_frame(const actuator_config_t *cfg, zeroerr_slot_t *st,
                                    uint8_t node)
{
	can_frame_t frame;
	int32_t hold_counts = st->fb_valid ? st->actual_counts : 0;

	canopen_pack_rxpdo1_cw_pos(&frame, node, st->controlword, hold_counts);
	(void)can_tx_enqueue(cfg->bus, &frame);
}

static void zeroerr_wait_begin(zeroerr_slot_t *st, zeroerr_phase_t wait_phase)
{
	uint32_t now = HAL_GetTick();

	st->wait_deadline_ms = now + ZEROERR_STATE_TIMEOUT_MS;
	st->wait_last_send_ms = now;
	zeroerr_schedule(st, wait_phase, ZEROERR_BOOT_GAP_MS);
}

/*
 * CiA402 §6.3 state handshake: after sending a controlword transition,
 * don't just assume it worked after a fixed delay — poll the statusword
 * (arrives async via zeroerr_on_rx_frame) until it reports the expected
 * state, a Fault, or we time out. ``resend_cw`` is re-sent periodically in
 * case the frame that triggered the transition was dropped (CAN is
 * best-effort, no ack at this layer). Timeout/Fault both route to
 * ZE_PHASE_FAULT, which cold-restarts the whole boot from IDLE rather than
 * ever assuming a transition happened when it didn't confirm.
 */
static void zeroerr_wait_state(const actuator_config_t *cfg, zeroerr_slot_t *st,
                               uint8_t node, uint16_t expected_state_masked,
                               uint16_t resend_cw, zeroerr_phase_t on_confirmed)
{
	uint32_t now = HAL_GetTick();

	if (st->fb_valid) {
		if ((st->statusword & ZEROERR_SW_FAULT_BIT) != 0u) {
			zeroerr_schedule(st, ZE_PHASE_FAULT, 0u);
			return;
		}
		if ((st->statusword & ZEROERR_SW_STATE_MASK) == expected_state_masked) {
			zeroerr_schedule(st, on_confirmed, 0u);
			return;
		}
	}

	if ((int32_t)(now - st->wait_deadline_ms) >= 0) {
		/* Never confirmed within budget — do not proceed on faith. */
		zeroerr_schedule(st, ZE_PHASE_FAULT, 0u);
		return;
	}

	if ((int32_t)(now - st->wait_last_send_ms) >= (int32_t)ZEROERR_STATE_RESEND_MS) {
		st->controlword = resend_cw;
		zeroerr_send_hold_frame(cfg, st, node);
		st->wait_last_send_ms = now;
	}

	zeroerr_schedule(st, st->phase, ZEROERR_STATE_POLL_MS);
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
		zeroerr_schedule(st, ZE_PHASE_MODE_SET, ZEROERR_BOOT_GAP_MS);
		break;
	case ZE_PHASE_MODE_SET:
		/* CSP (Cyclic Synchronous Position), not Profile Position — see the
		 * long comment in zeroerr_apply_cycle's OPERATIONAL streaming path
		 * for why PP was dropped. */
		if (!canopen_sdo_write_u8(bus, node, ZEROERR_IDX_MODE, 0u,
		                         (uint8_t)ZEROERR_MODE_CSP, tmo)) {
			zeroerr_schedule(st, ZE_PHASE_FAULT, ZEROERR_RETRY_MS);
			break;
		}
		zeroerr_schedule(st, ZE_PHASE_PROF_VEL, ZEROERR_BOOT_GAP_MS);
		break;
	case ZE_PHASE_PROF_VEL:
		if (!canopen_sdo_write_u32(bus, node, ZEROERR_IDX_PROF_VEL, 0u,
		                          ZEROERR_DEFAULT_PROF_VEL, tmo)) {
			zeroerr_schedule(st, ZE_PHASE_FAULT, ZEROERR_RETRY_MS);
			break;
		}
		zeroerr_schedule(st, ZE_PHASE_PROF_ACC, ZEROERR_BOOT_GAP_MS);
		break;
	case ZE_PHASE_PROF_ACC:
		if (!canopen_sdo_write_u32(bus, node, ZEROERR_IDX_PROF_ACC, 0u,
		                          ZEROERR_DEFAULT_PROF_ACC, tmo)) {
			zeroerr_schedule(st, ZE_PHASE_FAULT, ZEROERR_RETRY_MS);
			break;
		}
		zeroerr_schedule(st, ZE_PHASE_PROF_DEC, ZEROERR_BOOT_GAP_MS);
		break;
	case ZE_PHASE_PROF_DEC:
		if (!canopen_sdo_write_u32(bus, node, ZEROERR_IDX_PROF_DEC, 0u,
		                          ZEROERR_DEFAULT_PROF_DEC, tmo)) {
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
		/* Frame sent here (not deferred to the caller) — deferring on a
		 * post-hoc phase-range check previously dropped the ENABLE_0F frame
		 * entirely, since zeroerr_schedule() mutates st->phase in-place
		 * before the caller's range check ran. See eRob CANopen manual
		 * Ch.7/CiA402: each transition (06/07/0F) needs its own frame. */
		st->controlword = ZEROERR_CW_SHUTDOWN;
		zeroerr_send_hold_frame(cfg, st, node);
		zeroerr_wait_begin(st, ZE_PHASE_WAIT_RTSO);
		break;
	case ZE_PHASE_WAIT_RTSO:
		zeroerr_wait_state(cfg, st, node, ZEROERR_SW_STATE_READY_TO_SWITCH_ON,
		                   ZEROERR_CW_SHUTDOWN, ZE_PHASE_ENABLE_07);
		break;
	case ZE_PHASE_ENABLE_07:
		st->controlword = ZEROERR_CW_SWITCH_ON;
		zeroerr_send_hold_frame(cfg, st, node);
		zeroerr_wait_begin(st, ZE_PHASE_WAIT_SO);
		break;
	case ZE_PHASE_WAIT_SO:
		zeroerr_wait_state(cfg, st, node, ZEROERR_SW_STATE_SWITCHED_ON,
		                   ZEROERR_CW_SWITCH_ON, ZE_PHASE_ENABLE_0F);
		break;
	case ZE_PHASE_ENABLE_0F:
		st->controlword = ZEROERR_CW_ENABLE;
		zeroerr_send_hold_frame(cfg, st, node);
		zeroerr_wait_begin(st, ZE_PHASE_WAIT_OE);
		break;
	case ZE_PHASE_WAIT_OE:
		zeroerr_wait_state(cfg, st, node, ZEROERR_SW_STATE_OPERATION_ENABLED,
		                   ZEROERR_CW_ENABLE, ZE_PHASE_ENABLE_SETTLE);
		break;
	case ZE_PHASE_ENABLE_SETTLE:
		/* eRob manual Ch.7: brake activation ~150ms; wait >=500ms after
		 * Enable Operation before issuing motion commands. Hold position,
		 * no new-setpoint edge yet — first real move happens only once
		 * ZE_PHASE_OPERATIONAL's edge-detect path takes over next tick. */
		zeroerr_send_hold_frame(cfg, st, node);
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

	/* kp≈0 → shutdown / no boot spam (plan blank policy).
	 *
	 * Warm re-enable, not a cold reboot: land on ZE_PHASE_ENABLE_06, not
	 * ZE_PHASE_IDLE. NMT is still Operational and PDO1 mapping is still
	 * installed on the device — nothing about a transient idle desire
	 * invalidates either, so re-entering ZE_PHASE_IDLE would needlessly
	 * replay NMT_STOP/NMT_RESET_COMM + the entire PDO1 remap (~20 SDO/NMT
	 * round trips) on every idle->active edge, leaving the joint with no
	 * valid RxPDO1 target for the whole replay instead of just a brief
	 * disable/re-enable. ZE_PHASE_ENABLE_06 only re-walks the controlword
	 * state machine (06->07->0F->settle), which is also what's actually
	 * required after a genuine CiA402 disable. */
	if (zeroerr_desire_idle(desire)) {
		if (st->phase == ZE_PHASE_OPERATIONAL) {
			canopen_pack_rxpdo1_cw_pos(&frame, node, ZEROERR_CW_SHUTDOWN,
			                          st->fb_valid ? st->actual_counts : 0);
			(void)can_tx_enqueue(cfg->bus, &frame);
			zeroerr_schedule(st, ZE_PHASE_ENABLE_06, 0u);
		}
		return;
	}

	if (st->phase != ZE_PHASE_OPERATIONAL) {
		/* Boot/enable frames (including the enable-settle hold) are sent
		 * directly inside zeroerr_boot_step's case bodies now — see
		 * zeroerr_send_hold_frame. Nothing tracks desire->position here;
		 * the first real move happens once ZE_PHASE_OPERATIONAL is reached. */
		zeroerr_boot_step(cfg, st);
		return;
	}

	/* Steady-state fault watch: previously nothing here ever inspected the
	 * Fault bit, so a mid-motion fault went undetected — the drive would
	 * silently drop to Fault/Switch On Disabled while this kept streaming
	 * stale controlwords/setpoints at it. Route to the same cold-restart
	 * (ZE_PHASE_FAULT -> IDLE) the enable-walk timeout uses, and stop
	 * mounting a desire this tick — no motion command follows a detected
	 * fault. */
	if (st->fb_valid && (st->statusword & ZEROERR_SW_FAULT_BIT) != 0u) {
		zeroerr_schedule(st, ZE_PHASE_FAULT, 0u);
		return;
	}

	st->cmd_counts = zeroerr_rad_to_counts(desire->position);

	/*
	 * CSP (Cyclic Synchronous Position, mode 8): no New-Setpoint edge/ack
	 * protocol — unlike Profile Position, the drive tracks whatever target
	 * position is in RxPDO1 every cycle, controlword just stays at steady
	 * Operation Enabled (0x0F). Bench-confirmed reason for the switch: PP's
	 * bit4 (0x1F) requires the host to wait for statusword bit12 ack and
	 * then lower bit4 again before the next target, or set Change-Set-
	 * Immediately (bit5, never implemented here) to skip that — a
	 * continuously-streamed target (teleop) kept re-asserting bit4 before
	 * the prior one was ack'd, which the drive appears to treat as a
	 * protocol violation: intermittent Fault -> this file's fault-detect
	 * routes to a full cold-restart (audible brake click) roughly every
	 * update. CSP has no such handshake, so streaming is safe. */
	st->controlword = ZEROERR_CW_ENABLE;

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

/* No plugin_ops_t/generic pack_tx-parse_rx here (unlike RobStride/Damiao/
 * CubeMars) — actuator.c special-cases PROTO_ZEROERR and always calls
 * zeroerr_apply_cycle/zeroerr_on_rx_frame directly, `continue`-ing before
 * the generic dispatcher is ever reached. A single CAN frame can't carry
 * ZeroErr's enable-walk anyway, so a generic single-frame pack/parse pair
 * would be misleading, not just redundant. See plugin_table.c.  */

/* --------------------------------------------------------------------- */
/* Bench discover/probe — never called from zeroerr_apply_cycle.         */
/* --------------------------------------------------------------------- */

bool zeroerr_probe_node(can_bus_id_t bus, uint8_t node_id, uint32_t sdo_timeout_ms,
                        zeroerr_probe_result_t *out)
{
	uint32_t vendor = 0u, product = 0u, revision = 0u;

	if (out == NULL)
		return false;
	if (sdo_timeout_ms == 0u)
		sdo_timeout_ms = ZEROERR_SDO_TIMEOUT_MS;

	memset(out, 0, sizeof(*out));
	out->node_id = node_id;

	if (!zeroerr_read_identity(bus, node_id, &vendor, &product, &revision, sdo_timeout_ms))
		return false;

	out->found = true;
	out->discovered_id = node_id;
	out->vendor = vendor;
	out->product = product;
	out->revision = revision;
	out->vendor_match = (vendor == ZEROERR_VENDOR_ID) && (product == ZEROERR_PRODUCT_CODE);
	return true;
}

bool zeroerr_probe_node_range(can_bus_id_t bus, uint8_t start_id, uint8_t end_id,
                              uint32_t sdo_timeout_ms, zeroerr_probe_result_t *out)
{
	uint16_t lo = start_id;
	uint16_t hi = end_id;

	if (out == NULL)
		return false;
	if (hi < lo) {
		uint16_t tmp = lo;
		lo = hi;
		hi = tmp;
	}
	if (lo == 0u)
		lo = 1u; /* node 0 is not a valid CANopen node id */
	if (hi > 127u)
		hi = 127u;

	memset(out, 0, sizeof(*out));
	out->node_id = (uint8_t)lo;

	for (uint16_t node = lo; node <= hi; node++) {
		if (zeroerr_probe_node(bus, (uint8_t)node, sdo_timeout_ms, out))
			return true;
		out->node_id = (uint8_t)node;
		plant_diag_yield_usb();
	}
	return false;
}
