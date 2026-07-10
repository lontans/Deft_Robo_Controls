#include "plant/plugins/damiao.h"
#include "plant/plugin_schema/plugin.h"
#include "plant/can/can_router.h"
#include "plant/plant_diag.h"
#include "plant/actuator.h"
#include "main.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static int float_to_uint(float x, float x_min, float x_max, unsigned bits)
{
	float span = x_max - x_min;

	if (x > x_max)
		x = x_max;
	else if (x < x_min)
		x = x_min;

	if (bits == 0u || bits > 16u)
		return 0;

	unsigned max_val = (1u << bits) - 1u;
	return (int)((x - x_min) * ((float)max_val / span));
}

static float uint_to_float(unsigned raw, float x_min, float x_max, unsigned bits)
{
	unsigned max_val = (bits >= 16u) ? 65535u : ((1u << bits) - 1u);
	return x_min + ((float)raw * (x_max - x_min) / (float)max_val);
}

static void damiao_limits(const actuator_config_t *cfg,
                          float *p_min, float *p_max,
                          float *v_min, float *v_max,
                          float *t_min, float *t_max)
{
	(void)cfg;
	*p_min = DM4310_P_MIN;
	*p_max = DM4310_P_MAX;
	*v_min = DM4310_V_MIN;
	*v_max = DM4310_V_MAX;
	*t_min = DM4310_T_MIN;
	*t_max = DM4310_T_MAX;
}

uint32_t damiao_master_id_resolve(const actuator_config_t *cfg)
{
	if (cfg == NULL)
		return DM_MASTER_ID_DEFAULT;

	if (cfg->master_id == DM_MASTER_ID_AUTO)
		return (cfg->motor_id + 0x10u) & 0x7FFu;

	if (cfg->master_id <= 0x7FFu)
		return cfg->master_id;

	return DM_MASTER_ID_DEFAULT;
}

static bool damiao_master_id_matches(const actuator_config_t *cfg, uint32_t rx_id)
{
	uint32_t expect = damiao_master_id_resolve(cfg) & 0x7FFu;
	uint32_t got = rx_id & 0x7FFu;

	if (got == expect)
		return true;

	if (cfg->master_id == DM_MASTER_ID_AUTO) {
		if (got == DM_MASTER_ID_DEFAULT)
			return true;
		if (got == (cfg->motor_id & 0x7FFu))
			return true;
	}

	return false;
}

static uint8_t damiao_feedback_motor_id(uint8_t id_err)
{
	/* D[0] = (ERR << 4) | (ESC_ID & 0x0F). ERR 0–7 = status, 8+ = fault. */
	if (id_err <= 0x0Fu)
		return id_err;
	return id_err & 0x0Fu;
}

static void damiao_probe_copy_rx(const can_frame_t *frame, damiao_probe_result_t *out)
{
	if (frame == NULL || out == NULL)
		return;

	out->rx_can_id = frame->id;
	out->master_id = (uint8_t)(frame->id & 0xFFu);
	memcpy(out->data, frame->data, 8u);
}

static void damiao_decode_feedback_payload(const uint8_t *data,
                                           float *position,
                                           float *velocity,
                                           float *torque,
                                           float *temperature)
{
	float p_min, p_max, v_min, v_max, t_min, t_max;
	damiao_limits(NULL, &p_min, &p_max, &v_min, &v_max, &t_min, &t_max);

	unsigned p_raw = ((unsigned)data[1] << 8) | data[2];
	unsigned v_raw = ((unsigned)data[3] << 4) | (data[4] >> 4);
	unsigned t_raw = (((unsigned)data[4] & 0x0Fu) << 8) | data[5];

	if (position != NULL)
		*position = uint_to_float(p_raw, p_min, p_max, 16);
	if (velocity != NULL)
		*velocity = uint_to_float(v_raw, v_min, v_max, 12);
	if (torque != NULL)
		*torque = uint_to_float(t_raw, t_min, t_max, 12);
	if (temperature != NULL)
		*temperature = (float)data[6];
}

static void damiao_pack_cmd(uint8_t motor_id, uint8_t cmd, can_frame_t *frame_out)
{
	/* J4310 on our bench accepts the legacy layout (0xFF fill, opcode in D[7]).
	 * V1.2 manual table shows D[0]=0xFF D[1]=cmd — that layout did not enable
	 * the motor here (LED stayed red). MIT/Seeed reference uses D[7]. */
	frame_out->id_type = CAN_ID_STD;
	frame_out->id      = (uint32_t)motor_id & CAN_STD_ID_MASK;
	frame_out->dlc     = 8;
	memset(frame_out->data, 0xFF, 8);
	frame_out->data[7] = cmd;
}

plugin_status_t damiao_send_enable(const actuator_config_t *cfg, can_frame_t *frame_out)
{
	if (cfg == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;

	damiao_pack_cmd((uint8_t)(cfg->motor_id & 0x7FFu), 0xFCu, frame_out);
	return PLUGIN_OK;
}

plugin_status_t damiao_send_disable(const actuator_config_t *cfg, can_frame_t *frame_out)
{
	if (cfg == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;

	damiao_pack_cmd((uint8_t)(cfg->motor_id & 0x7FFu), 0xFDu, frame_out);
	return PLUGIN_OK;
}

plugin_status_t damiao_send_clear_fault(const actuator_config_t *cfg, can_frame_t *frame_out)
{
	if (cfg == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;

	damiao_pack_cmd((uint8_t)(cfg->motor_id & 0x7FFu), 0xFBu, frame_out);
	return PLUGIN_OK;
}

plugin_status_t damiao_send_set_zero(const actuator_config_t *cfg, can_frame_t *frame_out)
{
	if (cfg == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;

	damiao_pack_cmd((uint8_t)(cfg->motor_id & 0x7FFu), 0xFEu, frame_out);
	return PLUGIN_OK;
}

plugin_status_t damiao_send_param_read(const actuator_config_t *cfg,
                                       uint8_t register_id,
                                       can_frame_t *frame_out)
{
	if (cfg == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;

	uint16_t esc_id = (uint16_t)(cfg->motor_id & 0x7FFu);

	frame_out->id_type = CAN_ID_STD;
	frame_out->id      = DM_CAN_BROADCAST_ID;
	frame_out->dlc     = 4;
	frame_out->data[0] = (uint8_t)(esc_id & 0xFFu);
	frame_out->data[1] = (uint8_t)(esc_id >> 8);
	frame_out->data[2] = 0x33u;
	frame_out->data[3] = register_id;
	return PLUGIN_OK;
}

static plugin_status_t damiao_pack_tx(const actuator_config_t *cfg,
                                      const actuator_desire_t *desire,
                                      can_frame_t *frame_out)
{
	if (cfg == NULL || desire == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;
	if (!cfg->enabled || cfg->protocol != PROTO_DAMIAO)
		return PLUGIN_ERR_UNSUPPORTED;

	float p_min, p_max, v_min, v_max, t_min, t_max;
	damiao_limits(cfg, &p_min, &p_max, &v_min, &v_max, &t_min, &t_max);

	unsigned p_u  = (unsigned)float_to_uint(desire->position, p_min, p_max, 16);
	unsigned v_u  = (unsigned)float_to_uint(desire->velocity, v_min, v_max, 12);
	unsigned kp_u = (unsigned)float_to_uint(desire->kp, DM_KP_MIN, DM_KP_MAX, 12);
	unsigned kd_u = (unsigned)float_to_uint(desire->kd, DM_KD_MIN, DM_KD_MAX, 12);
	unsigned t_u  = (unsigned)float_to_uint(desire->torque, t_min, t_max, 12);

	uint8_t motor_id = (uint8_t)(cfg->motor_id & 0x7FFu);

	frame_out->id_type = CAN_ID_STD;
	frame_out->id      = motor_id;
	frame_out->dlc     = 8;

	frame_out->data[0] = (uint8_t)(p_u >> 8);
	frame_out->data[1] = (uint8_t)(p_u & 0xFFu);
	frame_out->data[2] = (uint8_t)(v_u >> 4);
	frame_out->data[3] = (uint8_t)(((v_u & 0x0Fu) << 4) | ((kp_u >> 8) & 0x0Fu));
	frame_out->data[4] = (uint8_t)(kp_u & 0xFFu);
	frame_out->data[5] = (uint8_t)(kd_u >> 4);
	frame_out->data[6] = (uint8_t)(((kd_u & 0x0Fu) << 4) | ((t_u >> 8) & 0x0Fu));
	frame_out->data[7] = (uint8_t)(t_u & 0xFFu);

	return PLUGIN_OK;
}

static void damiao_pack_pos_tx(uint8_t motor_id, can_frame_t *frame_out)
{
	frame_out->id_type = CAN_ID_STD;
	frame_out->id      = (0x100u + (uint32_t)motor_id) & CAN_STD_ID_MASK;
	frame_out->dlc     = 8;
	memset(frame_out->data, 0, 8);
}

static void damiao_pack_vel_tx(uint8_t motor_id, can_frame_t *frame_out)
{
	frame_out->id_type = CAN_ID_STD;
	frame_out->id      = (0x200u + (uint32_t)motor_id) & CAN_STD_ID_MASK;
	frame_out->dlc     = 4;
	memset(frame_out->data, 0, 4);
}

static plugin_status_t damiao_parse_rx(const actuator_config_t *cfg,
                                       const can_frame_t *frame_in,
                                       actuator_state_t *state_out)
{
	if (cfg == NULL || frame_in == NULL || state_out == NULL)
		return PLUGIN_ERR_PARAM;
	if (!cfg->enabled || cfg->protocol != PROTO_DAMIAO)
		return PLUGIN_ERR_UNSUPPORTED;
	if (frame_in->id_type != CAN_ID_STD)
		return PLUGIN_ERR_UNSUPPORTED;
	if (!damiao_master_id_matches(cfg, frame_in->id)) {
		/* Bench fallback: accept feedback on ESC_ID when Master ID unset/wrong. */
		if ((uint8_t)(frame_in->id & 0xFFu) != (uint8_t)(cfg->motor_id & 0xFFu))
			return PLUGIN_ERR_UNSUPPORTED;
	}

	uint8_t expect_id = (uint8_t)(cfg->motor_id & 0xFFu);
	uint8_t rx_id = damiao_feedback_motor_id(frame_in->data[0]);

	if (rx_id != expect_id && (frame_in->data[0] & 0xFFu) != expect_id)
		return PLUGIN_ERR_UNSUPPORTED;

	state_out->fault = (uint32_t)((frame_in->data[0] >> 4) & 0x0Fu);
	{
		float pos, vel, torq, temp;

		damiao_decode_feedback_payload(frame_in->data, &pos, &vel, &torq, &temp);
		state_out->position    = pos;
		state_out->velocity    = vel;
		state_out->torque      = torq;
		state_out->temperature = temp;
	}
	return PLUGIN_OK;
}

static bool damiao_id_matches(uint8_t probed_id, uint8_t feedback_id_err)
{
	uint8_t rx = damiao_feedback_motor_id(feedback_id_err);

	return (rx == probed_id) ||
	       ((feedback_id_err & 0xFFu) == probed_id);
}

static bool damiao_try_parse_feedback(const can_frame_t *frame,
                                      uint8_t probed_id,
                                      uint8_t master_id_filter,
                                      bool promiscuous,
                                      damiao_probe_result_t *out)
{
	if (frame == NULL || out == NULL)
		return false;
	if (frame->id_type != CAN_ID_STD)
		return false;
	if (frame->dlc < 6u)
		return false;

	if (master_id_filter != DM_MASTER_ID_ANY) {
		if ((uint8_t)(frame->id & 0xFFu) != master_id_filter)
			return false;
	}

	if (!promiscuous && !damiao_id_matches(probed_id, frame->data[0]))
		return false;

	out->found = true;
	out->discovered_id = damiao_feedback_motor_id(frame->data[0]);
	out->master_id = (uint8_t)(frame->id & 0xFFu);
	out->rx_can_id = frame->id;
	out->err = (frame->data[0] >> 4) & 0x0Fu;
	memcpy(out->data, frame->data, 8u);
	damiao_decode_feedback_payload(frame->data,
	                               &out->position,
	                               &out->velocity,
	                               &out->torque,
	                               &out->temperature);
	return true;
}

static bool damiao_probe_is_param_kind(uint8_t kind)
{
	return kind == DM_PROBE_READ_PARAM || kind == DM_PROBE_REG_SCAN;
}

static bool damiao_try_parse_param_read(const can_frame_t *frame,
                                        uint8_t probed_id,
                                        uint8_t register_id,
                                        bool sweep_mode,
                                        damiao_probe_result_t *out)
{
	if (frame == NULL || out == NULL)
		return false;
	if (frame->id_type != CAN_ID_STD)
		return false;
	if (frame->dlc < 4u)
		return false;
	if (frame->data[2] != 0x33u)
		return false;
	if (register_id != 0u && frame->data[3] != register_id)
		return false;

	uint8_t esc = 0u;
	uint8_t mst = 0u;
	uint32_t param_value = 0u;

	if (frame->dlc >= 8u) {
		param_value = (uint32_t)frame->data[4] |
		              ((uint32_t)frame->data[5] << 8) |
		              ((uint32_t)frame->data[6] << 16) |
		              ((uint32_t)frame->data[7] << 24);
	} else if (frame->dlc >= 5u) {
		param_value = (uint32_t)frame->data[4];
	}

	if (register_id == DM_REG_ESC_ID) {
		esc = (frame->dlc >= 5u) ? (uint8_t)(param_value & 0xFFu)
		                         : (uint8_t)(frame->data[0] & 0xFFu);
		if (frame->data[0] != esc)
			return false;
		if (!sweep_mode && esc != probed_id)
			return false;
	} else if (register_id == DM_REG_MST_ID) {
		mst = (frame->dlc >= 5u) ? (uint8_t)(param_value & 0xFFu)
		                         : (uint8_t)(frame->id & 0xFFu);
		if (!sweep_mode && frame->data[0] != (uint8_t)(probed_id & 0xFFu))
			return false;
	} else {
		return false;
	}

	out->found = true;
	out->param_rid = frame->data[3];
	out->param_value = param_value;
	out->master_id = (uint8_t)(frame->id & 0xFFu);
	out->rx_can_id = frame->id;
	out->discovered_id = probed_id;
	if (frame->dlc >= 8u)
		memcpy(out->data, frame->data, 8u);

	if (out->param_rid == DM_REG_ESC_ID) {
		out->discovered_id = esc;
		out->motor_id = esc;
	} else if (out->param_rid == DM_REG_MST_ID) {
		out->master_id = mst;
	}

	/* 0x7FF register replies are not MIT feedback — do not decode motion. */
	out->position    = 0.0f;
	out->velocity    = 0.0f;
	out->torque      = 0.0f;
	out->temperature = 0.0f;
	out->err         = 0u;

	return true;
}

static uint8_t damiao_actuator_slot(const actuator_config_t *cfg)
{
	if (cfg == NULL)
		return 0u;

	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		if (actuator_table[i].bus == cfg->bus &&
		    actuator_table[i].motor_id == cfg->motor_id &&
		    actuator_table[i].protocol == PROTO_DAMIAO)
			return i;
	}
	return ACTUATOR_COUNT;
}

static bool damiao_enable_latched[ACTUATOR_COUNT];

void damiao_reset_enable_latch(uint8_t slot)
{
	if (slot < ACTUATOR_COUNT)
		damiao_enable_latched[slot] = false;
}

void damiao_apply_cycle(const actuator_config_t *cfg,
                        const actuator_desire_t *desire,
                        actuator_state_t *state_out)
{
	can_frame_t frame;
	can_bus_id_t bus;
	uint8_t slot;

	if (cfg == NULL || desire == NULL || !cfg->enabled ||
	    cfg->protocol != PROTO_DAMIAO)
		return;

	slot = damiao_actuator_slot(cfg);
	if (slot >= ACTUATOR_COUNT)
		slot = 0u;

	bus = cfg->bus;

	/* Plant teleop: one clear-fault + enable per session; MIT stream keeps ERR=1. */
	if (!damiao_enable_latched[slot]) {
		if (damiao_send_clear_fault(cfg, &frame) == PLUGIN_OK)
			(void)can_tx_enqueue(bus, &frame);
		if (damiao_send_enable(cfg, &frame) == PLUGIN_OK)
			(void)can_tx_enqueue(bus, &frame);
	}

	for (uint8_t i = 0; i < 3u; i++) {
		if (damiao_pack_tx(cfg, desire, &frame) == PLUGIN_OK)
			(void)can_tx_enqueue(bus, &frame);
	}

	can_router_poll_bus(bus);

	{
		bool got_fb = false;

		while (can_rx_pop(bus, &frame) == CAN_OK) {
			if (state_out != NULL) {
				(void)damiao_parse_rx(cfg, &frame, state_out);
				got_fb = true;
			}
		}

		/* Re-arm enable only after fresh feedback (ERR 1 = enabled). */
		if (got_fb && state_out != NULL) {
			uint8_t err = (uint8_t)(state_out->fault & 0x0Fu);

			damiao_enable_latched[slot] = (err == 1u);
		}
	}
}

static bool damiao_probe_send(can_bus_id_t bus,
                              const actuator_config_t *cfg,
                              uint8_t motor_id,
                              uint8_t probe_kind,
                              uint8_t param_rid,
                              can_frame_t *tx)
{
	actuator_config_t local = {
		.bus = bus,
		.protocol = PROTO_DAMIAO,
		.motor_id = motor_id,
		.master_id = DM_MASTER_ID_AUTO,
		.enabled = true,
	};
	static const actuator_desire_t desire_zero = {0};

	switch (probe_kind) {
	case DM_PROBE_MIT:
	case DM_PROBE_ALL:
		return damiao_pack_tx(&local, &desire_zero, tx) == PLUGIN_OK;
	case DM_PROBE_POS:
		damiao_pack_pos_tx(motor_id, tx);
		return true;
	case DM_PROBE_VEL:
		damiao_pack_vel_tx(motor_id, tx);
		return true;
	case DM_PROBE_ENABLE:
		return damiao_send_enable(&local, tx) == PLUGIN_OK;
	case DM_PROBE_DISABLE:
		return damiao_send_disable(&local, tx) == PLUGIN_OK;
	case DM_PROBE_CLEAR_FAULT:
		return damiao_send_clear_fault(&local, tx) == PLUGIN_OK;
	case DM_PROBE_SET_ZERO:
		return damiao_send_set_zero(&local, tx) == PLUGIN_OK;
	case DM_PROBE_READ_PARAM:
	case DM_PROBE_REG_SCAN:
		if (param_rid == 0u)
			param_rid = DM_REG_ESC_ID;
		return damiao_send_param_read(&local, param_rid, tx) == PLUGIN_OK;
	default:
		(void)cfg;
		return damiao_pack_tx(&local, &desire_zero, tx) == PLUGIN_OK;
	}
}

static uint8_t damiao_probe_step_kind(uint8_t probe_kind, uint8_t step)
{
	if (probe_kind != DM_PROBE_ALL && probe_kind != DM_PROBE_DISCOVER)
		return probe_kind;

	switch (step) {
	case 0: return DM_PROBE_MIT;
	case 1: return DM_PROBE_POS;
	default: return DM_PROBE_VEL;
	}
}

static uint8_t damiao_probe_tx_burst(uint8_t probe_kind)
{
	switch (probe_kind) {
	case DM_PROBE_ENABLE:
	case DM_PROBE_DISABLE:
	case DM_PROBE_CLEAR_FAULT:
		return 1u;
	case DM_PROBE_READ_PARAM:
	case DM_PROBE_REG_SCAN:
		return 2u;
	default:
		return 1u;
	}
}

static bool damiao_probe_listen_rx_only(can_bus_id_t bus,
                                        uint8_t motor_id,
                                        uint8_t param_rid,
                                        bool param_sweep,
                                        uint16_t listen_ms,
                                        damiao_probe_result_t *out);

static bool damiao_probe_is_maintenance_kind(uint8_t kind)
{
	return kind == DM_PROBE_ENABLE || kind == DM_PROBE_DISABLE ||
	       kind == DM_PROBE_CLEAR_FAULT || kind == DM_PROBE_SET_ZERO;
}

/* Bench MIT feedback: one command frame, then listen — not a 20× retransmit flood. */
static bool damiao_probe_is_one_shot_tx_kind(uint8_t kind)
{
	return damiao_probe_is_maintenance_kind(kind) || kind == DM_PROBE_MIT;
}

static bool damiao_probe_send_mit_zero(can_bus_id_t bus,
                                       uint8_t motor_id,
                                       can_frame_t *tx_out)
{
	actuator_config_t local = {
		.bus = bus,
		.protocol = PROTO_DAMIAO,
		.motor_id = motor_id,
		.master_id = DM_MASTER_ID_AUTO,
		.enabled = true,
	};
	static const actuator_desire_t desire_zero = {0};

	if (tx_out == NULL)
		return false;

	return damiao_pack_tx(&local, &desire_zero, tx_out) == PLUGIN_OK;
}

static bool damiao_probe_listen_window(can_bus_id_t bus,
                                       const can_frame_t *tx,
                                       uint8_t tx_burst,
                                       uint8_t motor_id,
                                       uint8_t active_kind,
                                       uint8_t master_id_filter,
                                       uint8_t param_rid,
                                       bool promiscuous,
                                       bool param_sweep,
                                       uint16_t listen_ms,
                                       damiao_probe_result_t *out)
{
	can_frame_t mit_tx;
	bool mit_ready = false;

	for (uint16_t attempt = 0; attempt < listen_ms; attempt++) {
		if (damiao_probe_is_one_shot_tx_kind(active_kind)) {
			if (attempt == 0u) {
				if (can_tx_enqueue(bus, tx) == CAN_OK) {
					if (out->tx_frames_sent < 255u)
						out->tx_frames_sent++;
				}
				can_router_poll_bus(bus);
			} else if (active_kind == DM_PROBE_ENABLE &&
			           (attempt == 1u || (attempt % 4u) == 0u)) {
				if (!mit_ready &&
				    damiao_probe_send_mit_zero(bus, motor_id, &mit_tx))
					mit_ready = true;
				if (mit_ready) {
					if (can_tx_enqueue(bus, &mit_tx) == CAN_OK) {
						if (out->tx_frames_sent < 255u)
							out->tx_frames_sent++;
					}
					can_router_poll_bus(bus);
				}
			}
		} else if (attempt == 0u || (attempt % 4u) == 0u) {
			for (uint8_t b = 0; b < tx_burst; b++) {
				if (can_tx_enqueue(bus, tx) == CAN_OK) {
					if (out->tx_frames_sent < 255u)
						out->tx_frames_sent++;
				}
			}
			can_router_poll_bus(bus);
		}

		can_frame_t rx;
		while (can_rx_pop(bus, &rx) == CAN_OK) {
			if (out->raw_frames_seen < 255u)
				out->raw_frames_seen++;
			damiao_probe_copy_rx(&rx, out);

			if (damiao_probe_is_param_kind(active_kind)) {
				if (damiao_try_parse_param_read(&rx, motor_id, param_rid,
				                                param_sweep, out))
					return true;
			}
			if (damiao_try_parse_feedback(&rx, motor_id, master_id_filter,
			                              promiscuous, out)) {
				if (active_kind == DM_PROBE_ENABLE && out->err != 1u)
					continue;
				return true;
			}
		}

		plant_diag_yield_usb();
		HAL_Delay(1);
	}

	return false;
}

bool damiao_probe_id(can_bus_id_t bus,
                     uint8_t motor_id,
                     uint8_t probe_kind,
                     uint8_t master_id_filter,
                     uint8_t param_rid,
                     uint16_t listen_ms,
                     damiao_probe_result_t *out)
{
	can_frame_t tx;
	actuator_config_t cfg = {
		.bus = bus,
		.protocol = PROTO_DAMIAO,
		.motor_id = motor_id,
		.master_id = DM_MASTER_ID_AUTO,
		.enabled = true,
	};
	bool promiscuous = (probe_kind == DM_PROBE_PROMISC);
	bool discover_request = (probe_kind == DM_PROBE_DISCOVER);
	uint8_t host_probe_kind = probe_kind;

	if (out == NULL || bus >= CAN_BUS_CH4)
		return false;
	if (listen_ms == 0u)
		listen_ms = 15u;

	memset(out, 0, sizeof(*out));
	out->motor_id = motor_id;
	out->probe_kind = probe_kind;
	out->param_rid = param_rid;

	/* Host master_id 0 = AUTO/unset on bench — accept feedback on any Master ID. */
	if (master_id_filter == 0u)
		master_id_filter = DM_MASTER_ID_ANY;

	/* Param reads reply on Master ID — always listen promiscuously. */
	if (damiao_probe_is_param_kind(probe_kind))
		master_id_filter = DM_MASTER_ID_ANY;

if (probe_kind == DM_PROBE_ENABLE && listen_ms < 40u)
		listen_ms = 40u;

	if (probe_kind == DM_PROBE_REG_SCAN) {
		uint16_t reg_listen = (listen_ms < 20u) ? 20u : listen_ms;

		if (!damiao_probe_send(bus, &cfg, motor_id, DM_PROBE_READ_PARAM,
		                       DM_REG_ESC_ID, &tx))
			return false;

		out->probe_kind = DM_PROBE_REG_SCAN;
		out->param_rid = DM_REG_ESC_ID;
		out->found = false;

		if (!damiao_probe_listen_window(bus, &tx,
		                                damiao_probe_tx_burst(DM_PROBE_READ_PARAM),
		                                motor_id, DM_PROBE_READ_PARAM,
		                                DM_MASTER_ID_ANY, DM_REG_ESC_ID,
		                                false, false, reg_listen, out)) {
			uint16_t grace = reg_listen;

			if (grace < 60u)
				grace = 60u;
			if (grace > 120u)
				grace = 120u;

			out->found = false;
			if (!damiao_probe_listen_rx_only(bus, motor_id, DM_REG_ESC_ID,
			                                 false, grace, out))
				return false;
		}

		{
			damiao_probe_result_t esc_hit = *out;

			if (damiao_probe_send(bus, &cfg, motor_id, DM_PROBE_READ_PARAM,
			                      DM_REG_MST_ID, &tx)) {
				uint16_t mst_listen = reg_listen;

				if (mst_listen > 60u)
					mst_listen = 60u;

				out->found = false;
				if (damiao_probe_listen_window(bus, &tx,
				                               damiao_probe_tx_burst(DM_PROBE_READ_PARAM),
				                               motor_id, DM_PROBE_READ_PARAM,
				                               DM_MASTER_ID_ANY, DM_REG_MST_ID,
				                               false, false, mst_listen, out)) {
					if (out->param_rid == DM_REG_MST_ID &&
					    (out->param_value & 0xFFu) != 0u)
						esc_hit.master_id = (uint8_t)(out->param_value & 0xFFu);
					else if (out->master_id != 0u)
						esc_hit.master_id = out->master_id;
				}
			}

			*out = esc_hit;
			out->probe_kind = DM_PROBE_REG_SCAN;
			out->param_rid = DM_REG_ESC_ID;
		}

		return true;
	}

	if (probe_kind == DM_PROBE_DISCOVER) {
		/* DISCOVER = MIT feedback probe only. Do not auto-enable: enabled motors
		 * require sustained commands (TIMEOUT register) or they fault (ERR=D). */
		probe_kind = DM_PROBE_MIT;
	}

	uint8_t step_count = (probe_kind == DM_PROBE_ALL) ? 3u : 1u;

	for (uint8_t step = 0; step < step_count; step++) {
		uint8_t kind = damiao_probe_step_kind(probe_kind, step);

		if (!damiao_probe_send(bus, &cfg, motor_id, kind, param_rid, &tx))
			continue;

		out->probe_kind = kind;

		if (damiao_probe_listen_window(bus, &tx, damiao_probe_tx_burst(kind),
		                               motor_id, kind, master_id_filter,
		                               param_rid, promiscuous, false,
		                               listen_ms, out)) {
			if (discover_request)
				out->probe_kind = host_probe_kind;
			return true;
		}
	}

	if (discover_request)
		out->probe_kind = host_probe_kind;

	return out->found;
}

static bool damiao_probe_listen_rx_only(can_bus_id_t bus,
                                        uint8_t motor_id,
                                        uint8_t param_rid,
                                        bool param_sweep,
                                        uint16_t listen_ms,
                                        damiao_probe_result_t *out)
{
	for (uint16_t attempt = 0; attempt < listen_ms; attempt++) {
		can_frame_t rx;

		while (can_rx_pop(bus, &rx) == CAN_OK) {
			if (out->raw_frames_seen < 255u)
				out->raw_frames_seen++;
			damiao_probe_copy_rx(&rx, out);

			if (damiao_try_parse_param_read(&rx, motor_id, param_rid,
			                                param_sweep, out))
				return true;
		}

		plant_diag_yield_usb();
		HAL_Delay(1);
	}

	return false;
}

bool damiao_probe_id_range(can_bus_id_t bus,
                           uint8_t start_id,
                           uint8_t end_id,
                           uint8_t param_rid,
                           uint16_t listen_ms_per_id,
                           damiao_probe_result_t *out)
{
	actuator_config_t cfg = {
		.bus = bus,
		.protocol = PROTO_DAMIAO,
		.master_id = DM_MASTER_ID_AUTO,
		.enabled = true,
	};
	can_frame_t tx;
	uint16_t lo = start_id;
	uint16_t hi = end_id;

	if (out == NULL || bus >= CAN_BUS_CH4)
		return false;
	if (listen_ms_per_id == 0u)
		listen_ms_per_id = 3u;
	if (param_rid == 0u)
		param_rid = DM_REG_ESC_ID;
	if (hi < lo) {
		uint16_t tmp = lo;
		lo = hi;
		hi = tmp;
	}

	memset(out, 0, sizeof(*out));
	out->probe_kind = DM_PROBE_ID_SWEEP;
	out->param_rid = param_rid;

	/* Register-read request is always frame ID 0x7FF; only the ESC_ID byte in
	 * the payload changes per candidate. Any motor whose configured ESC_ID
	 * matches replies -- on whatever Master ID it's configured with, so no
	 * master ID filter is applied here (damiao_try_parse_param_read doesn't
	 * check frame->id at all, only the echoed ESC_ID in the payload). */
	for (uint16_t id = lo; id <= hi; id++) {
		cfg.motor_id = (uint8_t)id;

		if (damiao_send_param_read(&cfg, param_rid, &tx) != PLUGIN_OK)
			continue;

		out->motor_id = (uint8_t)id;

		if (damiao_probe_listen_window(bus, &tx,
		                               2u,
		                               (uint8_t)id, DM_PROBE_READ_PARAM,
		                               DM_MASTER_ID_ANY, param_rid,
		                               false, true, listen_ms_per_id, out))
			return true;

		{
			uint16_t grace = listen_ms_per_id;

			if (grace < 40u)
				grace = 40u;
			if (grace > 80u)
				grace = 80u;

			if (damiao_probe_listen_rx_only(bus, (uint8_t)id, param_rid,
			                                true, grace, out))
				return true;
		}
	}

	return false;
}

const plugin_ops_t damiao_ops = {
	.pack_tx  = damiao_pack_tx,
	.parse_rx = damiao_parse_rx,
};
