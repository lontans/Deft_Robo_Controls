#include "plant/plugins/cubemars.h"
#include "plant/plugin_schema/plugin.h"
#include "plant/can/can_router.h"
#include "plant/can/mcp2518fd.h"
#include "plant/can/spi_can_router.h"
#include "plant/plant_diag.h"
#include "plant/actuator.h"
#include "main.h"
#include <string.h>

/*
 * CubeMars AK MIT Power Mode — hot plant path.
 * Lifecycle / PDF sample bugs: cubemars.h + docs/rfc-cubemars-mit-plant.md
 */

uint32_t cubemars_build_ext_id(cubemars_control_mode_t mode, uint8_t node_id)
{
	return (((uint32_t)mode & 0x1FFFFFu) << 8) | (uint32_t)node_id;
}

bool cubemars_parse_ext_id(uint32_t ext_id, cubemars_control_mode_t *mode, uint8_t *node_id)
{
	if (mode == NULL || node_id == NULL)
		return false;
	*node_id = (uint8_t)(ext_id & 0xFFu);
	*mode = (cubemars_control_mode_t)((ext_id >> 8) & 0x1FFFFFu);
	return true;
}

/* --------------------------------------------------------------------- */
/* MIT Power Mode (PDF §5.3)                                              */
/* --------------------------------------------------------------------- */

typedef struct {
	float v_min, v_max;
	float t_min, t_max;
} cubemars_ak_limits_t;

/* Per-module V/T (§5.3 table). P/Kp/Kd are shared macros in cubemars.h. */
static const cubemars_ak_limits_t k_ak_limits[CUBEMARS_AK_MODEL_COUNT] = {
	[CUBEMARS_AK10_9]  = { -50.0f,  50.0f,  -65.0f,  65.0f },
	[CUBEMARS_AK60_6]  = { -50.0f,  50.0f,  -15.0f,  15.0f },
	[CUBEMARS_AK70_10] = { -50.0f,  50.0f,  -25.0f,  25.0f },
	[CUBEMARS_AK80_6]  = { -76.0f,  76.0f,  -12.0f,  12.0f },
	[CUBEMARS_AK80_9]  = { -50.0f,  50.0f,  -18.0f,  18.0f },
	[CUBEMARS_AK80_80] = {  -8.0f,   8.0f, -144.0f, 144.0f },
};

/* TODO(cfg-model): no CFG field yet — default model until set_model is wired. */
static cubemars_ak_model_t s_model[ACTUATOR_COUNT];
static bool                s_model_init;
static bool                s_cubemars_enable_latched[ACTUATOR_COUNT];

static void cubemars_ensure_model_defaults(void)
{
	if (s_model_init)
		return;
	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++)
		s_model[i] = CUBEMARS_MIT_DEFAULT_MODEL;
	s_model_init = true;
}

void cubemars_set_model(uint8_t slot, cubemars_ak_model_t model)
{
	if (slot >= ACTUATOR_COUNT || model >= CUBEMARS_AK_MODEL_COUNT)
		return;
	cubemars_ensure_model_defaults();
	s_model[slot] = model;
}

cubemars_ak_model_t cubemars_get_model(uint8_t slot)
{
	cubemars_ensure_model_defaults();
	if (slot >= ACTUATOR_COUNT)
		return CUBEMARS_MIT_DEFAULT_MODEL;
	return s_model[slot];
}

uint32_t cubemars_resolve_rx_can_id(const actuator_config_t *cfg)
{
	if (cfg == NULL)
		return 0u;
	/* master_id 0 / AUTO sentinel → Drive ID (PDF "0x00 + Drive ID"). */
	if (cfg->master_id == 0u || cfg->master_id == 0xFFFFFFFFu)
		return cfg->motor_id & CAN_STD_ID_MASK;
	return cfg->master_id & CAN_STD_ID_MASK;
}

static uint8_t cubemars_actuator_slot(const actuator_config_t *cfg)
{
	if (cfg == NULL)
		return ACTUATOR_COUNT;

	if (cfg >= &actuator_table[0] && cfg < &actuator_table[ACTUATOR_COUNT])
		return (uint8_t)(cfg - actuator_table);

	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		if (actuator_table[i].bus == cfg->bus &&
		    actuator_table[i].motor_id == cfg->motor_id &&
		    actuator_table[i].protocol == PROTO_CUBEMARS)
			return i;
	}
	return ACTUATOR_COUNT;
}

static const cubemars_ak_limits_t *cubemars_limits_for_slot(uint8_t slot)
{
	cubemars_ensure_model_defaults();
	if (slot >= ACTUATOR_COUNT)
		return &k_ak_limits[CUBEMARS_MIT_DEFAULT_MODEL];
	return &k_ak_limits[s_model[slot]];
}

/* Damiao-correct map: (1<<bits)-1 both ways — not the PDF sample asymmetry. */
static int cubemars_float_to_uint(float x, float x_min, float x_max, unsigned bits)
{
	float span = x_max - x_min;
	unsigned max_val;

	if (x > x_max)
		x = x_max;
	else if (x < x_min)
		x = x_min;
	if (bits == 0u || bits > 16u || span <= 0.0f)
		return 0;
	max_val = (1u << bits) - 1u;
	return (int)((x - x_min) * ((float)max_val / span));
}

static float cubemars_uint_to_float(unsigned raw, float x_min, float x_max, unsigned bits)
{
	unsigned max_val;
	float span = x_max - x_min;

	if (bits == 0u || bits > 16u || span <= 0.0f)
		return x_min;
	max_val = (1u << bits) - 1u;
	return x_min + ((float)raw * span / (float)max_val);
}

static void cubemars_pack_cmd(uint8_t motor_id, uint8_t opcode, can_frame_t *frame_out)
{
	frame_out->id_type = CAN_ID_STD;
	frame_out->id      = (uint32_t)motor_id & CAN_STD_ID_MASK;
	frame_out->dlc     = 8;
	memset(frame_out->data, 0xFF, 8);
	frame_out->data[7] = opcode;
}

plugin_status_t cubemars_send_enable(const actuator_config_t *cfg, can_frame_t *frame_out)
{
	if (cfg == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;
	cubemars_pack_cmd((uint8_t)(cfg->motor_id & 0x7FFu), CUBEMARS_MIT_CMD_ENABLE, frame_out);
	return PLUGIN_OK;
}

plugin_status_t cubemars_send_disable(const actuator_config_t *cfg, can_frame_t *frame_out)
{
	if (cfg == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;
	cubemars_pack_cmd((uint8_t)(cfg->motor_id & 0x7FFu), CUBEMARS_MIT_CMD_DISABLE, frame_out);
	return PLUGIN_OK;
}

plugin_status_t cubemars_send_set_zero(const actuator_config_t *cfg, can_frame_t *frame_out)
{
	if (cfg == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;
	cubemars_pack_cmd((uint8_t)(cfg->motor_id & 0x7FFu), CUBEMARS_MIT_CMD_SET_ZERO, frame_out);
	return PLUGIN_OK;
}

static plugin_status_t cubemars_pack_mit(const actuator_config_t *cfg,
                                         const actuator_desire_t *desire,
                                         can_frame_t *frame_out)
{
	const cubemars_ak_limits_t *lim;
	uint8_t slot;
	unsigned p_u, v_u, kp_u, kd_u, t_u;
	uint8_t motor_id;

	if (cfg == NULL || desire == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;
	if (!cfg->enabled || cfg->protocol != PROTO_CUBEMARS)
		return PLUGIN_ERR_UNSUPPORTED;

	slot = cubemars_actuator_slot(cfg);
	lim = cubemars_limits_for_slot(slot);

	p_u  = (unsigned)cubemars_float_to_uint(desire->position, CUBEMARS_MIT_P_MIN, CUBEMARS_MIT_P_MAX, 16);
	v_u  = (unsigned)cubemars_float_to_uint(desire->velocity, lim->v_min, lim->v_max, 12);
	kp_u = (unsigned)cubemars_float_to_uint(desire->kp, CUBEMARS_MIT_KP_MIN, CUBEMARS_MIT_KP_MAX, 12);
	kd_u = (unsigned)cubemars_float_to_uint(desire->kd, CUBEMARS_MIT_KD_MIN, CUBEMARS_MIT_KD_MAX, 12);
	t_u  = (unsigned)cubemars_float_to_uint(desire->torque, lim->t_min, lim->t_max, 12);

	motor_id = (uint8_t)(cfg->motor_id & 0x7FFu);

	frame_out->id_type = CAN_ID_STD;
	frame_out->id      = motor_id;
	frame_out->dlc     = 8;

	/* Damiao nibble interleave — PDF sample data[6] wrongly reuses kp>>8. */
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

static plugin_status_t cubemars_parse_mit(const actuator_config_t *cfg,
                                          const can_frame_t *frame_in,
                                          actuator_state_t *state_out)
{
	const cubemars_ak_limits_t *lim;
	uint8_t slot;
	uint32_t expect_id;
	unsigned p_u, v_u, t_u;

	if (cfg == NULL || frame_in == NULL || state_out == NULL)
		return PLUGIN_ERR_PARAM;
	if (!cfg->enabled || cfg->protocol != PROTO_CUBEMARS)
		return PLUGIN_ERR_UNSUPPORTED;
	if (frame_in->id_type != CAN_ID_STD)
		return PLUGIN_ERR_UNSUPPORTED;
	/* Reject garbage early — do not touch state_out. */
	if (frame_in->dlc < 6u)
		return PLUGIN_ERR_UNSUPPORTED;

	expect_id = cubemars_resolve_rx_can_id(cfg);
	if ((frame_in->id & CAN_STD_ID_MASK) != expect_id)
		return PLUGIN_ERR_UNSUPPORTED;

	/* PDF field table: D[0] = Drive ID. Reject wrong-node frames on a shared bus. */
	if ((frame_in->data[0] & 0xFFu) != (uint8_t)(cfg->motor_id & 0xFFu))
		return PLUGIN_ERR_UNSUPPORTED;

	slot = cubemars_actuator_slot(cfg);
	lim = cubemars_limits_for_slot(slot);

	/* D[1..2]=p(16), D[3..5]=v(12)|t(12), D[6]=temp, D[7]=err */
	p_u = ((unsigned)frame_in->data[1] << 8) | frame_in->data[2];
	v_u = ((unsigned)frame_in->data[3] << 4) | (frame_in->data[4] >> 4);
	t_u = (((unsigned)frame_in->data[4] & 0x0Fu) << 8) | frame_in->data[5];

	state_out->position    = cubemars_uint_to_float(p_u, CUBEMARS_MIT_P_MIN, CUBEMARS_MIT_P_MAX, 16);
	state_out->velocity    = cubemars_uint_to_float(v_u, lim->v_min, lim->v_max, 12);
	state_out->torque      = cubemars_uint_to_float(t_u, lim->t_min, lim->t_max, 12);
	/* PDF DLC text says 6; table lists temp@6 err@7 — accept short motion frames. */
	state_out->temperature = (frame_in->dlc >= 7u) ? (float)frame_in->data[6] : 0.0f;
	state_out->fault       = (frame_in->dlc >= 8u) ? (uint32_t)frame_in->data[7] : 0u;

	return PLUGIN_OK;
}

void cubemars_reset_enable_latch(uint8_t slot)
{
	if (slot < ACTUATOR_COUNT)
		s_cubemars_enable_latched[slot] = false;
}

bool cubemars_is_enable_latched(uint8_t slot)
{
	if (slot >= ACTUATOR_COUNT)
		return false;
	return s_cubemars_enable_latched[slot];
}

/*
 * RESET → ENTER (0xFC once) → STREAM (MIT every tick) → EXIT via recovery only.
 * Latch is TX-driven: set true after enqueueing enable, no RX confirmation.
 */
void cubemars_apply_cycle(const actuator_config_t *cfg,
                          const actuator_desire_t *desire,
                          actuator_state_t *state_out)
{
	can_frame_t frame;
	uint8_t slot;

	if (cfg == NULL || desire == NULL || !cfg->enabled ||
	    cfg->protocol != PROTO_CUBEMARS)
		return;

	(void)state_out;

	slot = cubemars_actuator_slot(cfg);
	if (slot >= ACTUATOR_COUNT)
		return; /* never fall back to slot 0 latch */

	if (!s_cubemars_enable_latched[slot]) {
		if (cubemars_send_enable(cfg, &frame) == PLUGIN_OK)
			(void)can_tx_enqueue(cfg->bus, &frame);
		s_cubemars_enable_latched[slot] = true;
	}

	if (cubemars_pack_mit(cfg, desire, &frame) == PLUGIN_OK)
		(void)can_tx_enqueue(cfg->bus, &frame);
}

void cubemars_on_rx_frame(const actuator_config_t *cfg, uint8_t slot,
                         const can_frame_t *frame, actuator_state_t *state_out)
{
	(void)slot;
	if (cfg == NULL || frame == NULL || state_out == NULL)
		return;
	/* Failed parse leaves prior state_out untouched. */
	(void)cubemars_parse_mit(cfg, frame, state_out);
}

#if CUBEMARS_ENABLE_SERVO_MODE
/*
 * Servo Mode Position-Speed (mode 6) — reference only; not hot path.
 */
static plugin_status_t cubemars_servo_pack_tx(const actuator_config_t *cfg,
                                              const actuator_desire_t *desire,
                                              can_frame_t *frame_out)
{
	if (cfg == NULL || desire == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;
	if (!cfg->enabled)
		return PLUGIN_ERR_UNSUPPORTED;

	uint8_t node_id = (uint8_t)(cfg->motor_id & 0xFFu);
	uint32_t ext_id = cubemars_build_ext_id(CUBEMARS_MODE_POS_SPEED, node_id);

	int32_t pos_raw   = (int32_t)(desire->position * CUBEMARS_POS_SCALE);
	int16_t speed_raw = (int16_t)(desire->velocity * CUBEMARS_SPEED_SCALE);
	int16_t accel_raw = 0;

	frame_out->id_type = CAN_ID_EXT;
	frame_out->id      = ext_id & CAN_EXT_MASK;
	frame_out->dlc     = 8;

	frame_out->data[0] = (uint8_t)((uint32_t)pos_raw >> 24);
	frame_out->data[1] = (uint8_t)((uint32_t)pos_raw >> 16);
	frame_out->data[2] = (uint8_t)((uint32_t)pos_raw >> 8);
	frame_out->data[3] = (uint8_t)((uint32_t)pos_raw);
	frame_out->data[4] = (uint8_t)((uint16_t)speed_raw >> 8);
	frame_out->data[5] = (uint8_t)((uint16_t)speed_raw);
	frame_out->data[6] = (uint8_t)((uint16_t)accel_raw >> 8);
	frame_out->data[7] = (uint8_t)((uint16_t)accel_raw);

	return PLUGIN_OK;
}

static plugin_status_t cubemars_servo_parse_rx(const actuator_config_t *cfg,
                                               const can_frame_t *frame_in,
                                               actuator_state_t *state_out)
{
	if (cfg == NULL || frame_in == NULL || state_out == NULL)
		return PLUGIN_ERR_PARAM;

	if (cfg->master_id == 0u)
		return PLUGIN_ERR_UNSUPPORTED;
	if (frame_in->id_type != CAN_ID_EXT || frame_in->id != cfg->master_id)
		return PLUGIN_ERR_UNSUPPORTED;
	if (frame_in->dlc < 8u)
		return PLUGIN_ERR_UNSUPPORTED;

	int16_t pos_raw   = (int16_t)(((uint16_t)frame_in->data[0] << 8) | frame_in->data[1]);
	int16_t speed_raw = (int16_t)(((uint16_t)frame_in->data[2] << 8) | frame_in->data[3]);
	int16_t cur_raw   = (int16_t)(((uint16_t)frame_in->data[4] << 8) | frame_in->data[5]);
	int8_t  temp_raw  = (int8_t)frame_in->data[6];
	uint8_t err_code  = frame_in->data[7];

	state_out->position    = (float)pos_raw * 0.1f;
	state_out->velocity    = (float)speed_raw * 10.0f;
	state_out->torque      = (float)cur_raw * 0.01f;
	state_out->temperature = (float)temp_raw;
	state_out->fault       = (uint32_t)err_code;

	return PLUGIN_OK;
}
#endif /* CUBEMARS_ENABLE_SERVO_MODE */

const plugin_ops_t cubemars_ops = {
	.pack_tx  = cubemars_pack_mit,
	.parse_rx = cubemars_parse_mit,
};

/* --------------------------------------------------------------------- */
/* Bench discover/probe — never called from cubemars_apply_cycle.        */
/* --------------------------------------------------------------------- */

static void cubemars_probe_copy_rx(const can_frame_t *frame, cubemars_probe_result_t *out)
{
	if (frame == NULL || out == NULL)
		return;
	out->rx_can_id = frame->id;
	memcpy(out->data, frame->data, 8u);
}

/* Decodes with the default-model limits (position uses shared P_MIN/P_MAX,
 * always correct; velocity/torque may be off if the live drive isn't
 * CUBEMARS_MIT_DEFAULT_MODEL — acceptable for discover, which only needs
 * "does something answer at this Drive ID", not calibrated motion. */
static bool cubemars_probe_try_parse(const can_frame_t *frame, uint8_t probed_id,
                                     bool promiscuous, cubemars_probe_result_t *out)
{
	const cubemars_ak_limits_t *lim = &k_ak_limits[CUBEMARS_MIT_DEFAULT_MODEL];
	unsigned p_u, v_u, t_u;

	if (frame == NULL || out == NULL)
		return false;
	if (frame->id_type != CAN_ID_STD)
		return false;
	if (frame->dlc < 6u)
		return false;
	if (!promiscuous && (frame->data[0] & 0xFFu) != probed_id)
		return false;

	out->found = true;
	out->discovered_id = frame->data[0];
	out->rx_can_id = frame->id;
	memcpy(out->data, frame->data, 8u);

	p_u = ((unsigned)frame->data[1] << 8) | frame->data[2];
	v_u = ((unsigned)frame->data[3] << 4) | (frame->data[4] >> 4);
	t_u = (((unsigned)frame->data[4] & 0x0Fu) << 8) | frame->data[5];

	out->position    = cubemars_uint_to_float(p_u, CUBEMARS_MIT_P_MIN, CUBEMARS_MIT_P_MAX, 16);
	out->velocity    = cubemars_uint_to_float(v_u, lim->v_min, lim->v_max, 12);
	out->torque      = cubemars_uint_to_float(t_u, lim->t_min, lim->t_max, 12);
	out->temperature = (frame->dlc >= 7u) ? (float)frame->data[6] : 0.0f;
	out->fault       = (frame->dlc >= 8u) ? frame->data[7] : 0u;
	return true;
}

/* Bypass the software TX queue on MCP rails — queued enqueue+flush from this
 * blocking bench context was observed to stall (same fix as damiao_probe_tx_now
 * / RobStride probes). */
static bool cubemars_probe_tx_now(can_bus_id_t bus, const can_frame_t *frame)
{
	if (frame == NULL)
		return false;

	if (bus >= CAN_BUS_CH4) {
		mcp2518_prepare_tx(bus);
		if (spi_can_router_send_now(bus, frame))
			return true;
		mcp2518_prepare_tx(bus);
		return spi_can_router_send_now(bus, frame);
	}

	if (can_tx_enqueue(bus, frame) != CAN_OK)
		return false;
	can_router_poll_bus(bus);
	return true;
}

static bool cubemars_probe_send(uint8_t motor_id, uint8_t probe_kind, can_frame_t *tx)
{
	switch (probe_kind) {
	case CUBEMARS_PROBE_ENABLE:
		cubemars_pack_cmd(motor_id, CUBEMARS_MIT_CMD_ENABLE, tx);
		return true;
	case CUBEMARS_PROBE_DISABLE:
		cubemars_pack_cmd(motor_id, CUBEMARS_MIT_CMD_DISABLE, tx);
		return true;
	case CUBEMARS_PROBE_SET_ZERO:
		cubemars_pack_cmd(motor_id, CUBEMARS_MIT_CMD_SET_ZERO, tx);
		return true;
	default: {
		actuator_config_t local = {
			.protocol = PROTO_CUBEMARS,
			.motor_id = motor_id,
			.enabled = true,
		};
		static const actuator_desire_t desire_zero = {0};

		return cubemars_pack_mit(&local, &desire_zero, tx) == PLUGIN_OK;
	}
	}
}

static bool cubemars_probe_listen(can_bus_id_t bus, const can_frame_t *tx,
                                  uint8_t motor_id, bool promiscuous,
                                  uint16_t listen_ms, cubemars_probe_result_t *out)
{
	bool mcp = (bus >= CAN_BUS_CH4);

	if (cubemars_probe_tx_now(bus, tx) && out->tx_frames_sent < 255u)
		out->tx_frames_sent++;

	for (uint16_t attempt = 0; attempt < listen_ms; attempt++) {
		can_frame_t rx;

		if (!mcp)
			can_router_poll_bus(bus);
		while (can_rx_pop(bus, &rx) == CAN_OK) {
			if (out->raw_frames_seen < 255u)
				out->raw_frames_seen++;
			cubemars_probe_copy_rx(&rx, out);
			if (cubemars_probe_try_parse(&rx, motor_id, promiscuous, out))
				return true;
		}
		if (mcp)
			can_router_poll_bus(bus);
		plant_diag_yield_usb();
		HAL_Delay(1);
	}
	return false;
}

bool cubemars_probe_id(can_bus_id_t bus, uint8_t motor_id, uint8_t probe_kind,
                       uint16_t listen_ms, cubemars_probe_result_t *out)
{
	can_frame_t tx;
	bool promiscuous = (probe_kind == CUBEMARS_PROBE_PROMISC);

	if (out == NULL)
		return false;
	if (listen_ms == 0u)
		listen_ms = 15u;
	if (bus >= CAN_BUS_CH4 && listen_ms < 40u)
		listen_ms = 40u;

	memset(out, 0, sizeof(*out));
	out->motor_id = motor_id;
	out->probe_kind = probe_kind;

	if (!cubemars_probe_send(motor_id, probe_kind, &tx))
		return false;

	return cubemars_probe_listen(bus, &tx, motor_id, promiscuous, listen_ms, out);
}

bool cubemars_probe_id_range(can_bus_id_t bus, uint8_t start_id, uint8_t end_id,
                             uint16_t listen_ms_per_id, cubemars_probe_result_t *out)
{
	uint16_t lo = start_id;
	uint16_t hi = end_id;

	if (out == NULL)
		return false;
	if (listen_ms_per_id == 0u)
		listen_ms_per_id = 3u;
	if (bus >= CAN_BUS_CH4 && listen_ms_per_id < 20u)
		listen_ms_per_id = 20u;
	if (hi < lo) {
		uint16_t tmp = lo;
		lo = hi;
		hi = tmp;
	}

	memset(out, 0, sizeof(*out));
	out->probe_kind = CUBEMARS_PROBE_ID_SWEEP;

	for (uint16_t id = lo; id <= hi; id++) {
		can_frame_t tx;

		out->motor_id = (uint8_t)id;
		if (!cubemars_probe_send((uint8_t)id, CUBEMARS_PROBE_MIT, &tx))
			continue;
		if (cubemars_probe_listen(bus, &tx, (uint8_t)id, false, listen_ms_per_id, out))
			return true;
	}
	return false;
}
