#include "plant/plugins/cubemars.h"
#include "plant/plugin_schema/plugin.h"
#include "plant/can/can_router.h"
#include "plant/actuator.h"
#include <string.h>

/*
 * CubeMars AK-series driver board — MIT Power Mode is the hot plant path.
 * See cubemars.h for the full scope note and docs/rfc-cubemars-mit-plant.md
 * for the derivation, including the vendor PDF's own sample-code bugs this
 * implementation deliberately does NOT reproduce.
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

/* Per-module velocity/torque span (PDF §5.3 p.44 table) — NOT the PDF's own
 * pack_cmd() sample constants, which contradict this table (see cubemars.h
 * and docs/rfc-cubemars-mit-plant.md). Position/Kp/Kd are shared across all
 * six modules (CUBEMARS_MIT_P_MIN/MAX etc.). */
static const cubemars_ak_limits_t k_ak_limits[CUBEMARS_AK_MODEL_COUNT] = {
	[CUBEMARS_AK10_9]  = { -50.0f,  50.0f,  -65.0f,  65.0f },
	[CUBEMARS_AK60_6]  = { -50.0f,  50.0f,  -15.0f,  15.0f },
	[CUBEMARS_AK70_10] = { -50.0f,  50.0f,  -25.0f,  25.0f },
	[CUBEMARS_AK80_6]  = { -76.0f,  76.0f,  -12.0f,  12.0f },
	[CUBEMARS_AK80_9]  = { -50.0f,  50.0f,  -18.0f,  18.0f },
	[CUBEMARS_AK80_80] = {  -8.0f,   8.0f, -144.0f, 144.0f },
};

/* No CFG plumbing selects this per slot yet (see cubemars.h) — every slot
 * defaults to CUBEMARS_MIT_DEFAULT_MODEL until cubemars_set_model() grows a
 * caller. */
static cubemars_ak_model_t s_model[ACTUATOR_COUNT];
static bool                s_model_init;

void cubemars_set_model(uint8_t slot, cubemars_ak_model_t model)
{
	if (slot >= ACTUATOR_COUNT || model >= CUBEMARS_AK_MODEL_COUNT)
		return;
	s_model[slot] = model;
}

static void cubemars_ensure_model_defaults(void)
{
	if (s_model_init)
		return;
	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++)
		s_model[i] = CUBEMARS_MIT_DEFAULT_MODEL;
	s_model_init = true;
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

/* Same linear-map shape as damiao.c's float_to_uint/uint_to_float (kept
 * local/static rather than shared — see the RFC's note on a possible future
 * common mit_codec.c, not built speculatively here). Uses (1<<bits)-1 both
 * ways, unlike the PDF's own sample which is asymmetric
 * ((1<<bits) vs (1<<bits)-1) — see docs/rfc-cubemars-mit-plant.md. */
static int cubemars_float_to_uint(float x, float x_min, float x_max, unsigned bits)
{
	float span = x_max - x_min;
	unsigned max_val;

	if (x > x_max)
		x = x_max;
	else if (x < x_min)
		x = x_min;
	if (bits == 0u || bits > 16u)
		return 0;
	max_val = (1u << bits) - 1u;
	return (int)((x - x_min) * ((float)max_val / span));
}

static float cubemars_uint_to_float(unsigned raw, float x_min, float x_max, unsigned bits)
{
	unsigned max_val;

	if (bits == 0u || bits > 16u)
		return x_min;
	max_val = (1u << bits) - 1u;
	return x_min + ((float)raw * (x_max - x_min) / (float)max_val);
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

	/* PDF §5.3 command layout — byte-for-byte the same nibble interleave as
	 * damiao_pack_tx(). The PDF's own "Sends routine code" sample has a real
	 * bug here (data[6] reuses kp_int>>8 instead of t_int>>8) — do not port
	 * it; see docs/rfc-cubemars-mit-plant.md. */
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
	unsigned p_u, v_u, t_u;

	if (cfg == NULL || frame_in == NULL || state_out == NULL)
		return PLUGIN_ERR_PARAM;
	if (frame_in->id_type != CAN_ID_STD)
		return PLUGIN_ERR_UNSUPPORTED;
	/* PDF: feedback Identifier = "0x00 + Drive ID" — the same numeric ID
	 * space as the command's own target, unlike Damiao's separate ESC-ID /
	 * Master-ID split. No master_id indirection needed here. */
	if ((frame_in->id & CAN_STD_ID_MASK) != (cfg->motor_id & CAN_STD_ID_MASK))
		return PLUGIN_ERR_UNSUPPORTED;
	if (frame_in->dlc < 6u)
		return PLUGIN_ERR_UNSUPPORTED;

	slot = cubemars_actuator_slot(cfg);
	lim = cubemars_limits_for_slot(slot);

	p_u = ((unsigned)frame_in->data[1] << 8) | frame_in->data[2];
	v_u = ((unsigned)frame_in->data[3] << 4) | (frame_in->data[4] >> 4);
	t_u = (((unsigned)frame_in->data[4] & 0x0Fu) << 8) | frame_in->data[5];

	state_out->position    = cubemars_uint_to_float(p_u, CUBEMARS_MIT_P_MIN, CUBEMARS_MIT_P_MAX, 16);
	state_out->velocity    = cubemars_uint_to_float(v_u, lim->v_min, lim->v_max, 12);
	state_out->torque      = cubemars_uint_to_float(t_u, lim->t_min, lim->t_max, 12);
	/* PDF text says "DLC: 6 bytes" but the field table lists 8 (temp @6,
	 * error @7) — a documented contradiction (see the RFC). Accept the
	 * shorter 6-byte form for motion-only frames; temp/fault default to 0
	 * when the frame is short rather than reading past dlc. */
	state_out->temperature = (frame_in->dlc >= 7u) ? (float)frame_in->data[6] : 0.0f;
	state_out->fault       = (frame_in->dlc >= 8u) ? (uint32_t)frame_in->data[7] : 0u;

	return PLUGIN_OK;
}

static bool s_cubemars_enable_latched[ACTUATOR_COUNT];

void cubemars_reset_enable_latch(uint8_t slot)
{
	if (slot < ACTUATOR_COUNT)
		s_cubemars_enable_latched[slot] = false;
}

/*
 * Deliberately Damiao-shaped, not ZeroErr-shaped: this plugin never sends
 * the exit-mode (0xFD) frame from the routine apply path, only from
 * plant_recovery_all() (mirrors damiao_apply_cycle, which never calls
 * damiao_send_disable() either — only plant_recovery_all does). A "blank"
 * desire (kp=kd=0, vel=0, torque=0) is itself a legitimate MIT zero-effort
 * command once entered, so idle is just streamed as-is, continuously, the
 * same as Damiao streams its idle MIT frame every tick rather than
 * toggling in and out of control mode. This keeps the joint ready to react
 * the instant a real desire arrives, at the cost of the drive staying in
 * MIT mode (not fully de-energized) between commands — the right trade for
 * an arm-class actuator (this RFC's whole premise, §1), unlike ZeroErr
 * (lift/gripper-shaped) which shuts down on idle instead.
 *
 * The enable latch itself is TX-driven (fires once per session, first
 * enable frame before the first MIT frame), not RX-gated like Damiao's
 * ERR==1 confirmation — CubeMars's PDF documents no fault/status semantics
 * for MIT mode to gate on, and inventing one would be exactly the kind of
 * guess docs/rfc-cubemars-mit-plant.md §6 says not to make.
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
		slot = 0u;

	/* "motor control mode must be entered before using CAN communication
	 * control motor" (PDF §5.3) — one enter-mode frame per session. */
	if (!s_cubemars_enable_latched[slot]) {
		if (cubemars_send_enable(cfg, &frame) == PLUGIN_OK)
			(void)can_tx_enqueue(cfg->bus, &frame);
		s_cubemars_enable_latched[slot] = true;
	}

	/* One MIT frame per slot per tick, every tick (idle or not) — Damiao
	 * already learned the hard way that redundant per-tick bursts
	 * oversubscribe the bus once more than a couple of slots are active
	 * (see damiao_apply_cycle's own comment); this sends exactly one. */
	if (cubemars_pack_mit(cfg, desire, &frame) == PLUGIN_OK)
		(void)can_tx_enqueue(cfg->bus, &frame);
}

void cubemars_on_rx_frame(const actuator_config_t *cfg, uint8_t slot,
                         const can_frame_t *frame, actuator_state_t *state_out)
{
	(void)slot;
	if (cfg == NULL || frame == NULL || state_out == NULL)
		return;
	(void)cubemars_parse_mit(cfg, frame, state_out);
}

#if CUBEMARS_ENABLE_SERVO_MODE
/*
 * Servo Mode, Position-Speed Loop Mode (control mode 6) — reference only,
 * not on the hot apply/RX path (see cubemars.h). No enable/handshake is
 * documented for Servo Mode, so this stays a stateless single-frame plugin
 * reachable only through the generic plugin_pack_tx()/plugin_parse_rx()
 * path if some future diagnostic explicitly selects it.
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
