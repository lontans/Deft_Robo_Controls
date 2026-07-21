#include "plant/plugins/cubemars.h"
#include "plant/plugin_schema/plugin.h"

/*
 * CubeMars Servo Mode, Position-Speed Loop Mode (control mode 6) only.
 * See cubemars.h for the scope decision and the unverified-units caveat.
 *
 * No enable/handshake is documented for Servo Mode (unlike RobStride's
 * maintain_enable or Damiao's clear-fault+enable latch), so this plugin is
 * stateless per-frame and needs no actuator.c special-casing — it goes
 * through the generic plugin_pack_tx()/plugin_parse_rx() single-frame path.
 * If bench testing later shows a keep-alive frame is actually required,
 * promote to a hand-written cubemars_apply_cycle() (mirroring
 * robstride_apply_cycle) as a fast-follow — do not build that speculatively.
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

static plugin_status_t cubemars_pack_tx(const actuator_config_t *cfg,
                                        const actuator_desire_t *desire,
                                        can_frame_t *frame_out)
{
	if (cfg == NULL || desire == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;
	if (!cfg->enabled)
		return PLUGIN_ERR_UNSUPPORTED;

	uint8_t node_id = (uint8_t)(cfg->motor_id & 0xFFu);
	uint32_t ext_id = cubemars_build_ext_id(CUBEMARS_MODE_POS_SPEED, node_id);

	/* TODO(hardware): position/speed passed through as placeholder units —
	 * see the P0 caveat in cubemars.h. accel has no ActuatorDesire field,
	 * defaults to 0 (TODO: confirm what accel=0 means for this firmware —
	 * doc-derivable, not hardware-gated, just not yet double-checked). */
	int32_t pos_raw   = (int32_t)(desire->position * CUBEMARS_POS_SCALE);
	int16_t speed_raw = (int16_t)(desire->velocity * CUBEMARS_SPEED_SCALE);
	int16_t accel_raw = 0;

	frame_out->id_type = CAN_ID_EXT;
	frame_out->id      = ext_id & CAN_EXT_MASK;
	frame_out->dlc     = 8;

	/* Big-endian (MSB first) — matches the vendor doc's buffer_append_int32/
	 * int16 helpers and RobStride's own big-endian wire convention. */
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

static plugin_status_t cubemars_parse_rx(const actuator_config_t *cfg,
                                         const can_frame_t *frame_in,
                                         actuator_state_t *state_out)
{
	if (cfg == NULL || frame_in == NULL || state_out == NULL)
		return PLUGIN_ERR_PARAM;

	/*
	 * TODO(hardware, P0): the periodic upload frame's CAN ID is NOT
	 * documented in the vendor PDF and is unknown until a bench CAN sniff
	 * (motor in servo mode, no host TX, observe the periodic broadcast).
	 * cfg->master_id (actuator.h: "feedback CAN ID; protocol-specific") is
	 * repurposed as that ID for CubeMars. Default 0 means "never matches" —
	 * a safe no-op until the real ID is bench-discovered and set at runtime
	 * via the CFG SET path (no firmware redeploy needed once known).
	 * This exact-match-or-ignore behavior also satisfies the mixed-frame-
	 * type bus requirement (CH1/CH3 accept both std and ext IDs now, with
	 * no bus-wide type-routing layer) — this plugin can never misparse a
	 * Damiao (standard-ID) frame sharing the same bus.
	 */
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

	/* Doc: position *0.1 -> deg, speed *10 -> electrical RPM, current *0.01 -> A.
	 * Same placeholder-units caveat as cubemars_pack_tx applies here — these
	 * are stored as-is (deg into "position", eRPM into "velocity") pending
	 * the same hardware verification, not silently relabeled as SI. */
	state_out->position    = (float)pos_raw * 0.1f;
	state_out->velocity    = (float)speed_raw * 10.0f;
	/* No true torque (Nm) is reported by Servo Mode — this carries raw
	 * current (A) instead, same "protocol-specific" convention already used
	 * for the fault field elsewhere in this codebase. */
	state_out->torque      = (float)cur_raw * 0.01f;
	state_out->temperature = (float)temp_raw;
	state_out->fault       = (uint32_t)err_code;

	return PLUGIN_OK;
}

const plugin_ops_t cubemars_ops = {
	.pack_tx  = cubemars_pack_tx,
	.parse_rx = cubemars_parse_rx,
};
