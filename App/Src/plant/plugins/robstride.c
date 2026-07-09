#include "plant/plugin_schema/plugin.h"
#include "plant/plugins/robstride.h"
#include "plant/plugin_schema/plugin_table.h"
#include "plant/can/can_router.h"
#include "plant/can/spi_can_router.h"
#include "plant/can/mcp2518fd.h"
#include "plant/plant_diag.h"
#include "plant/actuator.h"
#include "main.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static int float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	if (x > x_max) x = x_max;
	else if (x < x_min) x = x_min;
	return (int)((x - offset) * ((float)(((1u << (unsigned)bits) - 1u)) / span));
}

static float uint16_to_float(uint16_t x, float x_min, float x_max)
{
	return x_min + (x / 65535.0f) * (x_max - x_min);
}

static uint32_t robstride_build_ext_id(uint8_t mode, uint16_t data16, uint8_t id_byte)
{
	return ((uint32_t)mode << 24) | ((uint32_t)data16 << 8) | ((uint32_t)id_byte);
}

static void robstride_unpack_ext_id(uint32_t ext_id,
                                    uint8_t *mode,
                                    uint16_t *data16,
                                    uint8_t *id_byte)
{
	*id_byte = (uint8_t)(ext_id & 0xFF);
	*data16 = (uint16_t)((ext_id >> 8) & 0xFFFF);
	*mode = (uint8_t)((ext_id >> 24) & 0x1F);
}

static plugin_status_t robstride_pack_tx(const actuator_config_t *cfg,
                                         const actuator_desire_t *desire,
                                         can_frame_t *frame_out)
{
	if (cfg == NULL || desire == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;
	if (!cfg->enabled)
		return PLUGIN_ERR_UNSUPPORTED;

	uint8_t motor_id = (uint8_t)(cfg->motor_id & 0xFF);

	uint16_t t_u16 = (uint16_t)float_to_uint(desire->torque, RS02_T_MIN, RS02_T_MAX, 16);
	uint32_t ext_id = robstride_build_ext_id(RS02_COMM_MOTOR_CTRL, t_u16, motor_id);

	frame_out->id_type = CAN_ID_EXT;
	frame_out->id      = ext_id & CAN_EXT_MASK;
	frame_out->dlc     = 8;

	uint16_t p = (uint16_t)float_to_uint(desire->position, RS02_P_MIN, RS02_P_MAX, 16);
	uint16_t v = (uint16_t)float_to_uint(desire->velocity, RS02_V_MIN, RS02_V_MAX, 16);
	uint16_t kp = (uint16_t)float_to_uint(desire->kp, RS02_KP_MIN, RS02_KP_MAX, 16);
	uint16_t kd = (uint16_t)float_to_uint(desire->kd, RS02_KD_MIN, RS02_KD_MAX, 16);

	frame_out->data[0] = (uint8_t)(p >> 8);
	frame_out->data[1] = (uint8_t)(p & 0xFF);
	frame_out->data[2] = (uint8_t)(v >> 8);
	frame_out->data[3] = (uint8_t)(v & 0xFF);
	frame_out->data[4] = (uint8_t)(kp >> 8);
	frame_out->data[5] = (uint8_t)(kp & 0xFF);
	frame_out->data[6] = (uint8_t)(kd >> 8);
	frame_out->data[7] = (uint8_t)(kd & 0xFF);

	return PLUGIN_OK;
}

static plugin_status_t robstride_parse_rx(const actuator_config_t *cfg,
                                          const can_frame_t *frame_in,
                                          actuator_state_t *state_out)
{
	if (cfg == NULL || frame_in == NULL || state_out == NULL)
		return PLUGIN_ERR_PARAM;
	if (!cfg->enabled || frame_in->id_type != CAN_ID_EXT)
		return PLUGIN_ERR_UNSUPPORTED;

	uint8_t mode, id_byte;
	uint16_t data16;

	robstride_unpack_ext_id(frame_in->id, &mode, &data16, &id_byte);
	if (mode != RS02_COMM_FEEDBACK)
		return PLUGIN_ERR_UNSUPPORTED;

	uint8_t motor_id = (uint8_t)(cfg->motor_id & 0xFF);

	if (((data16 & 0xFF) != motor_id) && (id_byte != motor_id))
		return PLUGIN_ERR_UNSUPPORTED;

	uint16_t p_raw = ((uint16_t)frame_in->data[0] << 8) | frame_in->data[1];
	uint16_t v_raw = ((uint16_t)frame_in->data[2] << 8) | frame_in->data[3];
	uint16_t t_raw = ((uint16_t)frame_in->data[4] << 8) | frame_in->data[5];
	uint16_t temp_raw = ((uint16_t)frame_in->data[6] << 8) | frame_in->data[7];

	state_out->position = uint16_to_float(p_raw, RS02_P_MIN, RS02_P_MAX);
	state_out->velocity = uint16_to_float(v_raw, RS02_V_MIN, RS02_V_MAX);
	state_out->torque   = uint16_to_float(t_raw, RS02_T_MIN, RS02_T_MAX);
	state_out->temperature = (float)temp_raw / 10.0f;

	state_out->fault = 0;
	state_out->fault |= ((data16) >> 13 & 1u) << 6;
	state_out->fault |= ((data16) >> 12 & 1u) << 5;
	state_out->fault |= ((data16) >> 11 & 1u) << 4;
	state_out->fault |= ((data16) >> 10 & 1u) << 3;
	state_out->fault |= ((data16) >> 9 & 1u) << 2;
	state_out->fault |= ((data16) >> 8 & 1u) << 1;

	return PLUGIN_OK;
}

plugin_status_t robstride_set_run_mode(const actuator_config_t *cfg,
                                       uint8_t run_mode,
                                       can_frame_t *frame_out)
{
	if (cfg == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;
	if (!cfg->enabled)
		return PLUGIN_ERR_UNSUPPORTED;

	uint8_t motor_id = (uint8_t)(cfg->motor_id & 0xFF);
	uint32_t ext_id = robstride_build_ext_id(RS02_COMM_PARAWRITE, RS02_HOST_ID, motor_id);

	frame_out->id_type = CAN_ID_EXT;
	frame_out->id = ext_id & CAN_EXT_MASK;
	frame_out->dlc = 8;

	frame_out->data[0] = (uint8_t)(RS02_PARAM_RUN_MODE & 0xFFu);
	frame_out->data[1] = (uint8_t)(RS02_PARAM_RUN_MODE >> 8);
	frame_out->data[2] = 0;
	frame_out->data[3] = 0;
	frame_out->data[4] = run_mode;
	frame_out->data[5] = 0;
	frame_out->data[6] = 0;
	frame_out->data[7] = 0;

	return PLUGIN_OK;
}

plugin_status_t robstride_send_enable(const actuator_config_t *cfg, can_frame_t *frame_out)
{
	if (cfg == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;
	if (!cfg->enabled)
		return PLUGIN_ERR_UNSUPPORTED;

	uint8_t motor_id = (uint8_t)(cfg->motor_id & 0xFF);

	uint32_t ext_id = robstride_build_ext_id(RS02_COMM_MOTOR_IN, RS02_HOST_ID, motor_id);

	frame_out->id_type = CAN_ID_EXT;
	frame_out->id = ext_id & CAN_EXT_MASK;
	frame_out->dlc = 8;

	for (uint8_t i = 0; i < 8; i++)
		frame_out->data[i] = 0;

	return PLUGIN_OK;
}

plugin_status_t robstride_send_reset(const actuator_config_t *cfg, can_frame_t *frame_out)
{
	if (cfg == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;
	if (!cfg->enabled)
		return PLUGIN_ERR_UNSUPPORTED;

	uint8_t motor_id = (uint8_t)(cfg->motor_id & 0xFF);
	uint32_t ext_id = robstride_build_ext_id(RS02_COMM_MOTOR_RESET, RS02_HOST_ID, motor_id);

	frame_out->id_type = CAN_ID_EXT;
	frame_out->id = ext_id & CAN_EXT_MASK;
	frame_out->dlc = 8;

	for (uint8_t i = 0; i < 8; i++)
		frame_out->data[i] = 0;
	frame_out->data[0] = 1u;

	return PLUGIN_OK;
}

plugin_status_t robstride_send_disable(const actuator_config_t *cfg, can_frame_t *frame_out)
{
	if (cfg == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;
	if (!cfg->enabled)
		return PLUGIN_ERR_UNSUPPORTED;

	uint8_t motor_id = (uint8_t)(cfg->motor_id & 0xFF);
	uint32_t ext_id = robstride_build_ext_id(RS02_COMM_MOTOR_RESET, RS02_HOST_ID, motor_id);

	frame_out->id_type = CAN_ID_EXT;
	frame_out->id = ext_id & CAN_EXT_MASK;
	frame_out->dlc = 8;

	/* Official RobStride driver: comm 0x04 all-zero payload = de-energize (失能). */
	for (uint8_t i = 0; i < 8; i++)
		frame_out->data[i] = 0;

	return PLUGIN_OK;
}

plugin_status_t robstride_send_cali(const actuator_config_t *cfg, can_frame_t *frame_out)
{
	if (cfg == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;
	if (!cfg->enabled)
		return PLUGIN_ERR_UNSUPPORTED;

	uint8_t motor_id = (uint8_t)(cfg->motor_id & 0xFF);
	uint32_t ext_id = robstride_build_ext_id(RS02_COMM_MOTOR_CALI, RS02_HOST_ID, motor_id);

	frame_out->id_type = CAN_ID_EXT;
	frame_out->id = ext_id & CAN_EXT_MASK;
	frame_out->dlc = 8;

	for (uint8_t i = 0; i < 8u; i++)
		frame_out->data[i] = 0;

	return PLUGIN_OK;
}

plugin_status_t robstride_send_zero(const actuator_config_t *cfg, can_frame_t *frame_out)
{
	if (cfg == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;
	if (!cfg->enabled)
		return PLUGIN_ERR_UNSUPPORTED;

	uint8_t motor_id = (uint8_t)(cfg->motor_id & 0xFF);
	uint32_t ext_id = robstride_build_ext_id(RS02_COMM_MOTOR_ZERO, RS02_HOST_ID, motor_id);

	frame_out->id_type = CAN_ID_EXT;
	frame_out->id = ext_id & CAN_EXT_MASK;
	frame_out->dlc = 8;

	for (uint8_t i = 0; i < 8u; i++)
		frame_out->data[i] = 0;
	frame_out->data[0] = 1u;

	return PLUGIN_OK;
}

plugin_status_t robstride_send_data_save(const actuator_config_t *cfg, can_frame_t *frame_out)
{
	if (cfg == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;
	if (!cfg->enabled)
		return PLUGIN_ERR_UNSUPPORTED;

	uint8_t motor_id = (uint8_t)(cfg->motor_id & 0xFF);
	uint32_t ext_id = robstride_build_ext_id(RS02_COMM_DATA_SAVE, RS02_HOST_ID, motor_id);

	frame_out->id_type = CAN_ID_EXT;
	frame_out->id = ext_id & CAN_EXT_MASK;
	frame_out->dlc = 8;

	frame_out->data[0] = 0x01;
	frame_out->data[1] = 0x02;
	frame_out->data[2] = 0x03;
	frame_out->data[3] = 0x04;
	frame_out->data[4] = 0x05;
	frame_out->data[5] = 0x06;
	frame_out->data[6] = 0x07;
	frame_out->data[7] = 0x08;

	return PLUGIN_OK;
}

plugin_status_t robstride_send_para_read(const actuator_config_t *cfg,
                                         uint16_t param_index,
                                         can_frame_t *frame_out)
{
	if (cfg == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;
	if (!cfg->enabled)
		return PLUGIN_ERR_UNSUPPORTED;

	uint8_t motor_id = (uint8_t)(cfg->motor_id & 0xFF);
	uint32_t ext_id = robstride_build_ext_id(RS02_COMM_PARAREAD, RS02_HOST_ID, motor_id);

	frame_out->id_type = CAN_ID_EXT;
	frame_out->id = ext_id & CAN_EXT_MASK;
	frame_out->dlc = 8;

	frame_out->data[0] = (uint8_t)(param_index & 0xFFu);
	frame_out->data[1] = (uint8_t)(param_index >> 8);
	frame_out->data[2] = 0;
	frame_out->data[3] = 0;
	frame_out->data[4] = 0;
	frame_out->data[5] = 0;
	frame_out->data[6] = 0;
	frame_out->data[7] = 0;

	return PLUGIN_OK;
}

plugin_status_t robstride_send_para_write(const actuator_config_t *cfg,
                                          uint16_t param_index,
                                          uint32_t raw_value,
                                          can_frame_t *frame_out)
{
	if (cfg == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;
	if (!cfg->enabled)
		return PLUGIN_ERR_UNSUPPORTED;

	uint8_t motor_id = (uint8_t)(cfg->motor_id & 0xFF);
	uint32_t ext_id = robstride_build_ext_id(RS02_COMM_PARAWRITE, RS02_HOST_ID, motor_id);

	frame_out->id_type = CAN_ID_EXT;
	frame_out->id = ext_id & CAN_EXT_MASK;
	frame_out->dlc = 8;

	frame_out->data[0] = (uint8_t)(param_index & 0xFFu);
	frame_out->data[1] = (uint8_t)(param_index >> 8);
	frame_out->data[2] = 0;
	frame_out->data[3] = 0;
	frame_out->data[4] = (uint8_t)(raw_value & 0xFFu);
	frame_out->data[5] = (uint8_t)((raw_value >> 8) & 0xFFu);
	frame_out->data[6] = (uint8_t)((raw_value >> 16) & 0xFFu);
	frame_out->data[7] = (uint8_t)((raw_value >> 24) & 0xFFu);

	return PLUGIN_OK;
}

plugin_status_t robstride_send_proactive(const actuator_config_t *cfg,
                                         uint8_t enable,
                                         can_frame_t *frame_out)
{
	if (cfg == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;
	if (!cfg->enabled)
		return PLUGIN_ERR_UNSUPPORTED;

	uint8_t motor_id = (uint8_t)(cfg->motor_id & 0xFF);
	uint32_t ext_id = robstride_build_ext_id(RS02_COMM_PROACTIVE, RS02_HOST_ID, motor_id);

	frame_out->id_type = CAN_ID_EXT;
	frame_out->id = ext_id & CAN_EXT_MASK;
	frame_out->dlc = 8;

	frame_out->data[0] = 0x01;
	frame_out->data[1] = 0x02;
	frame_out->data[2] = 0x03;
	frame_out->data[3] = 0x04;
	frame_out->data[4] = 0x05;
	frame_out->data[5] = 0x06;
	frame_out->data[6] = enable ? 1u : 0u;
	frame_out->data[7] = 0x08;

	return PLUGIN_OK;
}

static float robstride_bytes_to_float(const uint8_t *bytes)
{
	uint32_t raw;
	float value;

	if (bytes == NULL)
		return 0.0f;

	raw = (uint32_t)bytes[4] |
	      ((uint32_t)bytes[5] << 8) |
	      ((uint32_t)bytes[6] << 16) |
	      ((uint32_t)bytes[7] << 24);
	memcpy(&value, &raw, sizeof(value));
	return value;
}

static void robstride_probe_decode_payload(uint8_t comm_mode,
                                           const uint8_t *data,
                                           robstride_probe_result_t *out)
{
	if (data == NULL || out == NULL)
		return;

	if (comm_mode == RS02_COMM_FEEDBACK) {
		uint16_t p_raw = ((uint16_t)data[0] << 8) | data[1];
		uint16_t v_raw = ((uint16_t)data[2] << 8) | data[3];
		uint16_t t_raw = ((uint16_t)data[4] << 8) | data[5];
		uint16_t temp_raw = ((uint16_t)data[6] << 8) | data[7];

		out->position = uint16_to_float(p_raw, RS02_P_MIN, RS02_P_MAX);
		out->velocity = uint16_to_float(v_raw, RS02_V_MIN, RS02_V_MAX);
		out->torque = uint16_to_float(t_raw, RS02_T_MIN, RS02_T_MAX);
		out->temperature = (float)temp_raw / 10.0f;
	} else if (comm_mode == RS02_COMM_PARAREAD ||
	           comm_mode == RS02_COMM_PARAWRITE) {
		out->position = robstride_bytes_to_float(data);
	}
}

static bool robstride_heard_pararead_reply(const robstride_probe_result_t *out)
{
	if (out == NULL || out->raw_frames_seen == 0u)
		return false;

	return (out->comm_mode == RS02_COMM_PARAREAD ||
	        out->comm_mode == RS02_COMM_PARAWRITE);
}

static bool robstride_try_parse_para_read(const can_frame_t *frame_in,
                                          uint8_t expect_motor_id,
                                          uint16_t expect_index,
                                          bool promiscuous,
                                          robstride_probe_result_t *out)
{
	uint8_t mode, id_byte;
	uint16_t data16;

	if (frame_in == NULL || out == NULL)
		return false;

	robstride_unpack_ext_id(frame_in->id, &mode, &data16, &id_byte);
	if (mode != RS02_COMM_PARAREAD && mode != RS02_COMM_PARAWRITE)
		return false;

	if (!promiscuous &&
	    ((data16 & 0xFF) != expect_motor_id) &&
	    (id_byte != expect_motor_id))
		return false;

	out->found = true;
	out->ext_id = frame_in->id;
	out->comm_mode = mode;
	memcpy(out->data, frame_in->data, 8u);
	robstride_probe_decode_payload(mode, frame_in->data, out);
	return true;
}

static bool robstride_record_raw_ext(const can_frame_t *frame_in, robstride_probe_result_t *out)
{
	if (frame_in == NULL || out == NULL)
		return false;
	if (frame_in->id_type != CAN_ID_EXT)
		return false;

	if (out->raw_frames_seen < 255u)
		out->raw_frames_seen++;

	out->ext_id = frame_in->id;
	out->comm_mode = (uint8_t)((frame_in->id >> 24) & 0x1Fu);
	memcpy(out->data, frame_in->data, 8u);
	robstride_probe_decode_payload(out->comm_mode, frame_in->data, out);
	return true;
}

static bool robstride_try_decode_mms(const can_frame_t *frame_in,
                                    uint8_t expect_motor_id,
                                    uint8_t *mms_out)
{
	uint8_t mode, id_byte;
	uint16_t data16;

	if (frame_in == NULL || mms_out == NULL || frame_in->id_type != CAN_ID_EXT)
		return false;

	robstride_unpack_ext_id(frame_in->id, &mode, &data16, &id_byte);
	if (mode != RS02_COMM_FEEDBACK)
		return false;
	if (((data16 & 0xFF) != expect_motor_id) && (id_byte != expect_motor_id))
		return false;

	*mms_out = (uint8_t)((data16 >> 14) & 0x3u);
	return true;
}

static bool robstride_try_parse_feedback(const can_frame_t *frame_in,
                                         uint8_t expect_motor_id,
                                         bool promiscuous,
                                         robstride_probe_result_t *out)
{
	if (frame_in == NULL || out == NULL)
		return false;
	if (frame_in->id_type != CAN_ID_EXT)
		return false;

	uint8_t mode, id_byte;
	uint16_t data16;

	robstride_unpack_ext_id(frame_in->id, &mode, &data16, &id_byte);
	if (mode != RS02_COMM_FEEDBACK)
		return false;

	uint8_t discovered = (uint8_t)(data16 & 0xFF);
	if (!promiscuous && discovered != expect_motor_id && id_byte != expect_motor_id)
		return false;
	if (promiscuous)
		discovered = (discovered != 0u) ? discovered : id_byte;

	out->found = true;
	out->motor_id = expect_motor_id;
	out->discovered_id = (uint8_t)(data16 & 0xFF);
	if (out->discovered_id == 0u)
		out->discovered_id = expect_motor_id;
	out->comm_mode = mode;
	out->ext_id = frame_in->id;
	memcpy(out->data, frame_in->data, 8u);
	robstride_probe_decode_payload(mode, frame_in->data, out);
	return true;
}

void robstride_bench_note_rx(const can_frame_t *frame,
                             uint8_t motor_id,
                             robstride_probe_result_t *out)
{
	if (frame == NULL || out == NULL)
		return;

	(void)robstride_record_raw_ext(frame, out);
	(void)robstride_try_parse_feedback(frame, motor_id, false, out);
}

static plugin_status_t robstride_enqueue(can_bus_id_t bus, const can_frame_t *frame)
{
	if (frame == NULL)
		return PLUGIN_ERR_PARAM;

	return (can_tx_enqueue(bus, frame) == CAN_OK) ? PLUGIN_OK : PLUGIN_ERR_UNSUPPORTED;
}

static void robstride_poll_listen(can_bus_id_t bus)
{
	if (bus >= CAN_BUS_CH4) {
		for (uint8_t n = 0u; n < 6u; n++)
			can_router_poll_bus(bus);
	} else {
		can_router_poll_bus_rx(bus);
	}
}

static plugin_status_t robstride_probe_tx(can_bus_id_t bus, const can_frame_t *frame)
{
	if (frame == NULL)
		return PLUGIN_ERR_PARAM;

	if (bus >= CAN_BUS_CH4) {
		mcp2518_prepare_tx(bus);
		if (spi_can_router_send_now(bus, frame))
			return PLUGIN_OK;
		/* One TXQ recovery attempt before giving up. */
		mcp2518_prepare_tx(bus);
		return spi_can_router_send_now(bus, frame) ? PLUGIN_OK : PLUGIN_ERR_UNSUPPORTED;
	}

	if (robstride_enqueue(bus, frame) != PLUGIN_OK)
		return PLUGIN_ERR_UNSUPPORTED;

	can_router_poll_bus(bus);
	return PLUGIN_OK;
}

static void robstride_maintain_enable(const actuator_config_t *cfg,
                                      const actuator_desire_t *desire,
                                      uint32_t *last_ms)
{
	can_frame_t frame;
	can_bus_id_t bus;
	uint32_t now;

	if (cfg == NULL || last_ms == NULL)
		return;

	/* MIT stream keeps run/enable during velocity teleop; periodic comm frames disturb motion. */
	if (desire != NULL && desire->kp > 0.0f && fabsf(desire->velocity) > 0.01f)
		return;

	bus = cfg->bus;
	now = HAL_GetTick();
	if (*last_ms != 0u && (now - *last_ms) < 2000u)
		return;

	*last_ms = now;
	if (robstride_set_run_mode(cfg, RS02_RUN_MODE_MOVE, &frame) == PLUGIN_OK)
		(void)can_tx_enqueue(bus, &frame);
	if (robstride_send_enable(cfg, &frame) == PLUGIN_OK)
		(void)can_tx_enqueue(bus, &frame);
}

static uint8_t robstride_actuator_slot(const actuator_config_t *cfg)
{
	if (cfg == NULL)
		return 0u;

	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		if (actuator_table[i].bus == cfg->bus &&
		    actuator_table[i].motor_id == cfg->motor_id)
			return i;
	}
	return 0u;
}

static bool robstride_desire_idle(const actuator_desire_t *desire)
{
	if (desire == NULL)
		return true;

	return (desire->position == 0.0f && desire->velocity == 0.0f &&
	        desire->kp == 0.0f && desire->kd == 0.0f && desire->torque == 0.0f);
}

static void robstride_apply_drain_rx(const actuator_config_t *cfg,
                                     uint8_t slot,
                                     uint8_t pararead_phase,
                                     uint16_t last_pararead_idx,
                                     actuator_state_t *state_out)
{
	can_frame_t frame;
	can_bus_id_t bus = cfg->bus;

	while (can_rx_pop(bus, &frame) == CAN_OK) {
		if (state_out == NULL)
			continue;
		if (plugin_parse_rx(cfg, &frame, state_out) == PLUGIN_OK)
			continue;
		if (frame.id_type != CAN_ID_EXT)
			continue;

		uint8_t mode, id_byte;
		uint16_t data16;

		robstride_unpack_ext_id(frame.id, &mode, &data16, &id_byte);
		if (mode != RS02_COMM_PARAREAD)
			continue;

		uint8_t motor_id = (uint8_t)(cfg->motor_id & 0xFF);
		if (((data16 & 0xFF) != motor_id) && (id_byte != motor_id))
			continue;

		float val;
		uint32_t raw = (uint32_t)frame.data[4] |
		               ((uint32_t)frame.data[5] << 8) |
		               ((uint32_t)frame.data[6] << 16) |
		               ((uint32_t)frame.data[7] << 24);

		memcpy(&val, &raw, sizeof(val));
		if (last_pararead_idx == RS02_PARAM_MECH_POS)
			state_out->position = val;
		else if (last_pararead_idx == RS02_PARAM_MECH_VEL)
			state_out->velocity = val;
		(void)slot;
		(void)pararead_phase;
	}
}

void robstride_apply_cycle(const actuator_config_t *cfg,
                           const actuator_desire_t *desire,
                           actuator_state_t *state_out)
{
	static uint32_t last_maintain_ms[ACTUATOR_COUNT];
	static uint32_t last_pararead_ms[ACTUATOR_COUNT];
	static uint8_t pararead_phase[ACTUATOR_COUNT];
	static uint16_t last_pararead_idx[ACTUATOR_COUNT];
	can_frame_t frame;
	can_bus_id_t bus;
	uint8_t slot;
	bool idle;
	bool mcp;
	uint8_t tx_burst;

	if (cfg == NULL || desire == NULL || !cfg->enabled)
		return;

	slot = robstride_actuator_slot(cfg);
	if (slot >= ACTUATOR_COUNT)
		slot = 0u;

	bus = cfg->bus;
	mcp = (bus >= CAN_BUS_CH4);
	idle = robstride_desire_idle(desire);
	/* MCP: one immediate MIT per 2 ms tick; FDCAN: triple enqueue + HW FIFO. */
	tx_burst = mcp ? 1u : 3u;

	can_router_poll_bus_rx(bus);

	if (idle) {
		robstride_apply_drain_rx(cfg, slot, pararead_phase[slot],
		                         last_pararead_idx[slot], state_out);
		if (mcp) {
			while (spi_can_router_tx_flush(bus) == CAN_OK)
				;
		}
		return;
	}

	robstride_maintain_enable(cfg, desire, &last_maintain_ms[slot]);

	for (uint8_t i = 0; i < tx_burst; i++) {
		if (plugin_pack_tx(cfg, desire, &frame) != PLUGIN_OK)
			continue;
		if (mcp)
			(void)spi_can_router_send_now(bus, &frame);
		else
			(void)can_tx_enqueue(bus, &frame);
	}

	if (mcp) {
		while (spi_can_router_tx_flush(bus) == CAN_OK)
			;
	}

	if (!mcp) {
		uint32_t now = HAL_GetTick();

		if (last_pararead_ms[slot] == 0u || (now - last_pararead_ms[slot]) >= 50u) {
			last_pararead_ms[slot] = now;
			last_pararead_idx[slot] = (pararead_phase[slot] == 0u) ?
			                          RS02_PARAM_MECH_POS : RS02_PARAM_MECH_VEL;
			pararead_phase[slot] ^= 1u;
			if (robstride_send_para_read(cfg, last_pararead_idx[slot], &frame) == PLUGIN_OK)
				(void)can_tx_enqueue(bus, &frame);
		}
	}

	can_router_poll_bus(bus);
	robstride_apply_drain_rx(cfg, slot, pararead_phase[slot],
	                         last_pararead_idx[slot], state_out);
}

static void robstride_listen_rx(can_bus_id_t bus,
                                uint8_t motor_id,
                                bool promiscuous,
                                uint16_t listen_ms,
                                robstride_probe_result_t *out)
{
	can_frame_t frame;

	for (uint16_t attempt = 0; attempt < listen_ms; attempt++) {
		robstride_poll_listen(bus);
		while (can_rx_pop(bus, &frame) == CAN_OK) {
			if (robstride_try_parse_feedback(&frame, motor_id, promiscuous, out)) {
				continue;
			}
			if (!out->found)
				(void)robstride_record_raw_ext(&frame, out);
			else if (out->raw_frames_seen < 255u)
				out->raw_frames_seen++;
		}
		plant_diag_yield_usb();
		HAL_Delay(1);
	}
}

bool robstride_probe_id(can_bus_id_t bus,
                        uint8_t motor_id,
                        uint8_t probe_kind,
                        const actuator_desire_t *desire_in,
                        uint16_t param_index,
                        uint32_t param_raw_value,
                        robstride_probe_result_t *out)
{
	if (out == NULL)
		return false;

	if (bus >= CAN_BACKEND_COUNT)
		bus = CAN_BUS_CH1;

	actuator_config_t cfg = {
		.bus = bus,
		.protocol = PROTO_ROBSTRIDE,
		.motor_id = motor_id,
		.enabled = true,
	};
	static const actuator_desire_t desire_default = {
		.position = 0.0f,
		.velocity = 0.0f,
		.kp = 50.0f,
		.kd = 1.0f,
		.torque = 0.0f,
	};
	const actuator_desire_t *desire = (desire_in != NULL) ? desire_in : &desire_default;
	can_frame_t frame;
	static uint32_t ctrl_fast_maintain_ms;
	bool promiscuous = (probe_kind == RS02_PROBE_PROMISC);
	bool reset_only = (probe_kind == RS02_PROBE_RESET);
	bool enable_only = (probe_kind == RS02_PROBE_ENABLE_ONLY);
	bool ctrl_fast = (probe_kind == RS02_PROBE_CTRL_FAST);
	bool para_read = (probe_kind == RS02_PROBE_PARAREAD);
	bool para_write = (probe_kind == RS02_PROBE_PARAWRITE);
	bool proactive = (probe_kind == RS02_PROBE_PROACTIVE);
	bool cali = (probe_kind == RS02_PROBE_CALI);
	bool zero_cmd = (probe_kind == RS02_PROBE_ZERO);
	bool data_save = (probe_kind == RS02_PROBE_DATA_SAVE);
	bool bench_cmd = cali || zero_cmd || data_save;
	bool do_run_mode = !reset_only && !ctrl_fast && !para_read && !para_write && !proactive && !bench_cmd &&
	                   (probe_kind <= RS02_PROBE_FULL || promiscuous || enable_only);
	bool do_enable = !reset_only && !ctrl_fast && !para_read && !para_write && !proactive && !bench_cmd &&
	                 (probe_kind <= RS02_PROBE_ENABLE_CTRL || promiscuous || enable_only);
	bool do_ctrl = !reset_only && !enable_only && !para_read && !para_write && !proactive && !bench_cmd;
	uint16_t listen_ms = enable_only ? 200u :
	                     (ctrl_fast ? 0u :
	                      (para_read ? 220u :
	                       (para_write ? 100u :
	                        (proactive ? 50u :
	                         (cali ? 0u :
	                          (zero_cmd ? 500u :
	                         (data_save ? 800u : 150u)))))));
	if (bus >= CAN_BUS_CH4)
		listen_ms = (uint16_t)(listen_ms + listen_ms / 2u + 120u);

	if (!ctrl_fast) {
		memset(out, 0, sizeof(*out));
		out->motor_id = motor_id;
		out->probe_kind = probe_kind;
		can_rx_drain(bus);
	} else {
		out->motor_id = motor_id;
		out->probe_kind = probe_kind;
	}

	if (para_read) {
		if (param_index == 0u)
			param_index = RS02_PARAM_MECH_ANGLE;

		robstride_maintain_enable(&cfg, NULL, &ctrl_fast_maintain_ms);
		can_router_poll_bus(bus);
		HAL_Delay(3);
		can_rx_drain(bus);

		for (uint16_t attempt = 0; attempt < listen_ms; attempt++) {
			if (attempt == 0u || (attempt % 50u) == 0u) {
				if (robstride_send_para_read(&cfg, param_index, &frame) != PLUGIN_OK)
					return false;
				if (robstride_probe_tx(bus, &frame) != PLUGIN_OK)
					return false;
				HAL_Delay(2);
			}
			robstride_poll_listen(bus);
			while (can_rx_pop(bus, &frame) == CAN_OK) {
				(void)robstride_record_raw_ext(&frame, out);
				if (robstride_try_parse_para_read(&frame, motor_id, param_index,
				                                  true, out))
					return true;
			}
			plant_diag_yield_usb();
			HAL_Delay(1);
		}

		return robstride_heard_pararead_reply(out);
	}

	if (para_write) {
		if (param_index == 0u)
			return false;

		can_router_poll_bus(bus);
		HAL_Delay(3);
		can_rx_drain(bus);

		if (robstride_send_para_write(&cfg, param_index, param_raw_value, &frame) != PLUGIN_OK)
			return false;
		if (robstride_probe_tx(bus, &frame) != PLUGIN_OK)
			return false;
		HAL_Delay(5);

		for (uint16_t attempt = 0; attempt < listen_ms; attempt++) {
			robstride_poll_listen(bus);
			while (can_rx_pop(bus, &frame) == CAN_OK) {
				if (!robstride_try_parse_para_read(&frame, motor_id, param_index, true, out)) {
					if (!out->found)
						(void)robstride_record_raw_ext(&frame, out);
					else if (out->raw_frames_seen < 255u)
						out->raw_frames_seen++;
				}
			}
			plant_diag_yield_usb();
			HAL_Delay(1);
		}

		return (out->raw_frames_seen > 0u);
	}

	if (proactive) {
		robstride_maintain_enable(&cfg, NULL, &ctrl_fast_maintain_ms);
		can_router_poll_bus(bus);
		HAL_Delay(3);

		/* param_index=1 disables proactive; 0 or any other value enables it. */
		uint8_t proactive_en = (param_index == 1u) ? 0u : 1u;
		if (robstride_send_proactive(&cfg, proactive_en, &frame) != PLUGIN_OK)
			return false;
		if (robstride_probe_tx(bus, &frame) != PLUGIN_OK)
			return false;
		HAL_Delay(5);

		for (uint16_t attempt = 0; attempt < listen_ms; attempt++) {
			robstride_poll_listen(bus);
			while (can_rx_pop(bus, &frame) == CAN_OK) {
				if (!robstride_try_parse_feedback(&frame, motor_id, false, out)) {
					if (!out->found)
						(void)robstride_record_raw_ext(&frame, out);
					else if (out->raw_frames_seen < 255u)
						out->raw_frames_seen++;
				}
			}
			plant_diag_yield_usb();
			HAL_Delay(1);
		}

		return true;
	}

	if (cali) {
		/* Motor must be in REST to accept comm=0x05.
		 * param_index low byte = listen seconds (0 → 90 s default).
		 * param_index bit 15 = listen-only (no reset / no re-TX 0x05) for host chunks.
		 * Loop exits early when mms returns to rest or running after cali. */
		bool listen_only = ((param_index & 0x8000u) != 0u);
		bool skip_reset = ((param_index & 0x4000u) != 0u);
		uint32_t cal_s = (uint32_t)(param_index & 0xFFu);
		if (cal_s == 0u)
			cal_s = 90u;

		if (!listen_only) {
			if (!skip_reset) {
				if (robstride_send_reset(&cfg, &frame) != PLUGIN_OK)
					return false;
				if (robstride_probe_tx(bus, &frame) != PLUGIN_OK)
					return false;
				HAL_Delay(500u);
				can_rx_drain(bus);
			}

			if (robstride_send_cali(&cfg, &frame) != PLUGIN_OK)
				return false;
			if (robstride_probe_tx(bus, &frame) != PLUGIN_OK)
				return false;
			if (bus >= CAN_BUS_CH4) {
				for (uint8_t settle = 0u; settle < 12u; settle++) {
					can_router_poll_bus(bus);
					HAL_Delay(2);
				}
			} else {
				HAL_Delay(5);
			}
		}

		{
			uint32_t deadline = HAL_GetTick() + cal_s * 1000u;
			uint32_t last_progress_ms = 0u;
			bool saw_cali = false;
			bool cal_done = false;
			bool had_spin = false;
			uint32_t spin_settled_ms = 0u;
			uint32_t saw_cali_at_ms = 0u;
			uint32_t fb_static_ms = 0u;
			uint32_t last_ext_id = 0u;
			uint8_t last_data[8] = {0};
			do {
				robstride_poll_listen(bus);
				while (can_rx_pop(bus, &frame) == CAN_OK) {
					uint8_t mms = 0u;
					if (robstride_try_decode_mms(&frame, motor_id, &mms)) {
						if (mms == 1u) {
							saw_cali = true;
							if (saw_cali_at_ms == 0u)
								saw_cali_at_ms = HAL_GetTick();
						}
						if (saw_cali && (mms == 0u || mms == 2u))
							cal_done = true;
					}
					if (robstride_try_parse_feedback(&frame, motor_id, false, out)) {
						out->found = true;
						if (saw_cali && fabsf(out->velocity) > 0.06f)
							had_spin = true;
					} else if (!out->found) {
						(void)robstride_record_raw_ext(&frame, out);
					} else if (out->raw_frames_seen < 255u) {
						out->raw_frames_seen++;
					}
				}
				if (cal_done)
					break;

				if (saw_cali && had_spin && out->found) {
					if (fabsf(out->velocity) < 0.20f) {
						if (spin_settled_ms == 0u)
							spin_settled_ms = HAL_GetTick();
						else if ((HAL_GetTick() - spin_settled_ms) >= 2000u &&
						         saw_cali_at_ms != 0u &&
						         (HAL_GetTick() - saw_cali_at_ms) >= 8000u)
							cal_done = true;
					} else {
						spin_settled_ms = 0u;
					}
				}

				/* Static comm0x02 repeats during cal — require min spin window first. */
				if (saw_cali && out->found && saw_cali_at_ms != 0u &&
				    (HAL_GetTick() - saw_cali_at_ms) >= 15000u &&
				    out->raw_frames_seen >= 8u) {
					if (out->ext_id != last_ext_id ||
					    memcmp(out->data, last_data, 8u) != 0) {
						last_ext_id = out->ext_id;
						memcpy(last_data, out->data, 8u);
						fb_static_ms = 0u;
					} else if (fb_static_ms == 0u) {
						fb_static_ms = HAL_GetTick();
					} else if ((HAL_GetTick() - fb_static_ms) >= 4000u) {
						cal_done = true;
					}
				}

				uint32_t now = HAL_GetTick();
				if (last_progress_ms == 0u ||
				    (now - last_progress_ms) >= 500u) {
					last_progress_ms = now;
					out->probe_kind = probe_kind;
					out->motor_id = motor_id;
					plant_diag_probe_progress(out);
				}

				plant_diag_yield_usb();
				HAL_Delay(1);
			} while ((int32_t)(deadline - HAL_GetTick()) > 0);

			return cal_done;
		}
	}

	if (zero_cmd) {
		if (robstride_send_reset(&cfg, &frame) != PLUGIN_OK)
			return false;
		if (robstride_probe_tx(bus, &frame) != PLUGIN_OK)
			return false;
		HAL_Delay(10);

		if (robstride_send_zero(&cfg, &frame) != PLUGIN_OK)
			return false;
		if (robstride_probe_tx(bus, &frame) != PLUGIN_OK)
			return false;
		HAL_Delay(3);

		robstride_listen_rx(bus, motor_id, false, listen_ms, out);
		return out->found;
	}

	if (data_save) {
		if (robstride_send_data_save(&cfg, &frame) != PLUGIN_OK)
			return false;
		if (robstride_probe_tx(bus, &frame) != PLUGIN_OK)
			return false;
		HAL_Delay(5);

		robstride_listen_rx(bus, motor_id, false, listen_ms, out);
		return out->found;
	}

	if (ctrl_fast) {
		robstride_maintain_enable(&cfg, NULL, &ctrl_fast_maintain_ms);
		if (plugin_pack_tx(&cfg, desire, &frame) != PLUGIN_OK)
			return false;
		if (robstride_probe_tx(bus, &frame) != PLUGIN_OK)
			return false;
		while (can_rx_pop(bus, &frame) == CAN_OK) {
			(void)robstride_record_raw_ext(&frame, out);
			(void)robstride_try_parse_feedback(&frame, motor_id, false, out);
		}
		return (out->raw_frames_seen > 0u);
	}

	if (reset_only) {
		if (robstride_send_reset(&cfg, &frame) != PLUGIN_OK)
			return false;
		if (robstride_probe_tx(bus, &frame) != PLUGIN_OK)
			return false;
		HAL_Delay(10);
	} else {
		/* rs02_can_scan WAKE_SEQUENCE / --bench-cmds: reset before full init. */
		if (probe_kind == RS02_PROBE_FULL || probe_kind == RS02_PROBE_ENABLE_CTRL) {
			if (robstride_send_reset(&cfg, &frame) != PLUGIN_OK)
				return false;
			if (robstride_probe_tx(bus, &frame) != PLUGIN_OK)
				return false;
			HAL_Delay(10);
			can_rx_drain(bus);
		}

		if (do_run_mode) {
			if (robstride_set_run_mode(&cfg, RS02_RUN_MODE_MOVE, &frame) != PLUGIN_OK)
				return false;
			if (robstride_probe_tx(bus, &frame) != PLUGIN_OK)
				return false;
			HAL_Delay(2);
		}

		if (do_enable) {
			if (robstride_send_enable(&cfg, &frame) != PLUGIN_OK)
				return false;
			if (robstride_probe_tx(bus, &frame) != PLUGIN_OK)
				return false;
			HAL_Delay(2);
		}
	}

	if (do_ctrl) {
		if (plugin_pack_tx(&cfg, desire, &frame) != PLUGIN_OK)
			return false;
		if (robstride_probe_tx(bus, &frame) != PLUGIN_OK)
			return false;
	}

	if (probe_kind == RS02_PROBE_FULL && listen_ms < 200u)
		listen_ms = 200u;

	for (uint16_t attempt = 0; attempt < listen_ms; attempt++) {
		robstride_poll_listen(bus);
		while (can_rx_pop(bus, &frame) == CAN_OK) {
			(void)robstride_record_raw_ext(&frame, out);
			if (robstride_try_parse_feedback(&frame, motor_id, promiscuous, out))
				return true;
		}
		plant_diag_yield_usb();
		HAL_Delay(1);
	}

	return (out->raw_frames_seen > 0u);
}

const plugin_ops_t robstride_ops = {
	.pack_tx = robstride_pack_tx,
	.parse_rx = robstride_parse_rx,
};
