#include "plant/plugin_schema/plugin.h"
#include "plant/plugins/robstride.h"
#include <stddef.h>
#include <stdint.h>

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

	uint8_t motor_id = (uint8_t)(cfg->motor_id & 0xFF);

	if (mode != RS02_COMM_FEEDBACK)
		return PLUGIN_ERR_UNSUPPORTED;
	if ((data16 & 0xFF) != motor_id)
		return PLUGIN_ERR_UNSUPPORTED;
	if (id_byte != RS02_HOST_ID)
		return PLUGIN_ERR_UNSUPPORTED;

	if (frame_in->dlc < 8)
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

const plugin_ops_t robstride_ops = {
	.pack_tx = robstride_pack_tx,
	.parse_rx = robstride_parse_rx,
};
