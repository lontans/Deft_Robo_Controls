#pragma once
#include "plant/actuator.h"
#include "plant/plugin_schema/plugin_types.h"
#include "plant/can/can_frame.h"

#define RS02_P_MIN  (-12.57f)
#define RS02_P_MAX  (12.57f)
#define RS02_V_MIN  (-44.0f)
#define RS02_V_MAX  (44.0f)
#define RS02_KP_MIN (0.0f)
#define RS02_KP_MAX (500.0f)
#define RS02_KD_MIN (0.0f)
#define RS02_KD_MAX (5.0f)
#define RS02_T_MIN  (-17.0f)
#define RS02_T_MAX  (17.0f)

#define RS02_HOST_ID 0xFD

#define RS02_COMM_GET_ID      0x00
#define RS02_COMM_MOTOR_CTRL  0x01
#define RS02_COMM_FEEDBACK    0x02
#define RS02_COMM_MOTOR_IN    0x03
#define RS02_COMM_MOTOR_RESET 0x04

plugin_status_t robstride_send_enable(const actuator_config_t *cfg, can_frame_t *frame_out);
