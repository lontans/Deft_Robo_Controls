#pragma once
#include "actuator.h"
#include "plugin_types.h"
#include "can_frame.h"

// Hardware RS-02 limits per RS-02 datasheet, probably not edited by Jetson
#define RS02_P_MIN  (-12.57f)
#define RS02_P_MAX  (12.57f)
#define RS02_V_MIN  (-44.0f)
#define RS02_V_MAX  (44.0f)  // this is max rad/s
#define RS02_KP_MIN (0.0f)
#define RS02_KP_MAX (500.0f)
#define RS02_KD_MIN (0.0f)
#define RS02_KD_MAX (5.0f)
#define RS02_T_MIN  (-17.0f)
#define RS02_T_MAX  (17.0f)  // Maximum torque

#define RS02_HOST_ID 0xFD    // Gateway host ID

// declaring RS02 datasheet commands, more to add
#define RS02_COMM_GET_ID      0x00
#define RS02_COMM_MOTOR_CTRL  0x01
#define RS02_COMM_FEEDBACK    0x02
#define RS02_COMM_MOTOR_IN    0x03
#define RS02_COMM_MOTOR_RESET 0x04

/* robstride_ops lives in robstride.c; plugin_table.c declares extern */
plugin_status_t robstride_send_enable(const actuator_config_t *cfg, can_frame_t *frame_out);
