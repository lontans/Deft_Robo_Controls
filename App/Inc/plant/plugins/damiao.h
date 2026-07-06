#pragma once

#include "plant/actuator.h"

#include "plant/plugin_schema/plugin_types.h"

#include "plant/can/can_frame.h"

#include <stdbool.h>



/* MIT linear map limits — must match Damiao assistant P_MAX / V_MAX / T_MAX. */

#define DM4310_P_MIN  (-12.5f)

#define DM4310_P_MAX  ( 12.5f)

#define DM4310_V_MIN  (-30.0f)

#define DM4310_V_MAX  ( 30.0f)

#define DM4310_T_MIN  (-10.0f)

#define DM4310_T_MAX  ( 10.0f)



#define DM_KP_MIN     (0.0f)

#define DM_KP_MAX     (500.0f)

#define DM_KD_MIN     (0.0f)

#define DM_KD_MAX     (5.0f)



#define DM_MASTER_ID_DEFAULT      0u

#define DM_MASTER_ID_ANY          0xFFu

#define DM_MASTER_ID_AUTO         0xFFFFFFFFu



#define DM_CAN_BROADCAST_ID       0x7FFu



/* Register addresses (DM-J4310 manual register map). */

#define DM_REG_MST_ID             0x07u

#define DM_REG_ESC_ID             0x08u

#define DM_REG_CTRL_MODE          0x0Au



/* Probe kinds (host PDU data[4]). */

#define DM_PROBE_MIT              0u

#define DM_PROBE_POS              1u   /* std id = 0x100 + motor_id */

#define DM_PROBE_VEL              2u   /* std id = 0x200 + motor_id */

#define DM_PROBE_ALL              3u   /* MIT, then POS, then VEL */

#define DM_PROBE_PROMISC          10u  /* accept any motor id in feedback */

#define DM_PROBE_ENABLE           11u  /* FF..FF FC on std id=motor_id */

#define DM_PROBE_DISABLE          12u  /* FF..FF FD */

#define DM_PROBE_CLEAR_FAULT      13u  /* FF..FF FB */

#define DM_PROBE_READ_PARAM       14u  /* 0x7FF read; param RID in PDU data[7] */

#define DM_PROBE_DISCOVER         15u  /* clear fault, enable, then ALL modes */

#define DM_PROBE_REG_SCAN         16u  /* 0x7FF read ESC_ID + MST_ID; master=ANY */



typedef struct {

	bool     found;

	uint8_t  motor_id;

	uint8_t  discovered_id;

	uint8_t  master_id;

	uint8_t  probe_kind;

	uint8_t  err;

	uint8_t  raw_frames_seen;

	uint8_t  tx_frames_sent;

	uint8_t  param_rid;

	uint32_t param_value;

	uint32_t rx_can_id;

	uint8_t  data[8];

	float    position;

	float    velocity;

	float    torque;

	float    temperature;

} damiao_probe_result_t;



uint32_t damiao_master_id_resolve(const actuator_config_t *cfg);



plugin_status_t damiao_send_enable(const actuator_config_t *cfg, can_frame_t *frame_out);

plugin_status_t damiao_send_disable(const actuator_config_t *cfg, can_frame_t *frame_out);

plugin_status_t damiao_send_clear_fault(const actuator_config_t *cfg, can_frame_t *frame_out);

plugin_status_t damiao_send_set_zero(const actuator_config_t *cfg, can_frame_t *frame_out);

plugin_status_t damiao_send_param_read(const actuator_config_t *cfg,

                                       uint8_t register_id,

                                       can_frame_t *frame_out);



void damiao_apply_cycle(const actuator_config_t *cfg,

                        const actuator_desire_t *desire,

                        actuator_state_t *state_out);



bool damiao_probe_id(can_bus_id_t bus,

                     uint8_t motor_id,

                     uint8_t probe_kind,

                     uint8_t master_id_filter,

                     uint8_t param_rid,

                     uint16_t listen_ms,

                     damiao_probe_result_t *out);

