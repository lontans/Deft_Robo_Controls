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
#define RS02_COMM_MOTOR_CALI  0x05
#define RS02_COMM_MOTOR_ZERO  0x06
#define RS02_COMM_DATA_SAVE   0x16
#define RS02_COMM_PARAREAD    0x11
#define RS02_COMM_PARAWRITE   0x12
#define RS02_COMM_PROACTIVE   0x18

#define RS02_PARAM_RUN_MODE   0x7005u
#define RS02_PARAM_MECH_ANGLE 0x7016u
#define RS02_PARAM_MECH_POS   0x7019u
#define RS02_PARAM_MECH_VEL   0x701Bu
#define RS02_PARAM_BUS_VOLT   0x701Cu
#define RS02_PARAM_IQ_TEST    0x702Du
#define RS02_RUN_MODE_MOVE    0u

#define RS02_PROBE_FULL        0u
#define RS02_PROBE_ENABLE_CTRL 1u
#define RS02_PROBE_CTRL_ONLY   2u
#define RS02_PROBE_PROMISC     10u
#define RS02_PROBE_RESET       11u
#define RS02_PROBE_ENABLE_ONLY 12u
#define RS02_PROBE_CTRL_FAST   13u
#define RS02_PROBE_PARAREAD    14u
#define RS02_PROBE_PROACTIVE   15u
#define RS02_PROBE_CALI        16u
#define RS02_PROBE_ZERO        17u
#define RS02_PROBE_DATA_SAVE   18u
#define RS02_PROBE_PARAWRITE   19u

typedef struct {
	bool found;
	uint8_t motor_id;
	uint8_t discovered_id;
	uint8_t comm_mode;
	uint8_t probe_kind;
	uint32_t ext_id;
	uint8_t data[8];
	uint8_t raw_frames_seen;
	float position;
	float velocity;
	float torque;
	float temperature;
} robstride_probe_result_t;

plugin_status_t robstride_set_run_mode(const actuator_config_t *cfg,
                                       uint8_t run_mode,
                                       can_frame_t *frame_out);
plugin_status_t robstride_send_enable(const actuator_config_t *cfg, can_frame_t *frame_out);
plugin_status_t robstride_send_reset(const actuator_config_t *cfg, can_frame_t *frame_out);
plugin_status_t robstride_send_cali(const actuator_config_t *cfg, can_frame_t *frame_out);
plugin_status_t robstride_send_zero(const actuator_config_t *cfg, can_frame_t *frame_out);
plugin_status_t robstride_send_data_save(const actuator_config_t *cfg, can_frame_t *frame_out);
plugin_status_t robstride_send_para_read(const actuator_config_t *cfg,
                                         uint16_t param_index,
                                         can_frame_t *frame_out);
plugin_status_t robstride_send_para_write(const actuator_config_t *cfg,
                                          uint16_t param_index,
                                          uint32_t raw_value,
                                          can_frame_t *frame_out);
plugin_status_t robstride_send_proactive(const actuator_config_t *cfg,
                                         uint8_t enable,
                                         can_frame_t *frame_out);
void robstride_apply_cycle(const actuator_config_t *cfg,
                           const actuator_desire_t *desire,
                           actuator_state_t *state_out);
void robstride_bench_note_rx(const can_frame_t *frame,
                             uint8_t motor_id,
                             robstride_probe_result_t *out);
bool robstride_probe_id(can_bus_id_t bus,
                        uint8_t motor_id,
                        uint8_t probe_kind,
                        const actuator_desire_t *desire_in,
                        uint16_t param_index,
                        uint32_t param_raw_value,
                        robstride_probe_result_t *out);
