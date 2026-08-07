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
#define RS02_COMM_SET_CAN_ID  0x07
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
/* 20/21/22 taken by PLANT_DIAG_PROBE_MCP_SMOKE/WAKE/DISABLE (diag.h) — same
 * kind-byte namespace, not RS02-specific probes. */
#define RS02_PROBE_SET_CAN_ID  23u

static inline bool rs02_probe_kind_mounts_desire(uint8_t kind)
{
	switch (kind) {
	case RS02_PROBE_FULL:
	case RS02_PROBE_ENABLE_CTRL:
	case RS02_PROBE_CTRL_ONLY:
	case RS02_PROBE_CTRL_FAST:
		return true;
	default:
		return false;
	}
}

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
	/* Host bus 1..6 when multi-bus discover attributed a hit; 0 = unset. */
	uint8_t can_bus;
} robstride_probe_result_t;

plugin_status_t robstride_set_run_mode(const actuator_config_t *cfg,
                                       uint8_t run_mode,
                                       can_frame_t *frame_out);
plugin_status_t robstride_send_enable(const actuator_config_t *cfg, can_frame_t *frame_out);
plugin_status_t robstride_send_reset(const actuator_config_t *cfg, can_frame_t *frame_out);
plugin_status_t robstride_send_disable(const actuator_config_t *cfg, can_frame_t *frame_out);
plugin_status_t robstride_send_cali(const actuator_config_t *cfg, can_frame_t *frame_out);
plugin_status_t robstride_send_zero(const actuator_config_t *cfg, can_frame_t *frame_out);
plugin_status_t robstride_send_data_save(const actuator_config_t *cfg, can_frame_t *frame_out);
/* comm=0x07 (vendor RobStride_Set_CAN_ID): motor must be reset/at rest first
 * (caller's job, see RS02_PROBE_SET_CAN_ID handling). Payload is all-zero;
 * new_can_id rides in the ext-ID data field, cfg->motor_id is the OLD id. */
plugin_status_t robstride_send_set_can_id(const actuator_config_t *cfg,
                                          uint8_t new_can_id,
                                          can_frame_t *frame_out);
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
/* commanded_buses: bit N set ⇒ schematic CH(N+1) has a non-blank hold this tick. */
void robstride_plant_tick_begin(uint32_t commanded_buses);
void robstride_apply_cycle(const actuator_config_t *cfg,
                           const actuator_desire_t *desire,
                           actuator_state_t *state_out,
                           uint8_t slot);
/* After all apply_cycle calls: one prepare_tx + flush per MCP bus that enqueued. */
void robstride_mcp_flush_pending(void);
void robstride_on_rx_frame(const actuator_config_t *cfg, uint8_t slot,
                           const can_frame_t *frame, actuator_state_t *state_out);
void robstride_host_desire_updated(uint8_t slot, const actuator_desire_t *desire);
/* Bench/matrix: pack a synthetic RS02 feedback frame (no bus TX). */
bool robstride_pack_feedback(const actuator_config_t *cfg,
                             float position,
                             float velocity,
                             float torque,
                             float temp_c,
                             can_frame_t *frame_out);
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
/* ENABLE_ONLY / PROMISC across bus_mask (bit0=CH1…): TX all, one RR listen. */
bool robstride_probe_id_buses(uint8_t bus_mask,
                              uint8_t motor_id,
                              uint8_t probe_kind,
                              robstride_probe_result_t *out);
