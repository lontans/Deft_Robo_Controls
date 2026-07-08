#pragma once
#include "host/host_exchange_schema.h"
#include "plant/plugins/robstride.h"
#include <stdbool.h>
#include <stdint.h>

/* Bench PDU (562 B image pdu.data[32]). */
#define PLANT_DIAG_PDU_TAG0          'R'
#define PLANT_DIAG_PDU_TAG1          'S'
#define PLANT_DIAG_PDU_TAG2          '2'
#define PLANT_DIAG_PDU_RESP_TAG      'r'
#define PLANT_DIAG_DXL_TAG0          'D'
#define PLANT_DIAG_DXL_TAG1          'X'
#define PLANT_DIAG_DXL_TAG2          'L'
#define PLANT_DIAG_DM_TAG0           'D'
#define PLANT_DIAG_DM_TAG1           'M'
#define PLANT_DIAG_DM_TAG2           '0'
#define PLANT_DIAG_DM_RESP_TAG       'm'
#define PLANT_DIAG_DM_PDU_MASTER_ID  5u
#define PLANT_DIAG_DM_PDU_LISTEN_MS  6u
#define PLANT_DIAG_DM_PDU_PARAM_RID  7u
#define PLANT_DIAG_DM_PDU_END_ID     8u
#define PLANT_DM_FB_MAGIC            0xDA000000u
#define PLANT_DM_FB_TTL_PROBE        250u
#define PLANT_DM_FB_TTL_SESSION      64u
#define PLANT_DM_FW_MARKER0          'D'
#define PLANT_DM_FW_MARKER1          '1'
#define PLANT_DXL_PROBE_SCAN         1u
#define PLANT_DXL_PROBE_PING         2u
#define PLANT_DXL_PROBE_FIND_BAUD    3u
#define PLANT_DIAG_PROBE_FULL        RS02_PROBE_FULL
#define PLANT_DIAG_PROBE_ENABLE_CTRL RS02_PROBE_ENABLE_CTRL
#define PLANT_DIAG_PROBE_CTRL_ONLY   RS02_PROBE_CTRL_ONLY
#define PLANT_DIAG_PROBE_PROMISC     RS02_PROBE_PROMISC
#define PLANT_DIAG_PROBE_RESET       RS02_PROBE_RESET
#define PLANT_DIAG_PROBE_ENABLE_ONLY RS02_PROBE_ENABLE_ONLY
#define PLANT_DIAG_PROBE_CTRL_FAST   RS02_PROBE_CTRL_FAST
#define PLANT_DIAG_PROBE_PARAREAD    RS02_PROBE_PARAREAD
#define PLANT_DIAG_PROBE_PROACTIVE   RS02_PROBE_PROACTIVE
#define PLANT_DIAG_PROBE_CALI        RS02_PROBE_CALI
#define PLANT_DIAG_PROBE_ZERO        RS02_PROBE_ZERO
#define PLANT_DIAG_PROBE_DATA_SAVE   RS02_PROBE_DATA_SAVE
#define PLANT_DIAG_PROBE_PARAWRITE   RS02_PROBE_PARAWRITE
#define PLANT_DIAG_PROBE_MCP_SMOKE   20u
#define PLANT_DIAG_PROBE_MCP_WAKE    21u
#define PLANT_DIAG_PROBE_MCP_DISABLE 22u
#define PLANT_DXL_PROBE_TOGGLE_BAUD  4u
#define PLANT_DXL_PROBE_SET_BAUD_1M  PLANT_DXL_PROBE_TOGGLE_BAUD
#define PLANT_DIAG_SESSION_BEGIN     254u
#define PLANT_DIAG_SESSION_END       255u
#define PLANT_DIAG_PDU_CAN_BUS       11u
#define PLANT_DIAG_RS2_QUIET_MS      3000u
#define PLANT_DIAG_DM_QUIET_MS       3000u

/* Why 500 Hz actuator apply was skipped (stamped in feedback system.reserved). */
typedef enum {
	PLANT_BLOCK_NONE = 0,
	PLANT_BLOCK_BENCH_SESSION = 1,
	PLANT_BLOCK_PROBE_BUSY = 2,
	PLANT_BLOCK_QUIET_PERIOD = 3,
	PLANT_BLOCK_DIAG_ONLY = 4,
	PLANT_BLOCK_HOST_STALE = 5,
	PLANT_BLOCK_SERVO_SESSION = 6,
} plant_block_reason_t;

void plant_diag_release_actuator_can(void);
bool plant_diag_bench_session_active(void);
bool plant_diag_probe_busy(void);
bool plant_diag_quiet_period_active(void);
bool plant_diag_skip_actuator_can(void);
bool plant_diag_skip_servo_bus(void);
bool plant_diag_is_rs2_command(const host_command_image_t *cmd);
bool plant_diag_is_dxl_command(const host_command_image_t *cmd);
bool plant_diag_is_dm_command(const host_command_image_t *cmd);
void plant_diag_on_command(const host_command_image_t *cmd);
void plant_diag_on_dxl_command(const host_command_image_t *cmd);
void plant_diag_on_dm_command(const host_command_image_t *cmd);
void plant_diag_service(void);
void plant_diag_can_router_poll(void);
void plant_diag_yield_usb(void);
bool plant_diag_blocks_usb_feedback(void);
void plant_diag_probe_progress(const robstride_probe_result_t *partial);
void plant_diag_feedback_sent(uint8_t probe_kind);
void plant_diag_feedback_fill(host_pdu_feedback_t *pdu);
void plant_diag_feedback_stamp_fw_marker(host_pdu_feedback_t *pdu);

bool plant_runtime_actuator_can_apply(void);
plant_block_reason_t plant_runtime_actuator_block_reason(void);
bool plant_runtime_servo_can_apply(void);
