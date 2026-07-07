#pragma once
#include "host/host_exchange_schema.h"
#include <stdint.h>
#include <stdbool.h>

/* Bench PDU scan (layout v1 pdu.data[32], offset 530 in 562 B image). */
#define PLANT_DIAG_PDU_TAG0          'R'
#define PLANT_DIAG_PDU_TAG1          'S'
#define PLANT_DIAG_PDU_TAG2          '2'
#define PLANT_DIAG_PDU_RESP_TAG      'r'
#define PLANT_DIAG_DXL_TAG0          'D'
#define PLANT_DIAG_DXL_TAG1          'X'
#define PLANT_DIAG_DXL_TAG2          'L'

/* Damiao bench PDU (standard CAN MIT / position / speed probes). */
#define PLANT_DIAG_DM_TAG0           'D'
#define PLANT_DIAG_DM_TAG1           'M'
#define PLANT_DIAG_DM_TAG2           '0'
#define PLANT_DIAG_DM_RESP_TAG       'm'

#define PLANT_DIAG_DM_PDU_MASTER_ID  5u
#define PLANT_DIAG_DM_PDU_LISTEN_MS  6u
#define PLANT_DIAG_DM_PDU_PARAM_RID  7u
#define PLANT_DIAG_DM_PDU_END_ID     8u   /* DM_PROBE_ID_SWEEP only: range end (data[3]=start) */

/* actuator_feedback[slot].fault marker when DM probe results are mirrored */
#define PLANT_DM_FB_MAGIC            0xDA000000u
#define PLANT_DM_FB_TTL_PROBE        250u   /* ~500 ms @ 500 Hz USB feedback */
#define PLANT_DM_FB_TTL_SESSION      64u

/* Stamped into servo PDU bytes 29-30 when DM diag is compiled in. */
#define PLANT_DM_FW_MARKER0          'D'
#define PLANT_DM_FW_MARKER1          '1'

void plant_diag_feedback_stamp_fw_marker(host_pdu_feedback_t *pdu);

// DXL-Specific Plant Diag Params
#define PLANT_DXL_PROBE_SCAN         1u
#define PLANT_DXL_PROBE_PING         2u
#define PLANT_DXL_PROBE_FIND_BAUD    3u


#define PLANT_DIAG_PROBE_FULL        0u
#define PLANT_DIAG_PROBE_ENABLE_CTRL 1u
#define PLANT_DIAG_PROBE_CTRL_ONLY   2u
#define PLANT_DIAG_PROBE_PROMISC     10u
#define PLANT_DIAG_PROBE_RESET       11u
#define PLANT_DIAG_PROBE_ENABLE_ONLY 12u
#define PLANT_DIAG_PROBE_CTRL_FAST   13u
#define PLANT_DIAG_PROBE_PARAREAD    14u
#define PLANT_DIAG_PROBE_PROACTIVE   15u
#define PLANT_DIAG_PROBE_CALI        16u
#define PLANT_DIAG_PROBE_ZERO        17u
#define PLANT_DIAG_PROBE_DATA_SAVE   18u
#define PLANT_DIAG_PROBE_PARAWRITE   19u
#define PLANT_DIAG_PROBE_MCP_SMOKE    20u
#define PLANT_DIAG_PROBE_MCP_WAKE     21u
#define PLANT_DIAG_PROBE_MCP_DISABLE  22u
#define PLANT_DXL_PROBE_TOGGLE_BAUD  4u   /* 1M <-> 57600 on id_start..id_end */
#define PLANT_DXL_PROBE_SET_BAUD_1M  PLANT_DXL_PROBE_TOGGLE_BAUD
#define PLANT_DIAG_SESSION_BEGIN     254u
#define PLANT_DIAG_SESSION_END       255u

/* Host RS2 PDU: pdu.data[11] = schematic bus 1..6 (CH1–3 FDCAN, CH4–6 MCP2518). */
#define PLANT_DIAG_PDU_CAN_BUS       11u

#define PLANT_DIAG_RS2_QUIET_MS      3000u
#define PLANT_DIAG_DM_QUIET_MS       3000u

/* Drop bench session gates so plant teleop can drive actuators immediately. */
void plant_diag_release_actuator_can(void);

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
void plant_diag_feedback_sent(uint8_t probe_kind);
void plant_diag_feedback_fill(host_pdu_feedback_t *pdu);

