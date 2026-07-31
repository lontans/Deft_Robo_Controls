#pragma once
#include "host/host_exchange_schema.h"
#include "plant/diag/diag_gates.h"
#include "plant/plugins/robstride.h"
#include <stdbool.h>
#include <stdint.h>

/* Bench DEBUG RPC (discover / probe / lease) — not plant CMDH, not CFG.
 *
 * Tags live in image pdu.data[32]. Plant apply is gated via diag_gates.h
 * while a lease/probe/quiet is active. Bandwidth mode never sends these.
 */

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
/* SESSION_BEGIN bus_mask (bit0=CH1 … bit5=CH6). 0 → single primary bus.
 * RS2: data[5] (param_index unused on session). DM: data[9] (data[5]=master). */
#define PLANT_DIAG_RS2_PDU_BUS_MASK  5u
#define PLANT_DIAG_DM_PDU_BUS_MASK   9u
#define PLANT_DIAG_RS2_QUIET_MS      3000u
#define PLANT_DIAG_DM_QUIET_MS       3000u
/* RS2 DBGF: host bus 1..6 when multi-bus discover stamped a hit (was mcp init mask). */
#define PLANT_DIAG_RS2_PDU_RESP_BUS  27u

bool plant_diag_is_rs2_command(const host_command_image_t *cmd);
bool plant_diag_is_dxl_command(const host_command_image_t *cmd);
bool plant_diag_is_dm_command(const host_command_image_t *cmd);
void plant_diag_on_rs2_command(const host_command_image_t *cmd);
/* Back-compat alias — prefer plant_diag_on_rs2_command. */
#define plant_diag_on_command plant_diag_on_rs2_command
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
