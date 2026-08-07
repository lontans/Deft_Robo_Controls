#pragma once
#include "host/host_exchange_schema.h"
#include "plant/diag/diag_gates.h"
#include "plant/plugins/robstride.h"
#include "plant/plugins/cubemars.h"
#include "plant/plugins/zeroerr.h"
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
/* CubeMars bench PDU — Damiao-shaped tag/offset layout, no param_rid (no
 * PDF-documented register-read scheme for CubeMars). */
#define PLANT_DIAG_CM_TAG0           'C'
#define PLANT_DIAG_CM_TAG1           'M'
#define PLANT_DIAG_CM_TAG2           '0'
/* NOT 'c' — PLANT_CFG_PDU_RESP_TAG0 already owns that first byte; the resp
 * tag is the only byte plant_feedback.c uses to route/exclude, so this must
 * be globally unique across every protocol, not just unique among DM/RS2/CM. */
#define PLANT_DIAG_CM_RESP_TAG       'k'
#define PLANT_DIAG_CM_PDU_LISTEN_MS  6u
#define PLANT_DIAG_CM_PDU_END_ID     8u
#define PLANT_DIAG_CM_PDU_BUS_MASK   9u
#define PLANT_DIAG_CM_QUIET_MS       3000u
/* ZeroErr bench PDU — CANopen node sweep via SDO 0x1018, not MIT-shaped. */
#define PLANT_DIAG_ZE_TAG0           'Z'
#define PLANT_DIAG_ZE_TAG1           'E'
#define PLANT_DIAG_ZE_TAG2           '0'
#define PLANT_DIAG_ZE_RESP_TAG       'z'
#define PLANT_DIAG_ZE_PDU_TIMEOUT_MS 6u
#define PLANT_DIAG_ZE_PDU_END_ID     8u
#define PLANT_DIAG_ZE_PDU_BUS_MASK   9u
#define PLANT_DIAG_ZE_QUIET_MS       3000u
#define PLANT_DIAG_ZE_PROBE_NODE     0u  /* single node identity read */
/* Stage-1 bench probe: zeroerr_boot_blocking (NMT + PDO1 remap only, NO
 * controlword — brake stays engaged, node lands at most in Switch On
 * Disabled/Ready to Switch On). See App/Src/plant/plugins/zeroerr.c. */
#define PLANT_DIAG_ZE_PROBE_BOOT     1u
/* SDO read of 0x6064 (Position Actual Value) — no enable/PDO/NMT-Operational
 * required, safe at any time. See zeroerr_read_position(). */
#define PLANT_DIAG_ZE_PROBE_POSITION 2u
#define PLANT_DIAG_ZE_PROBE_SWEEP    17u /* sweep [motor_id..end_id], mirrors *_ID_SWEEP */
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
/* comm=0x07 set CAN_ID. param_index low byte = new CAN id (1..0x7F).
 * Motor is reset first (see robstride_probe_id), so caller need not. */
#define PLANT_DIAG_PROBE_SET_CAN_ID  RS02_PROBE_SET_CAN_ID
#define PLANT_DIAG_PROBE_MCP_SMOKE   20u
#define PLANT_DIAG_PROBE_MCP_WAKE    21u
#define PLANT_DIAG_PROBE_MCP_DISABLE 22u
#define PLANT_DXL_PROBE_TOGGLE_BAUD  4u
#define PLANT_DXL_PROBE_SET_BAUD_1M  PLANT_DXL_PROBE_TOGGLE_BAUD
/* target_id = current ID, id_start = new ID (id_end unused) — same param-
 * reuse pattern TOGGLE_BAUD already uses for id_start/id_end as a range.
 * Bring-up-only: two same-model DXLs share factory ID=1; reassign one
 * before both land on the same 2 Mbps domain (dxl_toggle_ids_baud would
 * otherwise create a real ID collision). */
#define PLANT_DXL_PROBE_SET_ID       5u
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
bool plant_diag_is_cm_command(const host_command_image_t *cmd);
bool plant_diag_is_ze_command(const host_command_image_t *cmd);
void plant_diag_on_rs2_command(const host_command_image_t *cmd);
/* Back-compat alias — prefer plant_diag_on_rs2_command. */
#define plant_diag_on_command plant_diag_on_rs2_command
void plant_diag_on_dxl_command(const host_command_image_t *cmd);
void plant_diag_on_dm_command(const host_command_image_t *cmd);
void plant_diag_on_cm_command(const host_command_image_t *cmd);
void plant_diag_on_ze_command(const host_command_image_t *cmd);
void plant_diag_service(void);
void plant_diag_can_router_poll(void);
void plant_diag_yield_usb(void);
bool plant_diag_blocks_usb_feedback(void);
void plant_diag_probe_progress(const robstride_probe_result_t *partial);
void plant_diag_feedback_sent(uint8_t probe_kind);
void plant_diag_feedback_fill(host_pdu_feedback_t *pdu);
void plant_diag_feedback_stamp_fw_marker(host_pdu_feedback_t *pdu);
