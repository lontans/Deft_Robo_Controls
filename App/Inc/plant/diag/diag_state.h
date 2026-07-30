#pragma once
/* Shared bench state — one translation unit (diag_state.c). */
#include "plant/actuator.h"
#include "plant/can/can_router.h"
#include "plant/can/mcp2518fd.h"
#include "plant/plugins/damiao.h"
#include "plant/plugins/robstride.h"
#include <stdbool.h>
#include <stdint.h>

extern robstride_probe_result_t g_last_probe;
extern mcp2518_smoke_result_t g_last_mcp_smoke;
extern volatile bool g_rs2_session_active;
extern volatile bool g_probe_in_progress;
extern volatile bool g_probe_progress_push;
extern uint32_t g_probe_started_ms;
extern uint8_t g_rs2_motor_id;
extern can_bus_id_t g_rs2_can_bus;
/* Bit i set ⇒ FW bus i (CH1+i) in the active RS2 bench session. 0 = legacy single. */
extern uint8_t g_rs2_bus_mask;
extern uint32_t g_rs2_quiet_until_ms;
extern bool g_dxl_feedback_active;
extern volatile bool g_dxl_probe_pending;
extern uint8_t g_dxl_pending_kind;
extern uint8_t g_dxl_pending_target_id;
extern uint8_t g_dxl_pending_id_start;
extern uint8_t g_dxl_pending_id_end;
extern volatile bool g_rs2_probe_pending;
extern uint8_t g_rs2_pending_kind;
extern uint8_t g_rs2_pending_motor_id;
extern can_bus_id_t g_rs2_pending_bus;
extern actuator_desire_t g_rs2_pending_desire;
extern bool g_rs2_pending_has_desire;
extern uint16_t g_rs2_pending_param_index;
extern uint32_t g_rs2_pending_param_raw;
extern uint8_t g_rs2_clear_after_send_kind;
extern damiao_probe_result_t g_last_dm_probe;
extern volatile bool g_dm_feedback_active;
extern uint8_t g_dm_pending_kind;
extern uint8_t g_dm_pending_motor_id;
extern uint8_t g_dm_pending_master_id;
extern uint8_t g_dm_pending_param_rid;
extern uint16_t g_dm_pending_listen_ms;
extern can_bus_id_t g_dm_pending_bus;
extern volatile bool g_dm_session_active;
extern can_bus_id_t g_dm_can_bus;
extern uint8_t g_dm_bus_mask;
extern uint8_t g_dm_feedback_ttl;
extern uint32_t g_dm_quiet_until_ms;
