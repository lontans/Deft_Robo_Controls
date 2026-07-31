#pragma once
#include "host/host_exchange_schema.h"
#include "plant/diag/diag_state.h"
#include "plant/can/can_frame.h"
#include <stdbool.h>
#include <stdint.h>

#define DIAG_PROBE_STUCK_MS    120000u
#define DIAG_HOST_STALE_MS      8000u

#define MCP_BENCH_LISTEN_MS_SMOKE 300u
#define MCP_BENCH_LISTEN_MS_WAKE  450u
#define MCP_BENCH_ZERO_GAIN_BURST 12u

can_bus_id_t diag_pdu_can_bus(const host_pdu_command_t *pdu);
uint8_t diag_pdu_bus_mask_at(const host_pdu_command_t *pdu,
                             uint8_t offset,
                             can_bus_id_t primary);
void diag_session_prepare_buses(uint8_t bus_mask, can_bus_id_t primary);
void diag_poll_session_buses(bool rx_only);
void diag_flush_usb(void);
void diag_finalize_probe(uint8_t kind, uint8_t motor_id, bool got);
void diag_run_rs2_pending(void);
void diag_stale_host_watchdog(void);

void diag_mcp_smoke_sync(uint8_t motor_id, can_bus_id_t bus);
void diag_mcp_wake_sync(uint8_t motor_id, can_bus_id_t bus);
void diag_mcp_disable_sync(uint8_t motor_id, can_bus_id_t bus);

void diag_dm_clear_actuator_mirror(void);
