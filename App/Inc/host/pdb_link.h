#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Controls <-> PDB (Power Distribution Board) UART link.
 *
 * Separate from the USB host_exchange link (host_link.c/.h). Fixed 64 B
 * frames both directions over UART4 (UART4_MODE_PDB, see uart4_mode.h).
 * Wire contract: docs/pdb-uart-v1.md. Decision record: docs/decisions.md
 * ADR-001.
 *
 * Hard-ESTOP wire (PB7) is driven by the PDU, not this MCU. This module
 * configures PB7 as a high-Z input and reports the sensed level; it does
 * not drive the net. Soft-kill / freshness still gate kill_state over UART.
 */

#define PDB_FRAME_BYTES 64u

typedef enum {
	PDB_KILL_NORMAL      = 0,
	PDB_KILL_SOFT_REQ    = 1,
	PDB_KILL_SOFT_READY  = 2,
	PDB_KILL_HARD_ESTOP  = 3,
} pdb_kill_state_t;

typedef enum {
	PDB_KILL_REASON_NONE         = 0,
	PDB_KILL_REASON_HOST         = 1,
	PDB_KILL_REASON_UNDERVOLTAGE = 2,
	PDB_KILL_REASON_OVERCURRENT  = 3,
	PDB_KILL_REASON_OVERTEMP     = 4,
	PDB_KILL_REASON_COMMS_LOSS   = 5,
	PDB_KILL_REASON_BUTTON       = 6,
	PDB_KILL_REASON_OTHER        = 7,
} pdb_kill_reason_t;

void pdb_link_init(void);

/* Call once per app_run() -- non-blocking. Drains RX, validates/parses
 * feedback frames, and sends command frames at the documented rate. */
void pdb_link_service(void);

/* UART4 bring-up diagnostics (kernel clock + programmed BRR). */
uint8_t  pdb_link_uart_clk_src(void); /* 1=HSI, 2=PCLK1, 0=unknown */
uint16_t pdb_link_uart_brr(void);
uint32_t pdb_link_uart_kernel_hz(void);

/* Raw RX path health (independent of CRC/magic accept). */
uint32_t pdb_link_rx_byte_count(void);   /* bytes pushed from UART ISR */
uint32_t pdb_link_rx_event_count(void);  /* ReceiveToIdle callbacks */
uint32_t pdb_link_rx_valid_count(void);  /* frames that passed magic+CRC */
uint32_t pdb_link_uart_err_sticky(void); /* OR of HAL ErrorCode seen */
uint32_t pdb_link_rx_crc_fail_count(void);
uint32_t pdb_link_rx_last_word(void);    /* first 4 B of last 64 B attempt */
uint32_t pdb_link_tx_complete_count(void);
uint32_t pdb_link_uart_cr1(void);
uint32_t pdb_link_uart_cr2(void);
uint32_t pdb_link_uart_cr3(void);

/* -- Controls -> PDB (what we ask for) ------------------------------------ */

/* 1 bit/rail; controls *requests*, PDB is the authority on switching. */
void pdb_link_set_rail_enable_cmd(uint8_t mask);

/* Host/local E-STOP request latch (soft-kill path / kill_request field).
 * Does not drive PB7 — PDU owns the hard-ESTOP wire. */
void pdb_link_request_estop(bool assert);

/* Tell the PDB where we are in the soft-kill park sequence. Set READY only
 * once actuators are confirmed safe -- see plant_recovery_all() and the
 * staged sequence in docs/pdb-uart-v1.md. Do not set this speculatively. */
void pdb_link_set_soft_kill_ready(bool ready);

/* -- PDB -> Controls (what we've heard) ------------------------------------ */

bool    pdb_link_is_fresh(void);
uint8_t pdb_link_kill_state(void);   /* pdb_kill_state_t */
uint8_t pdb_link_kill_reason(void);  /* pdb_kill_reason_t */
uint8_t pdb_link_estop_sense(void);  /* local PB7 wire sense */
/* PDBF estop_sense field (PDU-reported). Stale → 1 (released); kill_state
 * already reports HARD_ESTOP on comms loss. */
uint8_t pdb_link_peer_estop_sense(void);

/* Copies the last valid 64 B PDB feedback frame verbatim (or zeros if none
 * has ever validated) into `out` -- feeds host-exchange `pdb[64]` on the
 * plant path. `out` must have room for PDB_FRAME_BYTES. */
void pdb_link_fill_mirror(uint8_t *out);
