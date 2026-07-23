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
 * This module owns the hard-ESTOP GPIO: it is the single place that decides
 * whether power is allowed, based on PDB link freshness plus any explicit
 * request from elsewhere in firmware (e.g. a host-commanded E-STOP).
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
 * feedback frames, sends command frames at the documented rate, and drives
 * the hard-ESTOP GPIO based on link freshness + any explicit request. */
void pdb_link_service(void);

/* -- Controls -> PDB (what we ask for) ------------------------------------ */

/* 1 bit/rail; controls *requests*, PDB is the authority on switching. */
void pdb_link_set_rail_enable_cmd(uint8_t mask);

/* Explicit E-STOP request from elsewhere in firmware (host command, local
 * fault, etc). Latches until cleared -- clear only via a fresh recovery
 * path, not automatically. */
void pdb_link_request_estop(bool assert);

/* Tell the PDB where we are in the soft-kill park sequence. Set READY only
 * once actuators are confirmed safe -- see plant_recovery_all() and the
 * staged sequence in docs/pdb-uart-v1.md. Do not set this speculatively. */
void pdb_link_set_soft_kill_ready(bool ready);

/* -- PDB -> Controls (what we've heard) ------------------------------------ */

bool    pdb_link_is_fresh(void);
uint8_t pdb_link_kill_state(void);   /* pdb_kill_state_t */
uint8_t pdb_link_kill_reason(void);  /* pdb_kill_reason_t */
uint8_t pdb_link_estop_sense(void);

/* Copies the last valid 64 B PDB feedback frame verbatim (or zeros if none
 * has ever validated) into `out` -- feeds host-exchange `pdb[64]` on the
 * plant path. `out` must have room for PDB_FRAME_BYTES. */
void pdb_link_fill_mirror(uint8_t *out);
