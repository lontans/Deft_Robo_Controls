#pragma once
#include <stdint.h>

/* Provisional PDU V/I acceptability thresholds for Controls soft-kill overlay.
 *
 * Scales match the placeholder wire contract in docs/pdb-uart-v1.md:
 *   voltage  10 mV / count
 *   current  10 mA / count
 *
 * Keep these counts aligned with host SDK
 * scripts/deft_controls_sdk/pdb/limits.py (Claudius) when that module lands.
 * Tune after the first real PDB capture — not locked.
 *
 * Policy (three-agent PDU vbeta plan):
 *   - Evaluate fresh PDBF only (stale path stays HARD + COMMS_LOSS).
 *   - Overlay only when peer kill_state is NORMAL.
 *   - pack_v: 40–55 V for populated packs (count != 0).
 *   - rail_v[0] (central 48 V): 42–52 V when that rail looks active.
 *   - pack_i / rail_i: abs max 30 A on active channels.
 */

#define PDB_VI_PACK_V_MIN_COUNTS    4000u /* 40.00 V */
#define PDB_VI_PACK_V_MAX_COUNTS    5500u /* 55.00 V */

#define PDB_VI_RAIL48_V_MIN_COUNTS  4200u /* 42.00 V */
#define PDB_VI_RAIL48_V_MAX_COUNTS  5200u /* 52.00 V */

#define PDB_VI_I_ABS_MAX_COUNTS     3000u /* 30.00 A */
