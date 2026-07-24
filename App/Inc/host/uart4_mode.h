#pragma once

/* UART4 (PC10/PC11) role select — pick one at build time and reflash.
 *
 * UART4_MODE_TELEM:         alternate host_exchange transport
 * UART4_MODE_DAMIAO_BRIDGE: 921600 raw bridge via USB PDU mailbox
 * UART4_MODE_PDB:           Power Distribution Board (docs/pdb-uart-v1.md)
 */
#define UART4_MODE_TELEM         0
#define UART4_MODE_DAMIAO_BRIDGE 1
#define UART4_MODE_PDB           2

#define UART4_MODE  UART4_MODE_PDB

/* Temporary: overlay RX/TX health on system feedback for CDC prove. */
#define UART4_BRINGUP_DIAG  1

/* When 1: after init, send one PDBC and expect it back on RX.
 * Requires a temporary short of UART4 TX↔RX at the Controls JST (PC10↔PC11).
 * Pass → last_rx shows 'PDBC', valid>=1. Isolates MCU stack from Jetson. */
#define UART4_LOCAL_LOOPBACK_TEST  0

/* Pace PDB TX one byte per HostTask tick (~1 ms). tegra194-hsuart on the
 * Jetson corrupts bulk 64 B bursts (RX all 0x00); byte gaps fix decode.
 * Frame takes ~64 ms → effective PDBC rate ~10–15 Hz. */
#define UART4_TX_PACE_BYTES  1
