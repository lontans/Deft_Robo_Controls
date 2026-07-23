#pragma once

/* UART4 (PC10/PC11) role select — pick one at build time and reflash.
 *
 * UART4_MODE_TELEM:         UART4 keeps its existing role as the alternate
 *                           host_exchange transport (host_transport_uart.c),
 *                           only live when HOST_TRANSPORT_UART is also set.
 * UART4_MODE_DAMIAO_BRIDGE: UART4 is reconfigured to 921600-8N1 and wired to
 *                           an external debug UART (e.g. Damiao motor debug
 *                           port). Raw bytes are tunnelled through the PDU
 *                           channel of the primary USB host_exchange link so
 *                           a host-side script can bridge them to a real
 *                           serial terminal / vendor tool.
 * UART4_MODE_PDB:           PC10/PC11 are wired to the Power Distribution
 *                           Board (see docs/pdb-uart-v1.md, ADR-001 in
 *                           docs/decisions.md). This is what this hardware
 *                           actually ships with — TELEM/DAMIAO_BRIDGE are
 *                           retained only for a board revision that doesn't
 *                           populate the PDB connector; on any board that
 *                           does, those two modes drive the wrong protocol
 *                           onto wires that go to the power board, not just
 *                           "PDB telemetry unavailable while selected."
 */
#define UART4_MODE_TELEM         0
#define UART4_MODE_DAMIAO_BRIDGE 1
#define UART4_MODE_PDB           2

#define UART4_MODE  UART4_MODE_PDB
