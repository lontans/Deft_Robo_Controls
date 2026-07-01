#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "plant/can/can_frame.h"

#define MCP2518_RAIL_COUNT 3u
#define MCP2518_SYSCLK_HZ  20000000u  /* 20 MHz crystal on OSC1/OSC2 (schematic X3) */
#define MCP2518_CAN_BAUD   1000000u   /* Classic CAN 2.0 @ 1 Mbit/s */

typedef struct {
	uint8_t tx_ok;
	uint8_t tx_fail;
	uint8_t rx_frames;
	uint8_t tec;
	uint8_t rec;
	uint8_t tx_fifo_sta;
	uint8_t tx_fifo_con;
	uint8_t c1con_b2;   /* byte2: bit4 = TXQEN */
	uint8_t osc_b0;     /* bit0 = PLLEN (must be 0 for 20 MHz XTAL) */
	uint8_t osc_b1;     /* bit2 = OscReady */
	uint8_t nbt_tseg1;  /* CiNBTCFG TSEG1 readback (expect 17) */
	uint8_t bdiag1_b0;  /* CiBDIAG1 low byte: NACK/form/bit errs */
} mcp2518_smoke_result_t;

bool mcp2518_init_all(void);
uint8_t mcp2518_init_mask(void);
uint8_t mcp2518_rail_opmod(uint8_t rail);
void mcp2518_isr_rx_pending(uint8_t rail);
bool mcp2518_send(can_bus_id_t bus, const can_frame_t *frame);
bool mcp2518_recv(can_bus_id_t bus, can_frame_t *frame);
void mcp2518_drain_rx(can_bus_id_t bus);
void mcp2518_get_tx_stats(uint8_t rail, uint8_t *tx_ok, uint8_t *tx_fail);
void mcp2518_rail_trec(uint8_t rail, uint8_t *tec, uint8_t *rec);
bool mcp2518_recover_bus(can_bus_id_t bus);
void mcp2518_recover_all(void);
bool mcp2518_bus_smoke(can_bus_id_t bus, const can_frame_t *frame,
                       uint16_t listen_ms, mcp2518_smoke_result_t *out);
