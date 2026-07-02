#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "plant/can/can_frame.h"

/*
 * MCP2518FD + MCP2562FD SPI-CAN stack (schematic CH4 bench / rail0).
 *
 * Clock math (must match CubeMX):
 *   HSE 25 MHz → PLL ÷5 ×68 ÷2 = 170 MHz SYSCLK
 *   APB2 prescaler /2 → PCLK2 = 85 MHz (SPI1 kernel clock)
 *   SPI1 prescaler /8 → SCK ≈ 10.625 MHz (MCP2518 max 20 MHz)
 *
 * MCP2518 CAN bit clock (20 MHz crystal on OSC1/OSC2, PLLEN=0):
 *   f_bit = 20 MHz / ((BRP+1) × (1 + TSEG1 + TSEG2))
 *   1 Mbps = 20 MHz / (1 × 20 TQ) → BRP=0, TSEG1=17, TSEG2=2, SJW=2
 *   Sample point ≈ (1+17)/20 = 90%
 *
 * Operation: Classic CAN 2.0 only (BRSDIS=1, FDF=0, BRS=0), 29-bit extended IDs.
 */

#define MCP2518_RAIL_COUNT 3u
#define MCP2518_SYSCLK_HZ  20000000u
#define MCP2518_CAN_BAUD   1000000u

#define MCP2518_NBT_BRP_EXPECT   0u
#define MCP2518_NBT_TSEG1_EXPECT 17u
#define MCP2518_NBT_TSEG2_EXPECT 2u
#define MCP2518_NBT_SJW_EXPECT   2u

/* Init failure bits (mcp2518_init_diag_t.fail_bits). */
#define MCP_INIT_FAIL_SPI_RESET   (1u << 0)
#define MCP_INIT_FAIL_OSC_READY   (1u << 1)
#define MCP_INIT_FAIL_DEVID       (1u << 2)
#define MCP_INIT_FAIL_ENTER_CFG   (1u << 3)
#define MCP_INIT_FAIL_NBT_READBACK (1u << 4)
#define MCP_INIT_FAIL_TXQ_CFG     (1u << 5)
#define MCP_INIT_FAIL_ENTER_NORMAL (1u << 6)
#define MCP_INIT_FAIL_TXQ_SPACE   (1u << 7)
#define MCP_INIT_FAIL_EXT_LB      (1u << 8) /* optional self-test */

typedef struct {
	uint32_t fail_bits;
	uint8_t devid;
	uint8_t rev;
	uint8_t opmod;
	uint8_t ext_loopback_ok;
	uint8_t osc_b0;
	uint8_t osc_b1;
	uint8_t nbt_brp;
	uint8_t nbt_tseg1;
} mcp2518_init_diag_t;

typedef struct {
	uint8_t tx_ok;
	uint8_t tx_fail;
	uint8_t tx_nack;
	uint8_t rx_frames;
	uint8_t tec;
	uint8_t rec;
	uint8_t tec_before;
	uint8_t tx_fifo_sta;
	uint8_t tx_fifo_con;
	uint8_t c1con_b2;
	uint8_t osc_b0;
	uint8_t osc_b1;
	uint8_t nbt_brp;
	uint8_t nbt_tseg1;
	uint8_t bdiag1_b0;
	uint8_t bdiag1_b1;
	uint8_t init_fail_bits;
	uint8_t ext_loopback_ok;
	uint8_t devid;
} mcp2518_smoke_result_t;

bool mcp2518_init_all(void);
bool mcp2518_reinit_rail(can_bus_id_t bus);
uint8_t mcp2518_init_mask(void);
uint8_t mcp2518_rail_opmod(uint8_t rail);
void mcp2518_get_init_diag(uint8_t rail, mcp2518_init_diag_t *out);
void mcp2518_isr_rx_pending(uint8_t rail);
bool mcp2518_send(can_bus_id_t bus, const can_frame_t *frame);
bool mcp2518_recv(can_bus_id_t bus, can_frame_t *frame);
void mcp2518_drain_rx(can_bus_id_t bus);
void mcp2518_poll_rx(can_bus_id_t bus);
void mcp2518_get_tx_stats(uint8_t rail, uint8_t *tx_ok, uint8_t *tx_fail, uint8_t *tx_nack);
void mcp2518_reset_tx_stats(can_bus_id_t bus);
void mcp2518_prepare_tx(can_bus_id_t bus);
void mcp2518_rail_trec(uint8_t rail, uint8_t *tec, uint8_t *rec);
bool mcp2518_recover_bus(can_bus_id_t bus);
bool mcp2518_recover_if_busoff(can_bus_id_t bus);
void mcp2518_recover_all(void);
bool mcp2518_bus_smoke(can_bus_id_t bus, const can_frame_t *frame,
                       uint16_t listen_ms, mcp2518_smoke_result_t *out);
void mcp2518_refresh_smoke_diag(can_bus_id_t bus, mcp2518_smoke_result_t *out);

uint32_t mcp2518_pack_t0(uint32_t can_id, bool ext);
uint32_t mcp2518_unpack_t0(uint32_t t0, bool ext);
uint32_t mcp2518_pack_t1(uint8_t dlc, bool ext, bool remote);
bool mcp2518_id_roundtrip(uint32_t can_id);
