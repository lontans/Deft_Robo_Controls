#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "plant/can/can_frame.h"
#include "plant/can/can_router.h"

/*
 * Minimal CANopen master helpers (CiA 301) for ZeroErr eDriver bringup.
 * See docs/zeroerr-firmware-bringup.md.
 *
 * COB-IDs use the predefined connection set; N = node_id (1..127).
 * All frames are 11-bit standard IDs.
 */

#define CANOPEN_NMT_COB           0x000u
#define CANOPEN_TXPDO1_BASE       0x180u
#define CANOPEN_RXPDO1_BASE       0x200u
#define CANOPEN_TXPDO2_BASE       0x280u
#define CANOPEN_RXPDO2_BASE       0x300u
#define CANOPEN_SDO_TX_BASE       0x580u /* slave → master */
#define CANOPEN_SDO_RX_BASE       0x600u /* master → slave */

#define CANOPEN_NMT_START         0x01u
#define CANOPEN_NMT_STOP          0x02u
#define CANOPEN_NMT_PREOP         0x80u
#define CANOPEN_NMT_RESET_NODE    0x81u
#define CANOPEN_NMT_RESET_COMM    0x82u

#define CANOPEN_SDO_CMD_W1        0x2Fu
#define CANOPEN_SDO_CMD_W2        0x2Bu
#define CANOPEN_SDO_CMD_W4        0x23u
#define CANOPEN_SDO_CMD_R         0x40u
#define CANOPEN_SDO_RESP_R1       0x4Fu
#define CANOPEN_SDO_RESP_R2       0x4Bu
#define CANOPEN_SDO_RESP_R4       0x43u
#define CANOPEN_SDO_RESP_W        0x60u
#define CANOPEN_SDO_ABORT         0x80u

#define CANOPEN_PDO_DISABLE_BIT   0x80000000u

static inline uint16_t canopen_cob_txpdo1(uint8_t node)
{
	return (uint16_t)(CANOPEN_TXPDO1_BASE + (node & 0x7Fu));
}

static inline uint16_t canopen_cob_rxpdo1(uint8_t node)
{
	return (uint16_t)(CANOPEN_RXPDO1_BASE + (node & 0x7Fu));
}

static inline uint16_t canopen_cob_sdo_rx(uint8_t node)
{
	return (uint16_t)(CANOPEN_SDO_RX_BASE + (node & 0x7Fu));
}

static inline uint16_t canopen_cob_sdo_tx(uint8_t node)
{
	return (uint16_t)(CANOPEN_SDO_TX_BASE + (node & 0x7Fu));
}

void canopen_frame_std(can_frame_t *out, uint16_t cob_id, uint8_t dlc,
                       const uint8_t *data);

bool canopen_nmt_send(can_bus_id_t bus, uint8_t cs, uint8_t node_id);

/* Expedited SDO — polls RX until response or timeout_ms. Not for 500 Hz hot path.
 * No u16 variant: nothing in this codebase writes a 2-byte OD object yet
 * (mode is u8, profile vel/acc/dec + PDO comm params are u32) — add one
 * when a real caller needs it, CANOPEN_SDO_CMD_W2 (0x2B) is already defined. */
bool canopen_sdo_write_u8(can_bus_id_t bus, uint8_t node, uint16_t index,
                          uint8_t sub, uint8_t value, uint32_t timeout_ms);
bool canopen_sdo_write_u32(can_bus_id_t bus, uint8_t node, uint16_t index,
                           uint8_t sub, uint32_t value, uint32_t timeout_ms);
bool canopen_sdo_read_u32(can_bus_id_t bus, uint8_t node, uint16_t index,
                          uint8_t sub, uint32_t *out, uint32_t timeout_ms);

/* Remapped PDO1 (ZeroErr Python / bringup plan): cw u16 + pos i32, little-endian. */
void canopen_pack_rxpdo1_cw_pos(can_frame_t *out, uint8_t node,
                                uint16_t controlword, int32_t target_pos);
bool canopen_parse_txpdo1_sw_pos(const can_frame_t *in, uint8_t node,
                                 uint16_t *statusword, int32_t *actual_pos);
