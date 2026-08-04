#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "plant/plugin_schema/plugin.h"
#include "plant/can/can_frame.h"
#include "plant/actuator.h"

/*
 * ZeroErr eDriver — CiA 402 Profile Position over CANopen.
 * Frame formats + boot sequence: docs/zeroerr-firmware-bringup.md
 * EDS: External_Documentation/ZeroErr/ZeroErr_Driver_V1.5.eds
 *
 * motor_id in actuator_config_t = CANopen node ID (1..127).
 * Bus: prefer FDCAN CH1–3 @ 1 Mbps (EDS BaudRate_1000 only).
 */

#define ZEROERR_VENDOR_ID       0x5A65726Fu
#define ZEROERR_PRODUCT_CODE    0x26483052u
#define ZEROERR_ENCODER_RES     524288     /* counts/rev — confirm on hardware */

#define ZEROERR_IDX_CONTROLWORD 0x6040u
#define ZEROERR_IDX_STATUSWORD  0x6041u
#define ZEROERR_IDX_MODE        0x6060u
#define ZEROERR_IDX_MODE_DISP   0x6061u
#define ZEROERR_IDX_POS_ACT     0x6064u
#define ZEROERR_IDX_POS_TARGET  0x607Au
#define ZEROERR_IDX_PROF_VEL    0x6081u
#define ZEROERR_IDX_PROF_ACC    0x6083u
#define ZEROERR_IDX_PROF_DEC    0x6084u

#define ZEROERR_CW_SHUTDOWN     0x0006u
#define ZEROERR_CW_SWITCH_ON    0x0007u
#define ZEROERR_CW_ENABLE       0x000Fu
#define ZEROERR_CW_ENABLE_NEW   0x001Fu
#define ZEROERR_CW_FAULT_RESET  0x0080u

#define ZEROERR_SW_TARGET_REACHED 0x0400u

#define ZEROERR_MODE_PP         1

/* OD mapping entries (index|sub<<8|bits in low byte of EDS encoding 0xIIIISSLL). */
#define ZEROERR_MAP_CW          0x60400010u
#define ZEROERR_MAP_TARGET_POS  0x607A0020u
#define ZEROERR_MAP_SW          0x60410010u
#define ZEROERR_MAP_ACT_POS     0x60640020u

extern const plugin_ops_t zeroerr_ops;

int32_t zeroerr_rad_to_counts(float rad);
float   zeroerr_counts_to_rad(int32_t counts);

/* Plant path: boot FSM + RxPDO1 when operational. */
void zeroerr_apply_cycle(const actuator_config_t *cfg,
                         const actuator_desire_t *desire,
                         actuator_state_t *state_out);

/* RX demux for TxPDO1 (and optional SDO — ignored on plant once booted). */
void zeroerr_on_rx_frame(const actuator_config_t *cfg, uint8_t slot,
                         const can_frame_t *frame, actuator_state_t *state_out);

/* Disable / clear boot state (recovery). */
void zeroerr_reset_slot(uint8_t slot);
bool zeroerr_send_shutdown(const actuator_config_t *cfg, can_frame_t *frame_out);

/* Blocking boot for DEBUG/bench — installs PDO1 map + NMT start. */
bool zeroerr_boot_blocking(can_bus_id_t bus, uint8_t node_id, uint32_t sdo_timeout_ms);

/* SDO identity check (0x1018). */
bool zeroerr_read_identity(can_bus_id_t bus, uint8_t node_id,
                           uint32_t *vendor, uint32_t *product, uint32_t *revision,
                           uint32_t sdo_timeout_ms);

/* Bench discover/probe — never called from zeroerr_apply_cycle. Sweeps
 * CANopen node IDs via SDO-read 0x1018 (Identity Object); any node that
 * answers is reported, with vendor_match flagging a confirmed ZeroErr
 * eDriver (vendor/product match) vs. some other CANopen node replying. */
typedef struct {
	bool     found;
	bool     vendor_match;
	uint8_t  node_id;
	uint8_t  discovered_id;
	uint32_t vendor;
	uint32_t product;
	uint32_t revision;
} zeroerr_probe_result_t;

bool zeroerr_probe_node(can_bus_id_t bus, uint8_t node_id, uint32_t sdo_timeout_ms,
                        zeroerr_probe_result_t *out);

/* Sweeps [start_id, end_id] (clamped to valid CANopen node ids 1..127),
 * stopping at the first hit — same "host re-sweeps from hit+1" contract as
 * the Damiao/CubeMars ID sweeps. */
bool zeroerr_probe_node_range(can_bus_id_t bus, uint8_t start_id, uint8_t end_id,
                              uint32_t sdo_timeout_ms, zeroerr_probe_result_t *out);
