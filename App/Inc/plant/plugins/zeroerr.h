#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "plant/plugin_schema/plugin.h"
#include "plant/can/can_frame.h"
#include "plant/actuator.h"

/*
 * ZeroErr eDriver — CiA 402 Cyclic Synchronous Position (CSP) over CANopen.
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

/* Boot-time defaults for 0x6081/0x6083/0x6084 (units: counts/s, counts/s^2 —
 * no position-factor object configured, so these are raw encoder counts).
 * Placeholders (1 rev/s profile velocity, 2 rev/s^2 accel/decel) — previously
 * these objects were never written at all, leaving PP move dynamics at
 * whatever the drive's own NVM/factory default happened to be. Tune on bench. */
#define ZEROERR_DEFAULT_PROF_VEL ((uint32_t)ZEROERR_ENCODER_RES)
#define ZEROERR_DEFAULT_PROF_ACC ((uint32_t)ZEROERR_ENCODER_RES * 2u)
#define ZEROERR_DEFAULT_PROF_DEC ((uint32_t)ZEROERR_ENCODER_RES * 2u)

#define ZEROERR_CW_SHUTDOWN     0x0006u
#define ZEROERR_CW_SWITCH_ON    0x0007u
#define ZEROERR_CW_ENABLE       0x000Fu
/* PP-mode-only ("New Setpoint" edge, bit4) — unused now that we run CSP.
 * Bench-confirmed why: streaming a continuously-changing target under PP
 * kept re-asserting bit4 before the drive ack'd/cleared the prior one
 * (Change-Set-Immediately, bit5, was never implemented), which the drive
 * appears to treat as a fault -> repeated cold-restart/brake-click. Kept
 * defined (not deleted) as the record of that finding, not a live path. */
#define ZEROERR_CW_ENABLE_NEW   0x001Fu
#define ZEROERR_CW_FAULT_RESET  0x0080u

/* CiA402 statusword state decode (Table, DS402 §6.3): mask 0x006F (bits
 * 0,1,2,3,5,6) against the raw statusword to get one of these canonical
 * states. Bit3 alone (0x0008) is the Fault flag, checked unmasked. */
#define ZEROERR_SW_STATE_MASK               0x006Fu
#define ZEROERR_SW_STATE_NOT_READY          0x0000u
#define ZEROERR_SW_STATE_SWITCH_ON_DISABLED 0x0040u
#define ZEROERR_SW_STATE_READY_TO_SWITCH_ON 0x0021u
#define ZEROERR_SW_STATE_SWITCHED_ON        0x0023u
#define ZEROERR_SW_STATE_OPERATION_ENABLED  0x0027u
#define ZEROERR_SW_STATE_QUICK_STOP_ACTIVE  0x0007u
#define ZEROERR_SW_STATE_FAULT_REACTION     0x000Fu
#define ZEROERR_SW_STATE_FAULT              0x0008u
#define ZEROERR_SW_FAULT_BIT                0x0008u

/* ZEROERR_MODE_PP (Profile Position, value 1) is the vendor default and
 * still a valid mode for a future discrete point-to-point path, but nothing
 * in this firmware writes it anymore — CSP is the operating mode. */
#define ZEROERR_MODE_PP         1
#define ZEROERR_MODE_CSP        8

/* OD mapping entries (index|sub<<8|bits in low byte of EDS encoding 0xIIIISSLL). */
#define ZEROERR_MAP_CW          0x60400010u
#define ZEROERR_MAP_TARGET_POS  0x607A0020u
#define ZEROERR_MAP_SW          0x60410010u
#define ZEROERR_MAP_ACT_POS     0x60640020u

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
	/* Set only by zeroerr_read_position()/PLANT_DIAG_ZE_PROBE_POSITION —
	 * left zero for NODE/SWEEP/BOOT probes. */
	bool     position_valid;
	float    position_rad;
} zeroerr_probe_result_t;

bool zeroerr_probe_node(can_bus_id_t bus, uint8_t node_id, uint32_t sdo_timeout_ms,
                        zeroerr_probe_result_t *out);

/* SDO read of 0x6064 (Position Actual Value) — plain SDO upload, valid in
 * Pre-Operational or Operational, no PDO mapping/NMT-Operational/enable
 * required. Vendor manual §3.6.2 worked example confirms this exact byte
 * shape (COB-ID 0x600+node, `40 64 60 00 00 00 00 00`). Safe to call at any
 * point, including before zeroerr_boot_blocking. */
bool zeroerr_read_position(can_bus_id_t bus, uint8_t node_id, float *position_rad,
                           uint32_t sdo_timeout_ms);

/* Sweeps [start_id, end_id] (clamped to valid CANopen node ids 1..127),
 * stopping at the first hit — same "host re-sweeps from hit+1" contract as
 * the Damiao/CubeMars ID sweeps. */
bool zeroerr_probe_node_range(can_bus_id_t bus, uint8_t start_id, uint8_t end_id,
                              uint32_t sdo_timeout_ms, zeroerr_probe_result_t *out);
