#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "plant/plugin_schema/plugin_types.h"
#include "plant/can/can_frame.h"
#include "plant/actuator.h"

/*
 * CubeMars AK-series — MIT Power Mode (PDF §5.3) is the only hot plant path.
 *
 * Spec: External_Documentation/CubeMars/cubemars_motor_driver_doc.pdf §5.3
 * Derivation / PDF sample bugs: docs/rfc-cubemars-mit-plant.md
 *
 * Lifecycle (per slot) — Damiao-shaped, TX-driven latch (not RX-gated):
 *   RESET  → cubemars_reset_enable_latch() / plant_recovery_all()
 *   ENTER  → first cubemars_apply_cycle(): TX {FF×7, 0xFC}, latch=true
 *   STREAM → every apply_cycle tick: one MIT DLC=8 (idle kp=kd=0 included)
 *   EXIT   → plant_recovery_all() only: latch reset + TX {FF×7, 0xFD}
 *
 * apply_cycle never sends disable. Blank desire is a valid zero-effort MIT
 * command once entered — do not leave MIT mode on command gaps.
 * CubeMars PDF documents no MIT fault bit for RX-gated enable (unlike Damiao
 * ERR==1); inventing one is out of scope. No 0xFB clear-fault opcode.
 *
 * Host wire surface: ActuatorDesire(position, velocity, kp, kd, torque) —
 * same 22 B MIT slot as Damiao. Servo Mode (ext-ID mode 6) stays behind
 * CUBEMARS_ENABLE_SERVO_MODE (default 0) — reference only.
 */
#define CUBEMARS_ENABLE_SERVO_MODE 0

typedef enum {
	CUBEMARS_MODE_DUTY          = 0u,
	CUBEMARS_MODE_CURRENT       = 1u,
	CUBEMARS_MODE_CURRENT_BRAKE = 2u,
	CUBEMARS_MODE_SPEED         = 3u,
	CUBEMARS_MODE_POSITION      = 4u,
	CUBEMARS_MODE_SET_ORIGIN    = 5u,
	CUBEMARS_MODE_POS_SPEED     = 6u,
} cubemars_control_mode_t;

/* --------------------------------------------------------------------- */
/* MIT Power Mode (PDF §5.3)                                              */
/* --------------------------------------------------------------------- */

/* Special Can codes — FF*7 + opcode, std ID = motor ID. Byte-identical to
 * Damiao opcodes for enable/disable/zero. CubeMars has no documented 0xFB. */
#define CUBEMARS_MIT_CMD_ENABLE   0xFCu
#define CUBEMARS_MIT_CMD_DISABLE  0xFDu
#define CUBEMARS_MIT_CMD_SET_ZERO 0xFEu

/* Shared P/Kp/Kd from §5.3 table — NOT the PDF pack_cmd sample (±95.5). */
#define CUBEMARS_MIT_P_MIN  (-12.5f)
#define CUBEMARS_MIT_P_MAX  ( 12.5f)
#define CUBEMARS_MIT_KP_MIN (0.0f)
#define CUBEMARS_MIT_KP_MAX (500.0f)
#define CUBEMARS_MIT_KD_MIN (0.0f)
#define CUBEMARS_MIT_KD_MAX (5.0f)

typedef enum {
	CUBEMARS_AK10_9 = 0,
	CUBEMARS_AK60_6,
	CUBEMARS_AK70_10,
	CUBEMARS_AK80_6,
	CUBEMARS_AK80_9,
	CUBEMARS_AK80_80,
	CUBEMARS_AK_MODEL_COUNT,
} cubemars_ak_model_t;

/*
 * TODO(cfg-model): actuator_config_t has no per-slot AK model field. Default
 * every PROTO_CUBEMARS slot to CUBEMARS_MIT_DEFAULT_MODEL (same maturity as
 * damiao_limits() hardcoding DM4310). cubemars_set_model() is the hook for a
 * future CFG/diag wire — do not speculative-bump host CFG schema here.
 */
#define CUBEMARS_MIT_DEFAULT_MODEL CUBEMARS_AK80_9

void cubemars_set_model(uint8_t slot, cubemars_ak_model_t model);
cubemars_ak_model_t cubemars_get_model(uint8_t slot);

/* PDF: RX Identifier = 0x00 + Drive ID. master_id 0 / 0xFFFFFFFF → motor_id. */
uint32_t cubemars_resolve_rx_can_id(const actuator_config_t *cfg);

uint32_t cubemars_build_ext_id(cubemars_control_mode_t mode, uint8_t node_id);
bool     cubemars_parse_ext_id(uint32_t ext_id, cubemars_control_mode_t *mode, uint8_t *node_id);

plugin_status_t cubemars_send_enable(const actuator_config_t *cfg, can_frame_t *frame_out);
plugin_status_t cubemars_send_disable(const actuator_config_t *cfg, can_frame_t *frame_out);
plugin_status_t cubemars_send_set_zero(const actuator_config_t *cfg, can_frame_t *frame_out);

void cubemars_reset_enable_latch(uint8_t slot);
bool cubemars_is_enable_latched(uint8_t slot);

void cubemars_apply_cycle(const actuator_config_t *cfg,
                          const actuator_desire_t *desire,
                          actuator_state_t *state_out);
void cubemars_on_rx_frame(const actuator_config_t *cfg, uint8_t slot,
                         const can_frame_t *frame, actuator_state_t *state_out);

#if CUBEMARS_ENABLE_SERVO_MODE
/* Reference only — not on the hot plant path. */
#define CUBEMARS_POS_SCALE   10000.0f
#define CUBEMARS_SPEED_SCALE (1.0f / 10.0f)
#define CUBEMARS_ACCEL_SCALE (1.0f / 10.0f)
#endif
