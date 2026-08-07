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
	/* Not in either CubeMars PDF's §5.3 per-model table (neither doc
	 * mentions "AKH" at all) — V/T sourced from the official product page
	 * instead (cubemars.com AKH70-48 V1.0 KV41 spec sheet), which also
	 * confirms "SERVO and MIT control modes" support, i.e. MIT is not
	 * exclusive to the non-H AK line. See k_ak_limits[] in cubemars.c for
	 * the sourced numbers and the caveat that marketing-page values, like
	 * PDF sample values, should be bench-verified before trusting them for
	 * a hard clamp. */
	CUBEMARS_AKH70_48,
	CUBEMARS_AK_MODEL_COUNT,
} cubemars_ak_model_t;

/*
 * TODO(cfg-model): actuator_config_t has no per-slot AK model field. Default
 * every PROTO_CUBEMARS slot to CUBEMARS_MIT_DEFAULT_MODEL (same maturity as
 * damiao_limits() hardcoding DM4310), except a compiled-in Gen2 AKH70-48
 * slot seed (see k_gen2_akh70_48_slots[] in cubemars.c) — still not real
 * per-slot CFG, just a second hardcoded default layered on the first.
 * cubemars_set_model() is the hook for a future CFG/diag wire — do not
 * speculative-bump host CFG schema here.
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

/*
 * Bench discover/probe — separate from the hot apply_cycle path above (never
 * called from it; does not touch cubemars_apply_cycle's enable latch). No
 * PDF-documented 0x7FF-style register read for CubeMars (unlike Damiao), so
 * probing is MIT-frame-based only: send a zero-effort MIT command (or an
 * enable/disable/zero opcode) to a candidate Drive ID and listen for the FB
 * frame it echoes back (PDF: FB D[0] = Drive ID) — same "any valid frame
 * gets an FB reply" behavior the plant path already relies on.
 */
#define CUBEMARS_PROBE_MIT          0u   /* zero-effort MIT frame; listen for FB echo */
#define CUBEMARS_PROBE_PROMISC      10u  /* accept any Drive ID in the FB echo */
#define CUBEMARS_PROBE_ENABLE       11u  /* FF*7 + 0xFC */
#define CUBEMARS_PROBE_DISABLE      12u  /* FF*7 + 0xFD */
#define CUBEMARS_PROBE_SET_ZERO     13u  /* FF*7 + 0xFE */
#define CUBEMARS_PROBE_DISCOVER     15u  /* zero MIT frame, listen only (no enable) */
#define CUBEMARS_PROBE_ID_SWEEP     17u  /* zero MIT frame per id in [motor_id..end_id] */

typedef struct {
	bool     found;
	uint8_t  motor_id;
	uint8_t  discovered_id;
	uint8_t  probe_kind;
	uint8_t  fault;
	uint8_t  raw_frames_seen;
	uint8_t  tx_frames_sent;
	uint32_t rx_can_id;
	uint8_t  data[8];
	float    position;
	float    velocity;
	float    torque;
	float    temperature;
} cubemars_probe_result_t;

/* Single candidate: MIT/DISCOVER/PROMISC send a zero-effort MIT frame and
 * listen; ENABLE/DISABLE/SET_ZERO send the matching opcode frame and listen
 * for the FB echo it triggers. Never enqueues to actuator_table — bench only. */
bool cubemars_probe_id(can_bus_id_t bus, uint8_t motor_id, uint8_t probe_kind,
                       uint16_t listen_ms, cubemars_probe_result_t *out);

/* Sweeps [start_id, end_id] with one zero-effort MIT frame per candidate,
 * MCU-side so the host doesn't pay a USB round trip per id. Stops at the
 * first hit — same "host re-sweeps from hit+1" contract as Damiao ID_SWEEP. */
bool cubemars_probe_id_range(can_bus_id_t bus, uint8_t start_id, uint8_t end_id,
                             uint16_t listen_ms_per_id, cubemars_probe_result_t *out);

#if CUBEMARS_ENABLE_SERVO_MODE
/* Reference only — not on the hot plant path. */
#define CUBEMARS_POS_SCALE   10000.0f
#define CUBEMARS_SPEED_SCALE (1.0f / 10.0f)
#define CUBEMARS_ACCEL_SCALE (1.0f / 10.0f)
#endif
