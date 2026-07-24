#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "plant/plugin_schema/plugin_types.h"
#include "plant/can/can_frame.h"
#include "plant/actuator.h"

/*
 * CubeMars AK-series driver board, MIT Power Mode (default hot plant path).
 * Source: External_Documentation/CubeMars/cubemars_motor_driver_doc.pdf §5.3
 * "MIT power mode communication protocol". See docs/rfc-cubemars-mit-plant.md
 * for the full derivation, the vendor PDF's own sample-code bugs (do not
 * port CUBEMARS_MIT_P_MIN/MAX etc. from the PDF's `pack_cmd()` sample —
 * those contradict the PDF's own per-module table two pages later), and why
 * this mirrors damiao.c's apply_cycle/enable-latch shape rather than the
 * stateless single-frame plugin_pack_tx()/plugin_parse_rx() path Servo Mode
 * used.
 *
 * Servo Mode (Position-Speed Loop, control mode 6, extended 29-bit ID) is
 * kept behind CUBEMARS_ENABLE_SERVO_MODE (default off) for reference / a
 * future bench diagnostic path — it is no longer what PROTO_CUBEMARS means
 * on the hot apply/RX path. See CubeMars_AK_Driver_Doc_Generalised.pdf
 * §5.1-5.2 for that mode; nothing in this header changes its behavior.
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

/* "Special Can code" — FF*7 + opcode, std ID = motor ID. Byte-identical to
 * Damiao's damiao_pack_cmd() opcodes; CubeMars documents no separate
 * clear-fault opcode (unlike Damiao's 0xFB) — do not invent one. */
#define CUBEMARS_MIT_CMD_ENABLE   0xFCu
#define CUBEMARS_MIT_CMD_DISABLE  0xFDu
#define CUBEMARS_MIT_CMD_SET_ZERO 0xFEu

/* Shared across every AK module per the §5.3 table (p.44) — do NOT use the
 * PDF's own `pack_cmd()` sample constants (P_MIN=-95.5/P_MAX=95.5), which
 * contradict this table and appear to be copy-pasted from an unrelated
 * example. See docs/rfc-cubemars-mit-plant.md "PDF sample bugs". */
#define CUBEMARS_MIT_P_MIN  (-12.5f)
#define CUBEMARS_MIT_P_MAX  ( 12.5f)
#define CUBEMARS_MIT_KP_MIN (0.0f)
#define CUBEMARS_MIT_KP_MAX (500.0f)
#define CUBEMARS_MIT_KD_MIN (0.0f)
#define CUBEMARS_MIT_KD_MAX (5.0f)

/* Per-module velocity/torque span (§5.3 p.44 table) — position/Kp/Kd above
 * are shared across all six; only these two vary per AK part number. */
typedef enum {
	CUBEMARS_AK10_9 = 0,
	CUBEMARS_AK60_6,
	CUBEMARS_AK70_10,
	CUBEMARS_AK80_6,
	CUBEMARS_AK80_9,
	CUBEMARS_AK80_80,
	CUBEMARS_AK_MODEL_COUNT,
} cubemars_ak_model_t;

/* No per-slot model selection is wired to CFG in this patch (would need a
 * new CFG SET field or to double-purpose master_id, and actuator_config_t
 * has no other spare slot). Matches Damiao's own current maturity level —
 * damiao_limits() hardcodes DM4310 for every slot too, ignoring cfg
 * entirely. cubemars_set_model() exists so a future CFG/diag hook can call
 * it; nothing calls it yet. Do not build that wiring speculatively. */
#define CUBEMARS_MIT_DEFAULT_MODEL CUBEMARS_AK80_9

void cubemars_set_model(uint8_t slot, cubemars_ak_model_t model);

uint32_t cubemars_build_ext_id(cubemars_control_mode_t mode, uint8_t node_id);
bool     cubemars_parse_ext_id(uint32_t ext_id, cubemars_control_mode_t *mode, uint8_t *node_id);

plugin_status_t cubemars_send_enable(const actuator_config_t *cfg, can_frame_t *frame_out);
plugin_status_t cubemars_send_disable(const actuator_config_t *cfg, can_frame_t *frame_out);
plugin_status_t cubemars_send_set_zero(const actuator_config_t *cfg, can_frame_t *frame_out);

void cubemars_reset_enable_latch(uint8_t slot);
void cubemars_apply_cycle(const actuator_config_t *cfg,
                          const actuator_desire_t *desire,
                          actuator_state_t *state_out);
void cubemars_on_rx_frame(const actuator_config_t *cfg, uint8_t slot,
                         const can_frame_t *frame, actuator_state_t *state_out);

/* cubemars_ops (plugin_ops_t) is declared in plugin_table.c, not here —
 * plugin_ops_t isn't visible from plugin_types.h alone, same convention as
 * robstride.h/damiao.h. PROTO_CUBEMARS is already registered there; this
 * patch only changes what cubemars_ops's pack_tx/parse_rx do (MIT instead
 * of Servo) plus the new apply_cycle/on_rx_frame/recovery entry points
 * actuator.c calls directly, mirroring PROTO_DAMIAO. */

#if CUBEMARS_ENABLE_SERVO_MODE
/* TODO(hardware, P0 — do not power a motor against these unverified):
 * The vendor doc's Servo-mode Position/Speed sections mix "electrical
 * degrees"/"electrical RPM" language inconsistently. Converting
 * ActuatorDesire's mechanical rad / rad*s^-1 to whatever this protocol
 * actually expects may require the motor's pole-pair count, unknown until
 * hardware is in hand. Kept for reference only — not on the hot path. */
#define CUBEMARS_POS_SCALE   10000.0f   /* documented: deg * 10000 -> int32 LSB */
#define CUBEMARS_SPEED_SCALE (1.0f / 10.0f)  /* documented: eRPM / 10 -> int16 LSB */
#define CUBEMARS_ACCEL_SCALE (1.0f / 10.0f)  /* documented: (eRPM/s^2) / 10 -> int16 LSB */
#endif
