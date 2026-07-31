#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Plant apply gates (TIM6 path) — separate from bench discover/probe PDU tags.
 *
 * Host mental model: discover → cfg → actions.
 * Firmware: plant_runtime_* answers "may actuator_apply_desire drive CAN?"
 * Bench leases / probes set skip flags; CFG is not a gate (see plant_command).
 */

/* Why 500 Hz actuator apply was skipped (stamped in feedback system.plant_block). */
typedef enum {
	PLANT_BLOCK_NONE = 0,
	PLANT_BLOCK_BENCH_SESSION = 1,
	PLANT_BLOCK_PROBE_BUSY = 2,
	PLANT_BLOCK_QUIET_PERIOD = 3,
	PLANT_BLOCK_APPLY_OFF = 4, /* plant_apply=0 (or legacy mcu_state=DIAG_ONLY) */
	PLANT_BLOCK_HOST_STALE = 5,
	PLANT_BLOCK_SERVO_SESSION = 6, /* reserved wire code; firmware does not set */
} plant_block_reason_t;

/* Back-compat alias — same code as PLANT_BLOCK_APPLY_OFF. */
#define PLANT_BLOCK_DIAG_ONLY PLANT_BLOCK_APPLY_OFF

bool plant_diag_bench_session_active(void);
bool plant_diag_probe_busy(void);
bool plant_diag_quiet_period_active(void);
bool plant_diag_skip_actuator_can(void);
bool plant_diag_skip_servo_bus(void);
void plant_diag_release_actuator_can(void);

bool plant_runtime_actuator_can_apply(void);
plant_block_reason_t plant_runtime_actuator_block_reason(void);
