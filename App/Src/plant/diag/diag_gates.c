#include "plant/diag/diag_gates.h"
#include "plant/diag/diag.h"
#include "plant/plant_command.h"
#include "host/host_link.h"
#include "plant/actuator.h"

#define ACTUATOR_HOST_STALE_MS 500u
/* Non-idle-live actuators get a grace period past ACTUATOR_HOST_STALE_MS
 * (avoid yanking torque/brake out from under a loaded joint on a momentary
 * host hiccup) — but only up to this hard ceiling. Previously there was no
 * ceiling at all: an actuator armed non-idle (kp/kd>0) stayed exempt from
 * the stale-host block forever, since nothing ever clears actuator_desire_live
 * once the host that set it disappears (process exit, USB unplug, crash).
 * Confirmed on bench: closing the host REPL left a ZeroErr actuator still
 * being driven/probed on CH1 indefinitely. */
#define ACTUATOR_HOST_STALE_HARD_MS 3000u

static plant_block_reason_t s_last_block = PLANT_BLOCK_NONE;

plant_block_reason_t plant_runtime_actuator_block_reason(void)
{
	return s_last_block;
}

bool plant_runtime_actuator_can_apply(void)
{
	if (!plant_command_plant_apply_readback()) {
		s_last_block = PLANT_BLOCK_APPLY_OFF;
		return false;
	}

	if (plant_diag_skip_actuator_can()) {
		if (plant_diag_bench_session_active())
			s_last_block = PLANT_BLOCK_BENCH_SESSION;
		else if (plant_diag_probe_busy())
			s_last_block = PLANT_BLOCK_PROBE_BUSY;
		else if (plant_diag_quiet_period_active())
			s_last_block = PLANT_BLOCK_QUIET_PERIOD;
		else
			s_last_block = PLANT_BLOCK_BENCH_SESSION;
		return false;
	}

	if (!host_link_command_is_fresh(ACTUATOR_HOST_STALE_MS) &&
	    (!actuator_any_non_idle_live() ||
	     !host_link_command_is_fresh(ACTUATOR_HOST_STALE_HARD_MS))) {
		s_last_block = PLANT_BLOCK_HOST_STALE;
		return false;
	}

	s_last_block = PLANT_BLOCK_NONE;
	return true;
}
