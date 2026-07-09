#include "plant/diag/diag.h"
#include "plant/plant_command.h"
#include "plant/servo.h"
#include "host/host_link.h"
#include "plant/actuator.h"

#define ACTUATOR_HOST_STALE_MS 500u

static plant_block_reason_t s_last_block = PLANT_BLOCK_NONE;

plant_block_reason_t plant_runtime_actuator_block_reason(void)
{
	return s_last_block;
}

bool plant_runtime_servo_can_apply(void)
{
	return !plant_diag_skip_servo_bus();
}

bool plant_runtime_actuator_can_apply(void)
{
	if (plant_command_mcu_state_readback() == PLANT_MCU_STATE_DIAG_ONLY) {
		s_last_block = PLANT_BLOCK_DIAG_ONLY;
		return false;
	}

	if (plant_diag_skip_actuator_can()) {
		if (plant_diag_bench_session_active())
			s_last_block = PLANT_BLOCK_BENCH_SESSION;
		else if (plant_diag_probe_busy())
			s_last_block = PLANT_BLOCK_PROBE_BUSY;
		else if (plant_diag_quiet_period_active())
			s_last_block = PLANT_BLOCK_QUIET_PERIOD;
		else if (servo_host_session_active())
			s_last_block = PLANT_BLOCK_SERVO_SESSION;
		else
			s_last_block = PLANT_BLOCK_BENCH_SESSION;
		return false;
	}

	if (!host_link_command_is_fresh(ACTUATOR_HOST_STALE_MS) &&
	    !actuator_any_non_idle_live()) {
		s_last_block = PLANT_BLOCK_HOST_STALE;
		return false;
	}

	s_last_block = PLANT_BLOCK_NONE;
	return true;
}
