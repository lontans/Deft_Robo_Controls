#include "plant/plant_command.h"
#include "plant/actuator.h"
#include "plant/servo.h"
#include "plant/led.h"
#include "plant/plant_diag.h"
#include <stdbool.h>

/*
 * RS2 bench probes that intentionally drive comm=0x01 using actuator slot 0
 * (host may patch position/kp/kd into actuator_commands[0]).
 * All other probe kinds (cali, pararead, reset, session, …) must NOT mount
 * actuator desires — avoids kp=50 hold-to-zero fights during cal.
 */
static bool probe_kind_needs_actuator_mount(uint8_t kind)
{
	switch (kind) {
	case PLANT_DIAG_PROBE_FULL:
	case PLANT_DIAG_PROBE_ENABLE_CTRL:
	case PLANT_DIAG_PROBE_CTRL_ONLY:
	case PLANT_DIAG_PROBE_CTRL_FAST:
		return true;
	default:
		return false;
	}
}

static uint8_t g_mcu_state_readback;

uint8_t plant_command_mcu_state_readback(void)
{
	return g_mcu_state_readback;
}

void plant_command_image_dispatch(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return;

	uint8_t mcu_state = (uint8_t)cmd->system.mcu_state;
	g_mcu_state_readback = mcu_state;

	/* Host recovery / e-stop: reset configured motors and zero all desires. */
	if (mcu_state == PLANT_MCU_STATE_RECOVERY || mcu_state == PLANT_MCU_STATE_ESTOP) {
		plant_recovery_all();
		return;
	}

	/*
	 * pdu_rs2 == true when bytes 530–532 are 'R','S','2' (bench/cal/teleop backdoor).
	 * Normal plant teleop leaves pdu zero — pdu_rs2 stays false.
	 */
	bool pdu_rs2 = plant_diag_is_rs2_command(cmd);
	bool pdu_dxl = plant_diag_is_dxl_command(cmd);
	bool diag_only = (mcu_state == PLANT_MCU_STATE_DIAG_ONLY);

	if (pdu_dxl)
		plant_diag_on_dxl_command(cmd);
	else if (pdu_rs2)
		plant_diag_on_command(cmd);

	/* DIAG_ONLY: run PDU handler above, never touch actuator_commands[]. */
	if (diag_only)
		return;

	/*
	 * RS2 frame but not a ctrl probe (cali/pararead/reset/session/…):
	 * plant_diag handled it; do not mount desires onto the 500 Hz loop.
	 */
	if (pdu_dxl)
		return;

	if (pdu_rs2 && !probe_kind_needs_actuator_mount(cmd->pdu.data[4]))
		return;

	/* Normal path + RS2 ctrl probes: copy actuator_commands[0..ACTUATOR_COUNT-1]. */
	actuator_command_mount(cmd);
	servo_command_mount(cmd);
	led_command_mount(cmd);
}
