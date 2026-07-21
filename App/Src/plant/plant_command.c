#include "plant/plant_command.h"
#include "plant/actuator.h"
#include "plant/servo.h"
#include "plant/led.h"
#include "plant/plant_diag.h"
#include "plant/plant_config_nvm.h"
#include "plant/plugins/robstride.h"
#include "plant/thermo.h"
#include "host/host_uart_bridge.h"
#include "host/soft_dfu.h"
#include <stdbool.h>

/*
 * RS2 bench probes that intentionally drive comm=0x01 using actuator slot 0
 * (host may patch position/kp/kd into actuator_commands[0]).
 * All other probe kinds (cali, pararead, reset, session, …) must NOT mount
 * actuator desires — avoids kp=50 hold-to-zero fights during cal.
 */

static uint8_t g_mcu_state_readback;

uint8_t plant_command_mcu_state_readback(void)
{
	return g_mcu_state_readback;
}

void plant_command_image_dispatch(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return;

	/* Reboot-into-bootloader backdoor: checked first and unconditionally --
	 * if matched, this resets the board and never returns. See soft_dfu.h. */
	if (soft_dfu_is_command(cmd)) {
		soft_dfu_on_command(cmd);
		return;
	}

	uint8_t mcu_state = (uint8_t)cmd->system.mcu_state;
	g_mcu_state_readback = mcu_state;

	if (plant_config_is_command(cmd)) {
		plant_config_on_command(cmd);
		return;
	}

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
	bool pdu_dm  = plant_diag_is_dm_command(cmd);
	bool pdu_ub  = host_uart_bridge_is_command(cmd);
	bool pdu_tmp = thermo_is_command(cmd);
	bool diag_only = (mcu_state == PLANT_MCU_STATE_DIAG_ONLY);

	/* TMP is read-only telemetry — does not gate desire mounting. */
	if (pdu_tmp)
		thermo_on_command(cmd);

	if (pdu_ub)
		host_uart_bridge_on_command(cmd);
	else if (pdu_dxl)
		plant_diag_on_dxl_command(cmd);
	else if (pdu_dm)
		plant_diag_on_dm_command(cmd);
	else if (pdu_rs2)
		plant_diag_on_command(cmd);

	/* DIAG_ONLY: run PDU handler above, never touch actuator_commands[]. */
	if (diag_only)
		return;

	/*
	 * RS2 frame but not a ctrl probe (cali/pararead/reset/session/…):
	 * plant_diag handled it; do not mount desires onto the 500 Hz loop.
	 */
	if (pdu_dxl || pdu_dm || pdu_ub)
		return;

	if (pdu_rs2 && !rs02_probe_kind_mounts_desire(cmd->pdu.data[4]))
		return;

	/* Plant teleop / runtime: end DM bench session gates before mounting desires. */
	plant_diag_release_actuator_can();

	/* Normal path + RS2 ctrl probes: copy actuator_commands[0..ACTUATOR_COUNT-1]. */
	actuator_command_mount(cmd);
	servo_command_mount(cmd);
	led_command_mount(cmd);
}
