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

static uint8_t g_mcu_state_readback;

uint8_t plant_command_mcu_state_readback(void)
{
	return g_mcu_state_readback;
}

void plant_command_image_dispatch_plant(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return;

	uint8_t mcu_state = (uint8_t)cmd->system.mcu_state;
	g_mcu_state_readback = mcu_state;

	if (mcu_state == PLANT_MCU_STATE_RECOVERY || mcu_state == PLANT_MCU_STATE_ESTOP) {
		plant_recovery_all();
		return;
	}

	if (mcu_state == PLANT_MCU_STATE_DIAG_ONLY)
		return;

	plant_diag_release_actuator_can();
	actuator_command_mount(cmd);
	servo_command_mount(cmd);
	led_command_mount(cmd);
}

void plant_command_image_dispatch_debug(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return;

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

	if (mcu_state == PLANT_MCU_STATE_RECOVERY || mcu_state == PLANT_MCU_STATE_ESTOP) {
		plant_recovery_all();
		return;
	}

	bool pdu_rs2 = plant_diag_is_rs2_command(cmd);
	bool pdu_dxl = plant_diag_is_dxl_command(cmd);
	bool pdu_dm  = plant_diag_is_dm_command(cmd);
	bool pdu_ub  = host_uart_bridge_is_command(cmd);
	bool pdu_tmp = thermo_is_command(cmd);
	bool diag_only = (mcu_state == PLANT_MCU_STATE_DIAG_ONLY);

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

	if (diag_only)
		return;

	if (pdu_dxl || pdu_dm || pdu_ub)
		return;

	if (pdu_rs2 && !rs02_probe_kind_mounts_desire(cmd->pdu.data[4]))
		return;

	plant_diag_release_actuator_can();
	actuator_command_mount(cmd);
	servo_command_mount(cmd);
	led_command_mount(cmd);
}
