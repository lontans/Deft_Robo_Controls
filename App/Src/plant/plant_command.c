#include "plant/plant_command.h"
#include "plant/actuator.h"
#include "plant/servo.h"
#include "plant/led.h"
#include "plant/plant_diag.h"
#include "plant/plant_config_nvm.h"
#include "plant/plugins/robstride.h"
#include "plant/thermo.h"
#include "plant/rx_sim/rx_sim.h"
#include "host/host_uart_bridge.h"
#include "host/soft_dfu.h"
#include "host/pdb_link.h"
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

	/* system.reserved bits0..3 (wire bits5..8): rx_sim children. */
	rx_sim_apply_from_reserved(cmd->system.reserved);

	/* Host ESTOP latches the PDB hard-ESTOP request; anything else clears
	 * it -- only an explicit ESTOP command should assert the wire from
	 * this path (link-loss fail-safe in pdb_link.c is independent). */
	pdb_link_request_estop(mcu_state == PLANT_MCU_STATE_ESTOP);

	if (mcu_state == PLANT_MCU_STATE_RECOVERY || mcu_state == PLANT_MCU_STATE_ESTOP) {
		plant_recovery_all();
		/* Soft-kill handshake: once parked, ack SOFT_KILL_READY to PDB.
		 * Only while peer still reports SOFT_KILL_REQ (docs/pdb-uart-v1.md). */
		if (pdb_link_kill_state() == (uint8_t)PDB_KILL_SOFT_REQ)
			pdb_link_set_soft_kill_ready(true);
		return;
	}

	if (pdb_link_kill_state() == (uint8_t)PDB_KILL_NORMAL)
		pdb_link_set_soft_kill_ready(false);

	if (mcu_state == PLANT_MCU_STATE_DIAG_ONLY)
		return;

	plant_diag_release_actuator_can();
	actuator_command_mount(cmd);
	/* Servo/LED: peripheral_command_mount() after host FB TX. */
}

void peripheral_command_mount(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return;

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

	pdb_link_request_estop(mcu_state == PLANT_MCU_STATE_ESTOP);

	if (mcu_state == PLANT_MCU_STATE_RECOVERY || mcu_state == PLANT_MCU_STATE_ESTOP) {
		plant_recovery_all();
		if (pdb_link_kill_state() == (uint8_t)PDB_KILL_SOFT_REQ)
			pdb_link_set_soft_kill_ready(true);
		return;
	}

	if (pdb_link_kill_state() == (uint8_t)PDB_KILL_NORMAL)
		pdb_link_set_soft_kill_ready(false);

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
	peripheral_command_mount(cmd);
}
