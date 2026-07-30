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
#include <string.h>

static uint8_t g_mcu_state_readback;
static uint8_t g_stm32_mode_readback;
static bool g_plant_apply_readback;
static bool g_debug_lanes_pending;
static uint16_t g_debug_lanes_arm_mask;

uint8_t plant_command_mcu_state_readback(void)
{
	return g_mcu_state_readback;
}

bool plant_command_plant_apply_readback(void)
{
	return g_plant_apply_readback;
}

uint8_t plant_command_stm32_mode_readback(void)
{
	return g_stm32_mode_readback;
}

static bool plant_command_extract_plant_apply(const host_command_image_t *cmd,
                                               uint8_t mcu_state)
{
	bool apply = ((cmd->system.reserved >> HOST_PLANT_APPLY_SHIFT) &
	              HOST_PLANT_APPLY_MASK) != 0u;
	/* Legacy hosts still send mcu_state=DIAG_ONLY for observe. */
	if (mcu_state == PLANT_MCU_STATE_DIAG_ONLY)
		apply = false;
	return apply;
}

bool plant_command_debug_lanes_pending(void)
{
	return g_debug_lanes_pending;
}

void plant_command_debug_lanes_clear_pending(void)
{
	g_debug_lanes_pending = false;
	g_debug_lanes_arm_mask = 0u;
}

uint16_t plant_command_debug_lanes_arm_mask(void)
{
	return g_debug_lanes_arm_mask;
}

static uint8_t plant_command_extract_stm32_mode(const host_command_image_t *cmd)
{
	return (uint8_t)((cmd->system.reserved >> HOST_STM32_MODE_SHIFT) &
	                 HOST_STM32_MODE_MASK);
}

void plant_command_observe_stm32_mode(const host_command_image_t *cmd)
{
	/* Echo tracks the latest USB-RX'd plant CMDH immediately so FB does
	 * not keep a previous session's debug/soft_dfu sticky across reconnect
	 * until TIM6 apply. Soft-DFU enter stays on plant apply. */
	if (cmd == NULL)
		return;
	g_stm32_mode_readback = plant_command_extract_stm32_mode(cmd);
}

static void plant_command_note_stm32_mode(const host_command_image_t *cmd)
{
	/* Soft-DFU via stm32_mode=2 is plant-CMDH only. Debug-lanes frames reuse
	 * the system-word bytes for the DL header — decoding those as mode 2 would
	 * falsely enter ROM DFU on every CFG/discover (ADR-004). */
	plant_command_observe_stm32_mode(cmd);
	if (g_stm32_mode_readback == HOST_STM32_MODE_SOFT_DFU)
		soft_dfu_on_command(cmd);
}

static bool host_debug_lanes_present(const host_command_image_t *cmd)
{
	const uint8_t *raw = (const uint8_t *)cmd;
	const host_debug_lanes_header_t *hdr;

	if (cmd == NULL)
		return false;
	hdr = (const host_debug_lanes_header_t *)(raw + HOST_DEBUG_LANES_HDR_OFF);
	return hdr->tag0 == (uint8_t)HOST_DEBUG_LANES_TAG0 &&
	       hdr->tag1 == (uint8_t)HOST_DEBUG_LANES_TAG1 &&
	       hdr->ver == (uint8_t)HOST_DEBUG_LANES_VER;
}

static const uint8_t *host_debug_lane_bytes(const host_command_image_t *cmd,
                                              uint8_t lane)
{
	const uint8_t *raw = (const uint8_t *)cmd;

	return raw + HOST_DEBUG_LANE0_OFF +
	       ((size_t)lane * (size_t)HOST_DEBUG_LANE_BYTES);
}

static void plant_command_dispatch_debug_legacy(const host_command_image_t *cmd)
{
	if (soft_dfu_is_command(cmd)) {
		soft_dfu_on_command(cmd);
		return;
	}

	uint8_t mcu_state = (uint8_t)cmd->system.mcu_state;
	g_mcu_state_readback = mcu_state;
	g_plant_apply_readback = plant_command_extract_plant_apply(cmd, mcu_state);

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

	/* Observe (plant_apply=0): handle RPC tags but do not mount plant or
	 * tear down an active bench lease — same role legacy DIAG_ONLY had. */
	if (!g_plant_apply_readback)
		return;

	if (pdu_dxl || pdu_dm || pdu_ub)
		return;

	if (pdu_rs2 && !rs02_probe_kind_mounts_desire(cmd->pdu.data[4]))
		return;

	plant_diag_release_actuator_can();
	actuator_command_mount(cmd);
	peripheral_command_mount(cmd);
}

static void plant_command_dispatch_debug_lanes(const host_command_image_t *cmd)
{
	const uint8_t *raw = (const uint8_t *)cmd;
	const host_debug_lanes_header_t *hdr =
		(const host_debug_lanes_header_t *)(raw + HOST_DEBUG_LANES_HDR_OFF);
	uint16_t arm = hdr->arm_mask;
	host_command_image_t local;

	/* DL header occupies the plant system word — do not decode mcu_state /
	 * plant_apply from those bytes. Debug-lanes RPC is observe/bench;
	 * ESTOP / RECOVERY / plant_apply still travel on interleaved plant CMDH. */
	g_plant_apply_readback = false;
	g_debug_lanes_pending = true;
	g_debug_lanes_arm_mask = arm;

	if (pdb_link_kill_state() == (uint8_t)PDB_KILL_NORMAL)
		pdb_link_set_soft_kill_ready(false);

	/* Remount each armed lane into the legacy pdu[] view so existing
	 * tag parsers (RS2 / DM0 / CFG / DXL / …) stay shared. */
	local = *cmd;
	memset(local.pdb, 0, sizeof(local.pdb));

	if (arm & (1u << HOST_DEBUG_LANE_CFG)) {
		memcpy(local.pdu.data,
		       host_debug_lane_bytes(cmd, HOST_DEBUG_LANE_CFG),
		       HOST_PDU_PAYLOAD_BYTES);
		if (plant_config_is_command(&local)) {
			plant_config_on_command(&local);
			return;
		}
	}

	if (arm & (1u << HOST_DEBUG_LANE_RS)) {
		memcpy(local.pdu.data,
		       host_debug_lane_bytes(cmd, HOST_DEBUG_LANE_RS),
		       HOST_PDU_PAYLOAD_BYTES);
		/* Optional MIT desire packed at lane[12..31] (5 floats). */
		memcpy(&local.actuator_commands[0],
		       &local.pdu.data[12],
		       sizeof(float) * 5u);
		local.actuator_commands[0].meta = 0u;
		if (plant_diag_is_rs2_command(&local))
			plant_diag_on_command(&local);
	}

	if (arm & (1u << HOST_DEBUG_LANE_DM)) {
		memcpy(local.pdu.data,
		       host_debug_lane_bytes(cmd, HOST_DEBUG_LANE_DM),
		       HOST_PDU_PAYLOAD_BYTES);
		if (plant_diag_is_dm_command(&local))
			plant_diag_on_dm_command(&local);
	}

	if (arm & (1u << HOST_DEBUG_LANE_SERVO)) {
		memcpy(local.pdu.data,
		       host_debug_lane_bytes(cmd, HOST_DEBUG_LANE_SERVO),
		       HOST_PDU_PAYLOAD_BYTES);
		if (plant_diag_is_dxl_command(&local))
			plant_diag_on_dxl_command(&local);
	}

	if (arm & (1u << HOST_DEBUG_LANE_PDU)) {
		memcpy(local.pdu.data,
		       host_debug_lane_bytes(cmd, HOST_DEBUG_LANE_PDU),
		       HOST_PDU_PAYLOAD_BYTES);
		if (host_uart_bridge_is_command(&local))
			host_uart_bridge_on_command(&local);
		else if (thermo_is_command(&local))
			thermo_on_command(&local);
	}

	(void)HOST_DEBUG_LANE_CM;
	(void)HOST_DEBUG_LANE_ZE;
	(void)HOST_DEBUG_LANE_LED;
}

void plant_command_image_dispatch_plant(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return;

	plant_command_note_stm32_mode(cmd);

	uint8_t mcu_state = (uint8_t)cmd->system.mcu_state;
	g_mcu_state_readback = mcu_state;
	g_plant_apply_readback = plant_command_extract_plant_apply(cmd, mcu_state);

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

	/* Observe: do not mount desires or release bench lease. */
	if (!g_plant_apply_readback)
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

	g_debug_lanes_pending = false;
	g_debug_lanes_arm_mask = 0u;

	/* DL header overlays the plant system word — never parse stm32_mode from it. */
	if (host_debug_lanes_present(cmd)) {
		g_stm32_mode_readback = HOST_STM32_MODE_DEBUG;
		plant_command_dispatch_debug_lanes(cmd);
		return;
	}

	/* Legacy DBGC: Soft-DFU only via DFU! mailbox tag (see legacy path). */
	g_stm32_mode_readback = plant_command_extract_stm32_mode(cmd);
	if (g_stm32_mode_readback == HOST_STM32_MODE_SOFT_DFU)
		g_stm32_mode_readback = HOST_STM32_MODE_DEBUG;

	plant_command_dispatch_debug_legacy(cmd);
}
