#include "plant/servo.h"
#include "plant/plant_diag.h"
#include "plant/plant_timing.h"
#include "plant/rx_sim/rx_sim.h"
#include "plant/rx_sim/rx_sim_servo.h"
#include "host/host_link.h"
#include "plant/plant_crit.h"
#include "main.h"
#include <string.h>

static servo_desire_t servo_desire_stage[SERVO_COUNT];
static servo_state_t  servo_state_stage[SERVO_COUNT];
static volatile bool  servo_desire_pending;

typedef enum {
	SERVO_ST_TORQUE = 0,
	SERVO_ST_UNI_WR,
	SERVO_ST_UNI_RD,
	SERVO_ST_RECOVER,
} servo_step_t;

static servo_step_t g_step;
static uint8_t        g_torque_slot;
static bool           g_torque_done;
static uint8_t        g_slot;
static uint16_t       g_bus_cycles;
static volatile bool  g_servo_host_session;
static bool           g_session_was;
static bool           g_pose_latched[SERVO_COUNT];

static uint8_t  g_hw_error[SERVO_COUNT];
static uint8_t  g_recover_slot;
static uint8_t  g_recover_phase;
static uint32_t g_recover_t0_ms;

#define SERVO_HW_POLL_CYCLES 80u
#define SERVO_RECOVER_WAIT_MS 500u
/* Each DXL R/W may block ≤ DXL_PLANT_RX_TIMEOUT_MS. Rate-limit so we do
 * not stack multiple TXNs every app_run spin. */
#define SERVO_BUS_MIN_PERIOD_MS 10u

static uint8_t g_diag_wr_ok;
static uint8_t g_diag_rd_ok;
static uint8_t g_diag_torque_ok;
static uint32_t g_last_bus_poll_ms;



static bool servo_host_command_stale(void)
{
	return !host_link_command_is_fresh(SERVO_HOST_STALE_MS);
}

servo_desire_t servo_desire_live[SERVO_COUNT];
servo_state_t  servo_state_live[SERVO_COUNT];

static uint32_t servo_clamp_goal(uint8_t slot, int16_t native_step)
{
	uint32_t goal = (uint32_t)native_step;

	if (goal < servo_table[slot].pos_min)
		goal = servo_table[slot].pos_min;
	if (goal > servo_table[slot].pos_max)
		goal = servo_table[slot].pos_max;

	return goal;
}

static bool servo_read_hw_error(uint8_t slot, uint8_t *err_out)
{
	uint8_t id;
	uint8_t err;

	if (slot >= SERVO_COUNT || !servo_table[slot].enabled || err_out == NULL)
		return false;

	id = servo_table[slot].id;
	if (!dxl_read_u8(id, DXL_ADDR_HW_ERROR_STATUS, &err))
		return false;

	*err_out = err;
	return true;
}

static bool servo_recovery_step(void)
{
	uint8_t id;

	if (g_recover_slot >= SERVO_COUNT || !servo_table[g_recover_slot].enabled)
		goto recovery_done;

	id = servo_table[g_recover_slot].id;

	switch (g_recover_phase) {
	case 0u:
		(void)dxl_reboot(id);
		g_recover_t0_ms = dxl_port_millis();
		g_recover_phase = 1u;
		return true;

	case 1u:
		if ((dxl_port_millis() - g_recover_t0_ms) < SERVO_RECOVER_WAIT_MS)
			return true;
		g_recover_phase = 2u;
		/* fall through */

	default:
		if (dxl_write_u8(id, DXL_ADDR_TORQUE_ENABLE, DXL_TORQUE_ON))
			g_diag_torque_ok++;
		g_hw_error[g_recover_slot] = 0u;
		goto recovery_done;
	}

recovery_done:
	g_recover_phase = 0u;
	g_step          = SERVO_ST_UNI_WR;
	return true;
}

static void servo_begin_recovery(uint8_t slot, uint8_t err)
{
	g_hw_error[slot]  = err;
	g_recover_slot    = slot;
	g_recover_phase   = 0u;
	g_step            = SERVO_ST_RECOVER;
}

static bool servo_unicast_write_slot(uint8_t slot)
{
	uint32_t goal;
	uint8_t  id;

	if (slot >= SERVO_COUNT || !servo_table[slot].enabled)
		return false;

	id = servo_table[slot].id;
	goal = servo_clamp_goal(slot, servo_desire_live[slot].native_step_position);

	if (!dxl_write_u32(id, DXL_ADDR_GOAL_POSITION, goal))
		return false;

	return true;
}

static bool servo_unicast_read_slot(uint8_t slot)
{
	uint32_t present;
	uint8_t  id;

	if (slot >= SERVO_COUNT || !servo_table[slot].enabled)
		return false;

	id = servo_table[slot].id;
	if (!dxl_read_u32(id, DXL_ADDR_PRESENT_POSITION, &present))
		return false;

	servo_state_live[slot].present_position = (int16_t)present;
	servo_state_live[slot].motor_source_id  = id;
	return true;
}

static void servo_advance_slot(uint8_t *slot)
{
	do {
		*slot = (uint8_t)((*slot + 1u) % SERVO_COUNT);
	} while (*slot < SERVO_COUNT && !servo_table[*slot].enabled);
}

static bool servo_write_motion_profile(uint8_t slot, uint8_t id)
{
	const servo_config_t *cfg = &servo_table[slot];

	/* RAM items — safe with torque off. Profile vel 0 = max speed (jerk). */
	if (!dxl_write_u16(id, DXL_ADDR_POSITION_D_GAIN, cfg->position_d_gain))
		return false;
	if (!dxl_write_u16(id, DXL_ADDR_POSITION_P_GAIN, cfg->position_p_gain))
		return false;
	if (!dxl_write_u32(id, DXL_ADDR_PROFILE_ACCEL,
	                   (uint32_t)cfg->default_profile_accel))
		return false;
	if (!dxl_write_u32(id, DXL_ADDR_PROFILE_VELOCITY,
	                   (uint32_t)cfg->default_profile_vel))
		return false;
	return true;
}

static bool servo_torque_on_step(void)
{
	uint8_t id;
	uint32_t present;

	while (g_torque_slot < SERVO_COUNT) {
		if (!servo_table[g_torque_slot].enabled ||
		    servo_desire_live[g_torque_slot].servo_id == 0u) {
			g_torque_slot++;
			continue;
		}

		id = servo_table[g_torque_slot].id;

		/* Prior noreply experiment may have left Status Return Level=READ
		 * (ACK writes then hang). Always force ALL with a noreply TX first. */
		if (!dxl_write_u8_noreply(id, DXL_ADDR_STATUS_RETURN_LEVEL,
		                          DXL_STATUS_RETURN_ALL))
			return false;

		/* Soft-start: latch goal to present before torque so we never
		 * slew from an unknown host seed (e.g. table mid). */
		if (dxl_read_u32(id, DXL_ADDR_PRESENT_POSITION, &present)) {
			if (present < servo_table[g_torque_slot].pos_min)
				present = servo_table[g_torque_slot].pos_min;
			if (present > servo_table[g_torque_slot].pos_max)
				present = servo_table[g_torque_slot].pos_max;
			if (!dxl_write_u32(id, DXL_ADDR_GOAL_POSITION, present))
				return false;
			servo_state_live[g_torque_slot].present_position =
				(int16_t)present;
			servo_state_live[g_torque_slot].motor_source_id = id;
			servo_desire_live[g_torque_slot].native_step_position =
				(int16_t)present;
			servo_desire_stage[g_torque_slot].native_step_position =
				(int16_t)present;
			g_pose_latched[g_torque_slot] = true;
		}

		/* Profile/gains before torque — without this XL330 wakes at max vel.
		 * Non-fatal: still torque-on if a RAM write NACKs so we don't wedge. */
		(void)servo_write_motion_profile(g_torque_slot, id);

		if (!dxl_write_u8(id, DXL_ADDR_TORQUE_ENABLE, DXL_TORQUE_ON))
			return false;

		g_diag_torque_ok++;
		g_torque_slot++;
		return true;
	}

	g_torque_done = true;
	g_step        = SERVO_ST_UNI_WR;
	return true;
}

static void servo_bus_reset(void)
{
	g_step        = SERVO_ST_TORQUE;
	g_torque_slot = 0u;
	g_torque_done = false;
	g_slot        = 0u;
	g_bus_cycles  = 0u;
	memset(g_hw_error, 0, sizeof(g_hw_error));
	memset(g_pose_latched, 0, sizeof(g_pose_latched));
	g_recover_phase = 0u;
}

static void servo_torque_off_all(void)
{
	uint8_t i;

	for (i = 0; i < SERVO_COUNT; i++) {
		if (!servo_table[i].enabled)
			continue;
		(void)dxl_write_u8(servo_table[i].id, DXL_ADDR_TORQUE_ENABLE,
		                   DXL_TORQUE_OFF);
	}
}

static void servo_bus_service(void)
{
	uint8_t hw_err;

	if (plant_diag_skip_servo_bus())
		return;

	/* Plant teleop does not mount servo commands. DXL runs in
	 * servo_bus_poll() after FB TX — not on the TIM6 autonomy path. */
	if (!g_servo_host_session)
		return;

	switch (g_step) {
	case SERVO_ST_TORQUE:
		if (g_torque_done) {
			g_step = SERVO_ST_UNI_WR;
			break;
		}
		(void)servo_torque_on_step();
		return;

	case SERVO_ST_RECOVER:
		(void)servo_recovery_step();
		return;

	case SERVO_ST_UNI_WR:
		if (!servo_host_command_stale()) {
			if (servo_unicast_write_slot(g_slot))
				g_diag_wr_ok++;
		}
		g_step = SERVO_ST_UNI_RD;
		return;

	case SERVO_ST_UNI_RD:
	default:
		if ((g_bus_cycles % SERVO_HW_POLL_CYCLES) == 0u) {
			if (servo_read_hw_error(g_slot, &hw_err) && hw_err != 0u) {
				servo_begin_recovery(g_slot, hw_err);
				g_bus_cycles++;
				return;
			}
		} else if (servo_unicast_read_slot(g_slot)) {
			g_diag_rd_ok++;
		}

		g_bus_cycles++;
		servo_advance_slot(&g_slot);
		g_step = SERVO_ST_UNI_WR;
		return;
	}
}

void servo_init(void)
{
	memset(servo_desire_live, 0, sizeof(servo_desire_live));
	memset(servo_state_live, 0, sizeof(servo_state_live));
	memset(servo_desire_stage, 0, sizeof(servo_desire_stage));
	memset(servo_state_stage, 0, sizeof(servo_state_stage));
	servo_desire_pending = false;
	g_servo_host_session = false;
	g_diag_wr_ok = 0u;
	g_diag_rd_ok = 0u;
	g_diag_torque_ok = 0u;
	g_last_bus_poll_ms = 0u;
	servo_bus_reset();
}

void servo_command_mount(const host_command_image_t *cmd)
{
	bool any = false;

	if (cmd == NULL)
		return;

	plant_crit_enter();
	for (uint8_t i = 0; i < SERVO_COUNT; i++) {
		if (cmd->servos[i].servo_id != 0u) {
			servo_desire_stage[i] = cmd->servos[i];
			any = true;
		}
	}
	g_servo_host_session = any;
	servo_desire_pending = true;
	plant_crit_exit();
}

bool servo_host_session_active(void)
{
	return g_servo_host_session;
}

void servo_desire_clear(void)
{
	plant_crit_enter();
	for (uint8_t i = 0; i < SERVO_COUNT; i++)
		memset(&servo_desire_live[i], 0, sizeof(servo_desire_t));
	memset(servo_desire_stage, 0, sizeof(servo_desire_stage));
	servo_desire_pending = false;
	g_servo_host_session = false;
	plant_crit_exit();
	/* Torque-off runs on next servo_bus_poll session falling edge. */
	servo_bus_reset();
}

void servo_apply_desire(void)
{
	plant_crit_enter();
	if (servo_desire_pending) {
		for (uint8_t i = 0; i < SERVO_COUNT; i++) {
			/* During soft-start, host may still stream a stale pose.
			 * Keep the present-position latch until torque phase ends. */
			if (!g_torque_done && g_pose_latched[i]) {
				int16_t hold = servo_desire_live[i].native_step_position;

				servo_desire_live[i] = servo_desire_stage[i];
				servo_desire_live[i].native_step_position = hold;
			} else {
				servo_desire_live[i] = servo_desire_stage[i];
			}
		}
		servo_desire_pending = false;
	}
	plant_crit_exit();
}

void servo_capture_state(void)
{
	/* Soft servo sim does not arm host session / block actuator CAN.
	 * Real DXL UART runs in servo_bus_poll() after host FB TX. */
	if (rx_sim_servo_enabled() && !servo_host_session_active())
		rx_sim_servo_tick();

	plant_crit_enter();
	for (uint8_t i = 0; i < SERVO_COUNT; i++)
		servo_state_stage[i] = servo_state_live[i];
	plant_crit_exit();
}

void servo_bus_poll(void)
{
	uint32_t now;

	if (rx_sim_servo_enabled() && !servo_host_session_active())
		return;

	/* Session end: drop torque so leave_idle / clear_servos actually stop. */
	if (!g_servo_host_session) {
		if (g_session_was) {
			servo_torque_off_all();
			servo_bus_reset();
			g_session_was = false;
		}
		return;
	}
	g_session_was = true;

	now = HAL_GetTick();
	if ((now - g_last_bus_poll_ms) < SERVO_BUS_MIN_PERIOD_MS)
		return;
	g_last_bus_poll_ms = now;

	/* One unicast TXN; blocking RX is OK here — not on the TIM6 path. */
	servo_bus_service();
}

void servo_feedback_snapshot(host_servo_feedback_t *dst, uint8_t count)
{
	if (dst == NULL || count == 0)
		return;

	uint8_t n = (count < SERVO_COUNT) ? count : SERVO_COUNT;

	plant_crit_enter();
	for (uint8_t i = 0; i < n; i++)
		dst[i] = servo_state_stage[i];
	plant_crit_exit();

	for (uint8_t i = n; i < count; i++)
		memset(&dst[i], 0, sizeof(dst[i]));
}

void servo_diag_feedback_fill(host_pdu_feedback_t *pdu)
{
	int16_t p0;
	int16_t p1;
	int16_t d0;
	int16_t d1;

	if (pdu == NULL)
		return;

	p0 = servo_state_live[0].present_position;
	p1 = servo_state_live[1].present_position;
	d0 = servo_desire_live[0].native_step_position;
	d1 = servo_desire_live[1].native_step_position;

	memset(pdu->data, 0, sizeof(pdu->data));
	pdu->data[0] = (uint8_t)'S';
	pdu->data[1] = (uint8_t)'V';
	pdu->data[2] = (uint8_t)'D';
	pdu->data[3] = g_torque_done ? 1u : 0u;
	pdu->data[4] = g_torque_slot;
	pdu->data[5] = (uint8_t)g_step;
	pdu->data[6] = g_slot;
	pdu->data[7] = g_hw_error[0];
	pdu->data[8] = g_diag_wr_ok;
	pdu->data[9] = g_diag_rd_ok;
	pdu->data[10] = g_diag_torque_ok;
	pdu->data[11] = g_hw_error[1];
	pdu->data[12] = (uint8_t)(p0 & 0xFF);
	pdu->data[13] = (uint8_t)((p0 >> 8) & 0xFF);
	pdu->data[14] = (uint8_t)(p1 & 0xFF);
	pdu->data[15] = (uint8_t)((p1 >> 8) & 0xFF);
	pdu->data[16] = servo_state_live[0].motor_source_id;
	pdu->data[17] = servo_state_live[1].motor_source_id;
	pdu->data[18] = servo_host_command_stale() ? 1u : 0u;
	pdu->data[19] = (uint8_t)(d0 & 0xFF);
	pdu->data[20] = (uint8_t)((d0 >> 8) & 0xFF);
	pdu->data[21] = (uint8_t)(d1 & 0xFF);
	pdu->data[22] = (uint8_t)((d1 >> 8) & 0xFF);
	plant_timing_svd_fill(pdu);
}
