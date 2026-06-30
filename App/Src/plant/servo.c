#include "plant/servo.h"
#include "plant/plant_diag.h"
#include "host/host_link.h"
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

static uint8_t  g_hw_error[SERVO_COUNT];
static uint8_t  g_recover_slot;
static uint8_t  g_recover_phase;
static uint32_t g_recover_t0_ms;

#define SERVO_HW_POLL_CYCLES 40u
#define SERVO_RECOVER_WAIT_MS 500u

static uint8_t g_diag_wr_ok;
static uint8_t g_diag_rd_ok;
static uint8_t g_diag_torque_ok;

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

static bool servo_torque_on_step(void)
{
	uint8_t id;

	while (g_torque_slot < SERVO_COUNT) {
		if (!servo_table[g_torque_slot].enabled) {
			g_torque_slot++;
			continue;
		}

		id = servo_table[g_torque_slot].id;
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
	g_recover_phase = 0u;
}

static void servo_bus_service(void)
{
	uint8_t hw_err;

	if (plant_diag_skip_servo_bus())
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
	servo_bus_reset();
}

void servo_command_mount(const host_command_image_t *cmd)
{
	bool any = false;

	if (cmd == NULL)
		return;

	__disable_irq();
	for (uint8_t i = 0; i < SERVO_COUNT; i++) {
		if (cmd->servos[i].servo_id != 0u) {
			servo_desire_stage[i] = cmd->servos[i];
			any = true;
		}
	}
	g_servo_host_session = any;
	servo_desire_pending = true;
	__enable_irq();
}

bool servo_host_session_active(void)
{
	return g_servo_host_session;
}

void servo_desire_clear(void)
{
	__disable_irq();
	for (uint8_t i = 0; i < SERVO_COUNT; i++)
		memset(&servo_desire_live[i], 0, sizeof(servo_desire_t));
	memset(servo_desire_stage, 0, sizeof(servo_desire_stage));
	servo_desire_pending = false;
	g_servo_host_session = false;
	servo_bus_reset();
	__enable_irq();
}

void servo_apply_desire(void)
{
	__disable_irq();
	if (servo_desire_pending) {
		for (uint8_t i = 0; i < SERVO_COUNT; i++)
			servo_desire_live[i] = servo_desire_stage[i];
		servo_desire_pending = false;
	}
	__enable_irq();
}

void servo_capture_state(void)
{
	servo_bus_service();

	__disable_irq();
	for (uint8_t i = 0; i < SERVO_COUNT; i++)
		servo_state_stage[i] = servo_state_live[i];
	__enable_irq();
}

void servo_feedback_snapshot(host_servo_feedback_t *dst, uint8_t count)
{
	if (dst == NULL || count == 0)
		return;

	uint8_t n = (count < SERVO_COUNT) ? count : SERVO_COUNT;

	__disable_irq();
	for (uint8_t i = 0; i < n; i++)
		dst[i] = servo_state_stage[i];
	__enable_irq();

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
}
