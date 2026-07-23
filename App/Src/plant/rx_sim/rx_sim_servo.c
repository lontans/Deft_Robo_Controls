#include "plant/rx_sim/rx_sim_servo.h"
#include "plant/rx_sim/rx_sim.h"
#include "plant/servo.h"
#include "plant/plugins/dynamixel.h"

/*
 * Soft DXL path: one "txn" worth of state update per plant tick across
 * enabled slots — mirrors unicast wr/rd cadence without UART (and without
 * arming servo_host_session, which would skip actuator CAN).
 */
void rx_sim_servo_tick(void)
{
	uint8_t i;

	if (!rx_sim_servo_enabled())
		return;

	for (i = 0; i < SERVO_COUNT; i++) {
		uint32_t goal;
		uint8_t id;

		if (!servo_table[i].enabled)
			continue;

		id = servo_table[i].id;
		goal = (uint32_t)servo_desire_live[i].native_step_position;
		if (goal < servo_table[i].pos_min)
			goal = servo_table[i].pos_min;
		if (goal > servo_table[i].pos_max)
			goal = servo_table[i].pos_max;

		servo_state_live[i].present_position = (int16_t)goal;
		servo_state_live[i].present_speed = 0;
		servo_state_live[i].moving = 0u;
		servo_state_live[i].target_reached = 1u;
		servo_state_live[i].motor_source_id = id;
		servo_state_live[i].err_input_volt = 0u;
		servo_state_live[i].err_overheating = 0u;
		servo_state_live[i].err_overload = 0u;
	}
}
