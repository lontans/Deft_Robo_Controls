#pragma once

/* Soft servo tick: update present_position from desire, no UART.
 * Does not arm servo_host_session (keeps actuator CAN unblocked). */
void rx_sim_servo_tick(void);
