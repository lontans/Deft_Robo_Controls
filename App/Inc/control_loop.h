// control_loop.h — 500 Hz fixed-rate tick: actuator_desire → plugins → CAN → actuator_state
#pragma once
#include <stdint.h>

extern volatile uint32_t g_control_tick_count; // Defined in control_loop.c

void control_loop_init(void);
void control_loop_tick(void);
