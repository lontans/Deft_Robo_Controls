#pragma once
#include <stdint.h>

extern volatile uint32_t g_control_tick_count;

void control_loop_init(void);
void control_loop_tick(void);
