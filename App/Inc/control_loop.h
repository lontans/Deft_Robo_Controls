// control_loop.h — 500 Hz fixed-rate tick: desires → plugins → CAN → states
#pragma once

void control_loop_init(void);
void control_loop_tick(void);
