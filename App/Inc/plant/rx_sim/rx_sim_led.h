#pragma once

#include <stdbool.h>

/* Force SPI3 LED role while enabled; restore previous role on disable. */
void rx_sim_led_set_active(bool enable);
