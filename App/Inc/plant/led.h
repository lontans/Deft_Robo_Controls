#pragma once

#include "host/host_exchange_schema.h"
#include <stdint.h>

void led_init(void);
void led_command_mount(const host_command_image_t *cmd);
void led_service(void);
void led_feedback_snapshot(host_led_feedback_t *dst);

/* Bench/rx_sim: start TEST chase, or black-out full SK9822 chain. */
void led_sim_kick_test(uint8_t brightness_0_31);
void led_blank_transmit(void);
void led_force_all_off(void);
