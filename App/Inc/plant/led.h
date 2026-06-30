#pragma once

#include "host/host_exchange_schema.h"

void led_init(void);
void led_command_mount(const host_command_image_t *cmd);
void led_service(void);
void led_feedback_snapshot(host_led_feedback_t *dst);
