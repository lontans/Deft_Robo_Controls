#pragma once
#include "host_exchange.h"
#include <stdbool.h>
#include <stdint.h>

void host_link_init(void);
void host_link_poll_tx(void);
void host_link_poll_rx(void);

bool host_command_image_valid(const host_command_image_t *cmd);
void command_image_apply(const host_command_image_t *cmd);

uint32_t host_link_last_command_seq(void);
