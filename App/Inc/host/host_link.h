#pragma once
#include "host/host_exchange_schema.h"
#include <stdbool.h>
#include <stdint.h>

void host_link_init(void);
void host_link_poll_tx(void);
void host_link_poll_rx(void);

bool host_command_image_valid(const host_command_image_t *cmd);
void host_command_image_dispatch(const host_command_image_t *cmd);

uint32_t host_link_last_command_seq(void);

/* True if a valid host command arrived within max_age_ms. */
bool host_link_command_is_fresh(uint32_t max_age_ms);
