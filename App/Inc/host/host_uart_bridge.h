#pragma once
#include "host/host_exchange_schema.h"
#include <stdbool.h>

/*
 * Raw UART4 <-> PDU bridge (see host/uart4_mode.h). Command PDU tag 'U','B'
 * carries bytes to transmit out UART4; feedback PDU tag 'u' carries bytes
 * received on UART4 since the last poll. No-ops unless UART4_MODE ==
 * UART4_MODE_DAMIAO_BRIDGE, so these are safe to call unconditionally.
 */
void host_uart_bridge_init(void);
bool host_uart_bridge_is_command(const host_command_image_t *cmd);
void host_uart_bridge_on_command(const host_command_image_t *cmd);
void host_uart_bridge_feedback_fill(host_pdu_feedback_t *pdu);
