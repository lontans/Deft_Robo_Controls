#pragma once
#include <stdint.h>

void host_transport_uart_rx_push(const uint8_t *data, uint32_t len);
void host_transport_uart_tx_complete(void);
