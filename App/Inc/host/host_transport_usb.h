#pragma once
#include <stdint.h>

void host_transport_usb_rx_push(const uint8_t *data, uint32_t len);
void host_transport_usb_tx_complete(void);
void host_transport_usb_tx_reset(void);
