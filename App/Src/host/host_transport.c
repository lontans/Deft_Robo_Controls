#include "host/host_transport.h"

#if HOST_TRANSPORT_UART
extern const host_transport_ops_t host_transport_uart_ops;
const host_transport_ops_t *host_transport_get(void) { return &host_transport_uart_ops; }
#else
extern const host_transport_ops_t host_transport_usb_ops;
const host_transport_ops_t *host_transport_get(void) { return &host_transport_usb_ops; }
#endif
