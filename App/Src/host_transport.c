#include "host_transport.h"


// Logic for defining which functions to use depending on USB vs UART interface
#if HOST_TRANSPORT_UART
extern const host_transport_ops_t host_transport_uart_ops;
const host_transport_ops_t *host_transport_get(void) {return &host_transport_uart_ops;}
#else
extern const host_transport_ops_t host_transport_usb_ops;
const host_transport_ops_t *host_transport_get(void) { return &host_transport_usb_ops; }
#endif
