#include "host_transport.h"


// Function stubs for uart transport methods
static void uart_init(void) {}
static size_t uart_read(uint8_t *dst, size_t max_len) {(void)dst; (void)max_len; return 0;}
static bool uart_write (const uint8_t *src, size_t len) {(void)src; (void)len; return true;}
static bool uart_tx_ready(void) {return true;}

// defining uart operations, mapped to host_link
const host_transport_ops_t host_transport_uart_ops = {
		.init = uart_init,
		.read = uart_read,
		.write = uart_write,
		.tx_ready = uart_tx_ready,
};
