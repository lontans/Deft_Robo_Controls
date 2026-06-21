#include "host_transport.h"

static void usb_init(void) { }
static size_t usb_read(uint8_t *dst, size_t max_len) { (void)dst; (void)max_len; return 0; }
static bool usb_write(const uint8_t *src, size_t len) { (void)src; (void)len; return true; }
static bool usb_tx_ready(void) { return true; }

const host_transport_ops_t host_transport_usb_ops = {
	.init = usb_init,
	.read = usb_read,
	.write = usb_write,
	.tx_ready = usb_tx_ready,
};
