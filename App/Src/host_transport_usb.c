#include "host_transport.h"
#include "host_transport_usb.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t UserTxBufferFS[];

#define USB_TRANSPORT_RX_RING_SIZE 2048u  // Per CubeMX, size is 2048 bytes

static uint8_t           rx_ring[USB_TRANSPORT_RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;

static volatile bool     tx_busy;

void host_transport_usb_rx_push(const uint8_t *data, uint32_t len)
{
	if (data == NULL || len == 0)
		return;

	for (uint32_t i = 0; i < len; i++) {
		uint16_t next = (uint16_t)((rx_head + 1u) % USB_TRANSPORT_RX_RING_SIZE);
		if (next == rx_tail)
			break; // break, dont increment i, drop to next transmission packet
		rx_ring[rx_head] = data[i]; // copy data into usb rx ring
		rx_head = next; // increment, compute next item
	}
}

static void usb_init(void){
	rx_head = 0;
	rx_tail = 0;
	tx_busy = false;
}

static size_t usb_read(uint8_t *dst, size_t max_len) // reading pushed rx rings into dst
{
	size_t n = 0;

	if (dst == NULL || max_len == 0)
		return 0;

	while (n < max_len && rx_tail != rx_head) {
		dst[n++] = rx_ring[rx_tail];
		rx_tail = (uint16_t)((rx_tail +1u) % USB_TRANSPORT_RX_RING_SIZE);
	}

	return n; // returns size of data input to dst
}

static bool usb_write(const uint8_t *src, size_t len)
{
	if (src == NULL || len == 0 || len > APP_TX_DATA_SIZE)
		return false;
	if (tx_busy)
		return false;

	memcpy (UserTxBufferFS, src, len);

	if (CDC_Transmit_FS(UserTxBufferFS, (uint16_t)len) == USBD_BUSY)
			return false;

	tx_busy = true;
	return true;
}

static bool usb_tx_ready(void)
{
	if (tx_busy)
		return false;
	USBD_CDC_HandleTypeDef *hcdc =
			(USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

	return (hcdc == NULL || hcdc->TxState == 0); // OR gate
}

void host_transport_usb_tx_complete(void)
{
	tx_busy = false;
}

const host_transport_ops_t host_transport_usb_ops = {
	.init = usb_init,
	.read = usb_read,
	.write = usb_write,
	.tx_ready = usb_tx_ready,
};
