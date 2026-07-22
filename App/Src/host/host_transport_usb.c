#include "host/host_transport.h"
#include "host/host_transport_usb.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t UserTxBufferFS[];

#define USB_TRANSPORT_RX_RING_SIZE 16384u

static uint8_t           rx_ring[USB_TRANSPORT_RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile bool     tx_busy;

void host_transport_usb_rx_push(const uint8_t *data, uint32_t len)
{
	uint16_t head;
	uint16_t tail;
	uint16_t free;
	uint16_t first;

	if (data == NULL || len == 0)
		return;

	/* Contiguous memcpy into the ring (ISR path). Byte-at-a-time push was a
	 * fixed tax on every CDC OUT at 200–500 Hz host × 672 B frames. */
	head = rx_head;
	tail = rx_tail;
	free = (uint16_t)((tail + USB_TRANSPORT_RX_RING_SIZE - head - 1u) %
	                  USB_TRANSPORT_RX_RING_SIZE);
	if (len > free)
		len = free;
	if (len == 0)
		return;

	first = (uint16_t)(USB_TRANSPORT_RX_RING_SIZE - head);
	if (first > len)
		first = (uint16_t)len;
	memcpy(&rx_ring[head], data, first);
	if (len > first)
		memcpy(&rx_ring[0], data + first, len - first);
	rx_head = (uint16_t)((head + len) % USB_TRANSPORT_RX_RING_SIZE);
}

static void usb_init(void)
{
	rx_head = 0;
	rx_tail = 0;
	tx_busy = false;
}

static size_t usb_read(uint8_t *dst, size_t max_len)
{
	uint16_t head;
	uint16_t tail;
	uint16_t avail;
	uint16_t first;
	size_t n;

	if (dst == NULL || max_len == 0)
		return 0;

	head = rx_head;
	tail = rx_tail;
	avail = (uint16_t)((head + USB_TRANSPORT_RX_RING_SIZE - tail) %
	                   USB_TRANSPORT_RX_RING_SIZE);
	if (avail == 0)
		return 0;
	if (max_len < avail)
		avail = (uint16_t)max_len;

	first = (uint16_t)(USB_TRANSPORT_RX_RING_SIZE - tail);
	if (first > avail)
		first = avail;
	memcpy(dst, &rx_ring[tail], first);
	if (avail > first)
		memcpy(dst + first, &rx_ring[0], avail - first);
	rx_tail = (uint16_t)((tail + avail) % USB_TRANSPORT_RX_RING_SIZE);
	n = avail;
	return n;
}

static bool usb_write(const uint8_t *src, size_t len)
{
	if (src == NULL || len == 0 || len > APP_TX_DATA_SIZE)
		return false;
	if (tx_busy)
		return false;

	memcpy(UserTxBufferFS, src, len);
	tx_busy = true;

	if (CDC_Transmit_FS(UserTxBufferFS, (uint16_t)len) != USBD_OK) {
		tx_busy = false;
		return false;
	}
	return true;
}

static bool usb_tx_ready(void)
{
	USBD_CDC_HandleTypeDef *hcdc =
			(USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

	if (hcdc != NULL && hcdc->TxState == 0u)
		tx_busy = false;

	return !tx_busy && (hcdc != NULL && hcdc->TxState == 0u);
}

void host_transport_usb_tx_complete(void)
{
	tx_busy = false;
}

void host_transport_usb_tx_reset(void)
{
	tx_busy = false;
}

const host_transport_ops_t host_transport_usb_ops = {
	.init = usb_init,
	.read = usb_read,
	.write = usb_write,
	.tx_ready = usb_tx_ready,
};
