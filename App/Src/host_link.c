#include "host_link.h"
#include "host_exchange.h"
#include "host_transport.h"
#include "actuator.h"
#include "feedback_image.h"
#include <string.h>

// TODO change static to header defined func when actually implementing command seq
static uint32_t             g_last_command_seq;
static uint8_t              g_cmd_rx_buf[HOST_COMMAND_IMAGE_BYTES];
static size_t               g_cmd_rx_fill;
static host_feedback_image_t g_fb_tx_frame;

static void host_link_rx_resync(void);
static void host_link_rx_feed_byte(uint8_t b);

uint32_t host_link_last_command_seq(void){
	return g_last_command_seq;
}

// Function ensures header contract is valid
bool host_command_image_valid(const host_command_image_t *cmd){
	if (cmd == NULL)                                        // Check image existence
		return false;
	if (cmd->header.magic != HOST_COMMAND_MAGIC)            // Check magic (correct leading ID)
		return false;
	if (cmd->header.layout_version != HOST_LAYOUT_VERSION)  // Check layout version matches expectations
		return false;
	if (cmd->header.byte_size != HOST_COMMAND_IMAGE_BYTES)  // Check correct byte scaling
		return false;

	return true;
}

void command_image_apply(const host_command_image_t *cmd){
	actuator_stage_desires(cmd);           // Stage desires into buffer, read by control loop
	g_last_command_seq = cmd->header.seq;  // Update last command to the image header sent to the desire buffer
	// TODO , cmd->system, servo,led, pdu, other staging etc
}

void host_link_init(void) {
	g_last_command_seq = 0;
	g_cmd_rx_fill      = 0;

	host_transport_get()->init();  // Call the selected transport method (usb vs uart)
}

// Poll rx incoming and transfer to rx buffer
void host_link_poll_rx(void) {
	const host_transport_ops_t *tp = host_transport_get();
	uint8_t chunk[64];
	size_t n;

	while ((n = tp->read(chunk, sizeof(chunk))) > 0) {
		for (size_t i = 0; i < n; i++)
			host_link_rx_feed_byte(chunk[i]);
	}
}

static void host_link_rx_resync(void)
{
	static const uint8_t magic[] = { 0x48, 0x44, 0x4D, 0x43 };
	size_t shift = HOST_COMMAND_IMAGE_BYTES;

	for (size_t i = 1; i < HOST_COMMAND_IMAGE_BYTES; i++) {
		if (memcmp(&g_cmd_rx_buf[i], magic, sizeof(magic)) == 0) {
			shift = i;
			break;
		}
	}

	if (shift < HOST_COMMAND_IMAGE_BYTES) {
		size_t remain = HOST_COMMAND_IMAGE_BYTES - shift;
		memmove(g_cmd_rx_buf, &g_cmd_rx_buf[shift], remain);
		g_cmd_rx_fill = remain;
	} else {
		g_cmd_rx_fill = 0;
	}
}

static void host_link_rx_feed_byte(uint8_t b)
{
	if (g_cmd_rx_fill >= HOST_COMMAND_IMAGE_BYTES)
		g_cmd_rx_fill = 0;

	g_cmd_rx_buf[g_cmd_rx_fill++] = b;

	if (g_cmd_rx_fill < HOST_COMMAND_IMAGE_BYTES)
		return;

	const host_command_image_t *cmd =
			(const host_command_image_t *)g_cmd_rx_buf;

	if (!host_command_image_valid(cmd)) {
		host_link_rx_resync();
		return;
	}

	g_cmd_rx_fill = 0;
	command_image_apply(cmd);
}

void host_link_poll_tx(void)
{
	const host_transport_ops_t *tp = host_transport_get();

	if (!tp->tx_ready())
		return;

	feedback_image_build(&g_fb_tx_frame);
	(void)tp->write((const uint8_t *)&g_fb_tx_frame, HOST_FEEDBACK_IMAGE_BYTES);
}


