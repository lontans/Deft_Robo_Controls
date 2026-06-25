#include "host/host_link.h"
#include "host/host_exchange_schema.h"
#include "host/host_transport.h"
#include "plant/plant_command.h"
#include "plant/plant_feedback.h"
#include "plant/control_loop.h"
#include "main.h"
#include <string.h>

static uint32_t              g_last_command_seq;
static uint32_t              g_last_command_ms;
static uint8_t               g_cmd_rx_buf[HOST_COMMAND_IMAGE_BYTES];
static size_t                g_cmd_rx_fill;
static host_feedback_image_t g_fb_tx_frame;

static void host_link_rx_resync(void);
static void host_link_rx_feed_byte(uint8_t b);
static void host_feedback_image_fetch(host_feedback_image_t *out);

uint32_t host_link_last_command_seq(void)
{
	return g_last_command_seq;
}

bool host_command_image_valid(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return false;
	if (cmd->header.magic != HOST_COMMAND_MAGIC)
		return false;
	if (cmd->header.layout_version != HOST_LAYOUT_VERSION)
		return false;
	if (cmd->header.byte_size != HOST_COMMAND_IMAGE_BYTES)
		return false;

	return true;
}

void host_command_image_dispatch(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return;

	plant_command_image_dispatch(cmd);
	g_last_command_seq = cmd->header.seq;
	g_last_command_ms  = HAL_GetTick();
}

bool host_link_command_is_fresh(uint32_t max_age_ms)
{
	if (g_last_command_ms == 0u)
		return false;

	return (HAL_GetTick() - g_last_command_ms) <= max_age_ms;
}

void host_link_init(void)
{
	g_last_command_seq = 0;
	g_last_command_ms  = 0;
	g_cmd_rx_fill      = 0;

	host_transport_get()->init();
}

void host_link_poll_rx(void)
{
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
	host_command_image_dispatch(cmd);
}

static void host_feedback_image_fetch(host_feedback_image_t *out)
{
	if (out == NULL)
		return;

	memset(out, 0, sizeof(*out));
	out->header.magic          = HOST_FEEDBACK_MAGIC;
	out->header.layout_version = HOST_LAYOUT_VERSION;
	out->header.byte_size      = HOST_FEEDBACK_IMAGE_BYTES;

	out->system.control_tick_count = (uint32_t)(g_control_tick_count & 0xFFFu);
	out->system.last_command_seq   = (uint32_t)(host_link_last_command_seq() & 0xFFu);
	out->system.mcu_state_readback = (uint32_t)plant_command_mcu_state_readback();

	plant_feedback_image_fetch(out);
}

void host_link_poll_tx(void)
{
	const host_transport_ops_t *tp = host_transport_get();

	if (!tp->tx_ready())
		return;

	host_feedback_image_fetch(&g_fb_tx_frame);
	(void)tp->write((const uint8_t *)&g_fb_tx_frame, HOST_FEEDBACK_IMAGE_BYTES);
}
