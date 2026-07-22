#include "host/host_link.h"
#include "host/host_exchange_schema.h"
#include "host/host_transport.h"
#include "plant/plant_command.h"
#include "plant/plant_feedback.h"
#include "plant/plant_diag.h"
#include "plant/plant_timing.h"
#include "plant/control_loop.h"
#include "main.h"
#include <string.h>

static uint32_t              g_last_command_seq;
static uint32_t              g_last_command_ms;
static uint8_t               g_cmd_rx_buf[HOST_COMMAND_IMAGE_BYTES];
static size_t                g_cmd_rx_fill;
static host_feedback_image_t g_fb_tx_frame;
static bool                  g_debug_reply_pending;

static void host_link_rx_resync(void);
static bool host_link_rx_feed_byte(uint8_t b);
static void host_feedback_image_fetch_plant(host_feedback_image_t *out);
static void host_feedback_image_fetch_debug(host_feedback_image_t *out);

uint32_t host_link_last_command_seq(void)
{
	return g_last_command_seq;
}

bool host_command_image_valid(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return false;
	if (cmd->header.layout_version != HOST_LAYOUT_VERSION)
		return false;
	if (cmd->header.byte_size != HOST_COMMAND_IMAGE_BYTES)
		return false;
	if (cmd->header.magic != HOST_COMMAND_MAGIC &&
	    cmd->header.magic != HOST_DEBUG_COMMAND_MAGIC)
		return false;
	return true;
}

void host_command_image_dispatch(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return;

	if (cmd->header.magic == HOST_DEBUG_COMMAND_MAGIC) {
		plant_command_image_dispatch_debug(cmd);
		g_debug_reply_pending = true;
	} else {
		plant_command_image_dispatch_plant(cmd);
	}
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
	g_debug_reply_pending = false;

	host_transport_get()->init();
}

void host_link_begin_loop(void)
{
}

void host_link_poll_rx(void)
{
	const host_transport_ops_t *tp = host_transport_get();
	uint8_t chunk[64];
	size_t n;

	while ((n = tp->read(chunk, sizeof(chunk))) > 0) {
		for (size_t i = 0; i < n; i++)
			(void)host_link_rx_feed_byte(chunk[i]);
	}
}

static bool host_link_magic_at(const uint8_t *buf, uint32_t magic)
{
	return buf[0] == (uint8_t)(magic & 0xFFu) &&
	       buf[1] == (uint8_t)((magic >> 8) & 0xFFu) &&
	       buf[2] == (uint8_t)((magic >> 16) & 0xFFu) &&
	       buf[3] == (uint8_t)((magic >> 24) & 0xFFu);
}

static void host_link_rx_resync(void)
{
	size_t shift = HOST_COMMAND_IMAGE_BYTES;

	for (size_t i = 1; i < HOST_COMMAND_IMAGE_BYTES; i++) {
		if (host_link_magic_at(&g_cmd_rx_buf[i], HOST_COMMAND_MAGIC) ||
		    host_link_magic_at(&g_cmd_rx_buf[i], HOST_DEBUG_COMMAND_MAGIC)) {
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

static bool host_link_rx_feed_byte(uint8_t b)
{
	if (g_cmd_rx_fill >= HOST_COMMAND_IMAGE_BYTES)
		g_cmd_rx_fill = 0;

	g_cmd_rx_buf[g_cmd_rx_fill++] = b;

	if (g_cmd_rx_fill < HOST_COMMAND_IMAGE_BYTES)
		return false;

	const host_command_image_t *cmd =
			(const host_command_image_t *)g_cmd_rx_buf;

	if (!host_command_image_valid(cmd)) {
		host_link_rx_resync();
		return false;
	}

	g_cmd_rx_fill = 0;
	host_command_image_dispatch(cmd);
	return true;
}

static void host_feedback_fill_system(host_feedback_image_t *out)
{
	out->system.control_tick_count = (uint32_t)(g_control_tick_count & 0xFFFu);
	out->system.last_command_seq   = (uint32_t)(host_link_last_command_seq() & 0xFFu);
	out->system.mcu_state_readback = (uint32_t)plant_command_mcu_state_readback();
	(void)plant_runtime_actuator_can_apply();
	out->system.plant_block =
		(uint32_t)plant_runtime_actuator_block_reason() & 0x7Fu;
	plant_timing_system_fill(&out->system);
}

static void host_feedback_image_fetch_plant(host_feedback_image_t *out)
{
	memset(out, 0, sizeof(*out));
	out->header.magic          = HOST_FEEDBACK_MAGIC;
	out->header.layout_version = HOST_LAYOUT_VERSION;
	out->header.byte_size      = HOST_FEEDBACK_IMAGE_BYTES;
	host_feedback_fill_system(out);
	plant_feedback_image_fetch_plant(out);
}

static void host_feedback_image_fetch_debug(host_feedback_image_t *out)
{
	memset(out, 0, sizeof(*out));
	out->header.magic          = HOST_DEBUG_FEEDBACK_MAGIC;
	out->header.layout_version = HOST_LAYOUT_VERSION;
	out->header.byte_size      = HOST_FEEDBACK_IMAGE_BYTES;
	host_feedback_fill_system(out);
	plant_feedback_image_fetch_plant(out);
	plant_feedback_image_fetch_debug_mailbox(&out->pdu);
}

void host_link_poll_tx(void)
{
	for (uint8_t i = 0; i < 8u; i++) {
		if (host_link_poll_tx_once())
			return;
	}
}

bool host_link_poll_tx_once(void)
{
	const host_transport_ops_t *tp = host_transport_get();

	if (plant_diag_blocks_usb_feedback())
		return false;

	if (!tp->tx_ready())
		return false;

	if (g_debug_reply_pending) {
		host_feedback_image_fetch_debug(&g_fb_tx_frame);
		if (!tp->write((const uint8_t *)&g_fb_tx_frame, HOST_FEEDBACK_IMAGE_BYTES))
			return false;
		plant_diag_feedback_sent(g_fb_tx_frame.pdu.data[25]);
		g_debug_reply_pending = false;
		return true;
	}

	host_feedback_image_fetch_plant(&g_fb_tx_frame);
	if (!tp->write((const uint8_t *)&g_fb_tx_frame, HOST_FEEDBACK_IMAGE_BYTES))
		return false;
	return true;
}
