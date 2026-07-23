#include "host/host_link.h"
#include "host/host_exchange_schema.h"
#include "host/host_transport.h"
#include "host/pdb_link.h"
#include "plant/plant_command.h"
#include "plant/plant_feedback.h"
#include "plant/plant_diag.h"
#include "plant/plant_timing.h"
#include "plant/plant_crit.h"
#include "plant/control_loop.h"
#include "main.h"
#include <string.h>

static uint32_t              g_last_command_seq;
static uint32_t              g_last_applied_seq;
static uint32_t              g_last_command_ms;
static uint8_t               g_cmd_rx_buf[HOST_COMMAND_IMAGE_BYTES];
static size_t                g_cmd_rx_fill;
static host_feedback_image_t g_fb_tx_frame;
static bool                  g_debug_reply_pending;

/* Plant command mount deferred to control_loop_service (TIM6-tick gated) —
 * see host_link_apply_pending_plant(). host_link_poll_rx() only stages the
 * latest coalesced image here; it does not mount. Peripheral (servo/LED)
 * mount is staged separately for host_link_apply_pending_peripheral(). */
static host_command_image_t  g_plant_pending_image;
static volatile bool         g_plant_pending;
static host_command_image_t  g_periph_pending_image;
static volatile bool         g_periph_pending;

static void host_link_rx_resync(void);
static void host_link_rx_consume_ready(bool *have_plant, host_command_image_t *plant_hold);
static void host_feedback_image_fetch_plant(host_feedback_image_t *out);
static void host_feedback_image_fetch_debug(host_feedback_image_t *out);

uint32_t host_link_last_command_seq(void)
{
	return g_last_command_seq;
}

uint32_t host_link_last_applied_seq(void)
{
	return g_last_applied_seq;
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
		plant_crit_enter();
		g_last_applied_seq = cmd->header.seq;
		g_periph_pending_image = *cmd;
		g_periph_pending = true;
		plant_crit_exit();
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
	g_last_applied_seq = 0;
	g_last_command_ms  = 0;
	g_cmd_rx_fill      = 0;
	g_debug_reply_pending = false;
	g_plant_pending = false;
	g_periph_pending = false;

	host_transport_get()->init();
}

void host_link_begin_loop(void)
{
}

void host_link_poll_rx(void)
{
	const host_transport_ops_t *tp = host_transport_get();
	uint8_t chunk[256];
	size_t n;
	host_command_image_t plant_hold;
	bool have_plant = false;

	/* Drain USB ring; coalesce plant images to the latest per lap so a
	 * 200–500 Hz host does not mount 25 slots for every queued frame. */
	while ((n = tp->read(chunk, sizeof(chunk))) > 0) {
		size_t off = 0;

		while (off < n) {
			size_t need = HOST_COMMAND_IMAGE_BYTES - g_cmd_rx_fill;
			size_t take = n - off;

			if (take > need)
				take = need;
			memcpy(&g_cmd_rx_buf[g_cmd_rx_fill], &chunk[off], take);
			g_cmd_rx_fill += take;
			off += take;

			if (g_cmd_rx_fill >= HOST_COMMAND_IMAGE_BYTES)
				host_link_rx_consume_ready(&have_plant, &plant_hold);
		}
	}

	if (have_plant) {
		/* Stage mount for TIM6 (control_loop_service) — do not remount 25
		 * slots on every HostTask spin. Advance last_command_seq here so FB
		 * last_command_seq / cmd_rx_seq = command received (USB RX);
		 * cmd_applied_seq advances later in host_link_apply_pending_plant. */
		plant_crit_enter();
		g_plant_pending_image = plant_hold;
		g_plant_pending = true;
		plant_crit_exit();
		g_last_command_seq = plant_hold.header.seq;
		g_last_command_ms = HAL_GetTick();
	}
}

void host_link_apply_pending_plant(void)
{
	host_command_image_t local;
	bool have = false;

	plant_crit_enter();
	if (g_plant_pending) {
		g_plant_pending = false;
		local = g_plant_pending_image;
		g_last_applied_seq = local.header.seq;
		have = true;
	}
	plant_crit_exit();

	if (!have)
		return;

	plant_command_image_dispatch_plant(&local);

	plant_crit_enter();
	g_periph_pending_image = local;
	g_periph_pending = true;
	plant_crit_exit();
}

void host_link_apply_pending_peripheral(void)
{
	host_command_image_t local;
	bool have = false;

	plant_crit_enter();
	if (g_periph_pending) {
		g_periph_pending = false;
		local = g_periph_pending_image;
		have = true;
	}
	plant_crit_exit();

	if (!have)
		return;

	peripheral_command_mount(&local);
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

static void host_link_rx_consume_ready(bool *have_plant, host_command_image_t *plant_hold)
{
	const host_command_image_t *cmd =
			(const host_command_image_t *)g_cmd_rx_buf;

	if (!host_command_image_valid(cmd)) {
		host_link_rx_resync();
		return;
	}

	g_cmd_rx_fill = 0;

	if (cmd->header.magic == HOST_DEBUG_COMMAND_MAGIC) {
		/* Bench/debug must not be coalesced away. */
		host_command_image_dispatch(cmd);
		return;
	}

	if (plant_hold != NULL && have_plant != NULL) {
		memcpy(plant_hold, cmd, sizeof(*plant_hold));
		*have_plant = true;
	} else {
		host_command_image_dispatch(cmd);
	}
}

static void host_feedback_fill_system(host_feedback_image_t *out)
{
	uint32_t rx_seq = host_link_last_command_seq();
	uint32_t applied_seq = host_link_last_applied_seq();

	out->system.control_tick_count = (uint32_t)(g_control_tick_count & 0xFFFu);
	/* 8-bit + u32 readbacks of the same USB-RX-staged command seq. */
	out->system.last_command_seq = (uint32_t)(rx_seq & 0xFFu);
	out->system.cmd_rx_seq = rx_seq;
	out->system.cmd_applied_seq = applied_seq;
	out->system.mcu_state_readback = (uint32_t)plant_command_mcu_state_readback();
	(void)plant_runtime_actuator_can_apply();
	out->system.plant_block =
		(uint32_t)plant_runtime_actuator_block_reason() & 0x7Fu;
	out->system.kill_state = pdb_link_kill_state();
	out->system.kill_reason = pdb_link_kill_reason();
	out->system.estop_sense = pdb_link_estop_sense();
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
	static uint32_t s_last_plant_fb_tick = 0xFFFFFFFFu;

	/* Plant FB at most once per TIM6 tick (500 Hz). Superloop otherwise
	 * rebuilds 672 B on every spin and starves all-25 under a fast host. */
	if (!g_debug_reply_pending &&
	    s_last_plant_fb_tick == (uint32_t)g_control_tick_count) {
		return;
	}

	for (uint8_t i = 0; i < 8u; i++) {
		if (host_link_poll_tx_once()) {
			if (!g_debug_reply_pending)
				s_last_plant_fb_tick = (uint32_t)g_control_tick_count;
			return;
		}
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
