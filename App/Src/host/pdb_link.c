#include "host/pdb_link.h"
#include "host/pdb_vi_limits.h"
#include "host/uart4_mode.h"
#include "usart.h"
#include "main.h"
#include "plant/plant_crit.h"
#include <string.h>

#if UART4_MODE == UART4_MODE_PDB

/* -------------------------------------------------------------------------- */
/* Wire format (docs/pdb-uart-v1.md) -- 64 B fixed, both directions          */
/* -------------------------------------------------------------------------- */

#define PDB_MAGIC_CMD 0x43424450u /* "PDBC" */
#define PDB_MAGIC_FB  0x46424450u /* "PDBF" */
#define PDB_VERSION   1u

#define PDB_OFF_MAGIC   0u
#define PDB_OFF_VERSION 4u
#define PDB_OFF_SEQ     5u
#define PDB_OFF_FLAGS   6u
#define PDB_OFF_PAYLOAD 8u
#define PDB_OFF_CRC     62u

/* Command payload (controls -> PDB), from PDB_OFF_PAYLOAD */
#define PDB_CMD_OFF_RAIL_ENABLE 8u
#define PDB_CMD_OFF_KILL_REQ    9u
#define PDB_CMD_OFF_HEARTBEAT   10u

/* Feedback payload (PDB -> controls), from PDB_OFF_PAYLOAD */
#define PDB_FB_OFF_PACK_V        8u  /* 4x u16 */
#define PDB_FB_OFF_RAIL_V        16u /* 4x u16 */
#define PDB_FB_OFF_PACK_I        24u /* 4x u16 */
#define PDB_FB_OFF_RAIL_I        32u /* 4x u16 */
#define PDB_FB_OFF_CONTACTOR     40u
#define PDB_FB_OFF_KILL_STATE    41u
#define PDB_FB_OFF_KILL_REASON   42u
#define PDB_FB_OFF_ESTOP_SENSE   43u
#define PDB_FB_OFF_FAULT_FLAGS   44u
#define PDB_FB_OFF_HB_ECHO       45u

#define PDB_STALE_MS     200u /* ~4 missed frames at a 20 Hz nominal rate */
#define PDB_TX_PERIOD_MS 20u  /* 50 Hz command rate */

/* -------------------------------------------------------------------------- */
/* Hard-ESTOP sense — PB7, PDU-driven (MCU must not drive). Active-low:       */
/* HIGH = power allowed, LOW = asserted. gpio.c + pdb_link_init: INPUT.       */
/* -------------------------------------------------------------------------- */
#define PDB_ESTOP_GPIO_PORT GPIOB
#define PDB_ESTOP_GPIO_PIN  GPIO_PIN_7

/* -------------------------------------------------------------------------- */
/* CRC16-CCITT (poly 0x1021, init 0xFFFF) -- bit-banged, not table-driven:    */
/* auditable and trivial for a PDB MCU on any toolchain to reproduce exactly. */
/* -------------------------------------------------------------------------- */
static uint16_t pdb_crc16(const uint8_t *data, uint32_t len)
{
	uint16_t crc = 0xFFFFu;

	for (uint32_t i = 0; i < len; i++) {
		crc ^= (uint16_t)((uint16_t)data[i] << 8);
		for (uint8_t bit = 0; bit < 8u; bit++) {
			if ((crc & 0x8000u) != 0u)
				crc = (uint16_t)((crc << 1) ^ 0x1021u);
			else
				crc = (uint16_t)(crc << 1);
		}
	}
	return crc;
}

/* -------------------------------------------------------------------------- */
/* RX: ISR-push ring buffer (contiguous memcpy, not byte-at-a-time) +         */
/* main-loop drain/resync -- same shape as host_transport_usb.c /            */
/* host_link.c, not host_transport_uart.c's byte-IT/blocking-TX pattern.      */
/* -------------------------------------------------------------------------- */
#define PDB_RX_RING_BYTES  256u

static uint8_t           s_rx_ring[PDB_RX_RING_BYTES];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static uint8_t           s_rx_byte; /* HAL_UART_Receive_IT single-byte buffer */

static uint8_t  s_rx_accum[PDB_FRAME_BYTES];
static uint16_t s_rx_fill;

static uint8_t  s_last_valid_fb[PDB_FRAME_BYTES];
static uint32_t s_last_rx_ms;
static bool     s_ever_synced;

static uint8_t           s_tx_frame[PDB_FRAME_BYTES];
static volatile bool     s_tx_busy;
static volatile bool     s_tx_active; /* mid paced frame */
static volatile uint8_t  s_tx_idx;
static uint32_t          s_last_tx_ms;
static uint8_t           s_tx_seq;

static uint8_t  s_rail_enable_cmd;
static uint8_t  s_kill_request;   /* PDB_KILL_NORMAL / SOFT_REQ(ack) / SOFT_READY as sent */
static bool     s_estop_requested;
static uint8_t  s_tx_heartbeat;

/* Snapshot of Cube MX_UART4_Init result (no second bring-up). */
static uint8_t  s_uart_clk_src;
static uint16_t s_uart_brr;
static uint32_t s_uart_kernel_hz;

/* Raw RX / error counters — prove Jetson TX reaches UART4 even when frames
 * fail CRC (mirror stays zero). Sticky error bits use HAL_UART_ERROR_*. */
static volatile uint32_t s_rx_bytes;
static volatile uint32_t s_rx_events;
static volatile uint32_t s_rx_valid;
static volatile uint32_t s_rx_crc_fail;
static volatile uint32_t s_rx_last_word;
static volatile uint32_t s_tx_complete;
static volatile uint32_t s_uart_err_sticky;

static void pdb_uart_rearm_rx(void)
{
	(void)HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1u);
}

static void pdb_rx_push(const uint8_t *data, uint32_t len)
{
	uint16_t head = s_rx_head;
	uint16_t tail = s_rx_tail;
	uint16_t free_bytes = (uint16_t)((tail + PDB_RX_RING_BYTES - head - 1u) % PDB_RX_RING_BYTES);
	uint16_t take = (uint16_t)len;
	uint16_t first;

	if (take > free_bytes)
		take = free_bytes;
	if (take == 0u)
		return;

	s_rx_bytes += take;

	first = (uint16_t)(PDB_RX_RING_BYTES - head);
	if (first > take)
		first = take;
	memcpy(&s_rx_ring[head], data, first);
	if (take > first)
		memcpy(&s_rx_ring[0], data + first, (size_t)(take - first));
	s_rx_head = (uint16_t)((head + take) % PDB_RX_RING_BYTES);
}

static uint16_t pdb_rx_drain(uint8_t *dst, uint16_t max_len)
{
	uint16_t head = s_rx_head;
	uint16_t tail = s_rx_tail;
	uint16_t avail = (uint16_t)((head + PDB_RX_RING_BYTES - tail) % PDB_RX_RING_BYTES);
	uint16_t first;

	if (avail == 0u)
		return 0u;
	if (max_len < avail)
		avail = max_len;

	first = (uint16_t)(PDB_RX_RING_BYTES - tail);
	if (first > avail)
		first = avail;
	memcpy(dst, &s_rx_ring[tail], first);
	if (avail > first)
		memcpy(dst + first, &s_rx_ring[0], (size_t)(avail - first));
	s_rx_tail = (uint16_t)((tail + avail) % PDB_RX_RING_BYTES);
	return avail;
}

static bool pdb_magic_at(const uint8_t *buf, uint32_t magic)
{
	return buf[0] == (uint8_t)(magic & 0xFFu) &&
	       buf[1] == (uint8_t)((magic >> 8) & 0xFFu) &&
	       buf[2] == (uint8_t)((magic >> 16) & 0xFFu) &&
	       buf[3] == (uint8_t)((magic >> 24) & 0xFFu);
}

/* Same shape as host_link_rx_resync(): scan for a valid magic starting at
 * offset 1 (offset 0 already failed), slide it to the front, keep any
 * partial bytes after it. Full miss -> drop everything. */
static void pdb_rx_resync(void)
{
	uint16_t shift = PDB_FRAME_BYTES;

	for (uint16_t i = 1; i < PDB_FRAME_BYTES; i++) {
		if (pdb_magic_at(&s_rx_accum[i], PDB_MAGIC_FB)) {
			shift = i;
			break;
		}
	}

	if (shift < PDB_FRAME_BYTES) {
		uint16_t remain = (uint16_t)(PDB_FRAME_BYTES - shift);

		memmove(s_rx_accum, &s_rx_accum[shift], remain);
		s_rx_fill = remain;
	} else {
		s_rx_fill = 0u;
	}
}

static uint16_t pdb_rd_u16(const uint8_t *buf, uint32_t off)
{
	return (uint16_t)(buf[off] | ((uint16_t)buf[off + 1u] << 8));
}

static bool pdb_frame_valid(const uint8_t *buf)
{
	uint16_t crc;

	if (!pdb_magic_at(buf, PDB_MAGIC_FB))
		return false;
	if (buf[PDB_OFF_VERSION] != PDB_VERSION)
		return false;
	crc = pdb_crc16(buf, PDB_OFF_CRC);
	return pdb_rd_u16(buf, PDB_OFF_CRC) == crc;
}

static void pdb_rx_consume_ready(void)
{
	s_rx_last_word = (uint32_t)s_rx_accum[0] |
	                 ((uint32_t)s_rx_accum[1] << 8) |
	                 ((uint32_t)s_rx_accum[2] << 16) |
	                 ((uint32_t)s_rx_accum[3] << 24);

	/* A corrupt/mismatched frame is treated as "no frame this cycle" -- it
	 * must NOT refresh s_last_rx_ms, so a stream of garbage degrades to the
	 * same fail-safe path as silence, not a false "link is fine". */
	if (!pdb_frame_valid(s_rx_accum)) {
		s_rx_crc_fail++;
		pdb_rx_resync();
		return;
	}

	/* Host and Plant tasks now run at the same priority (time-sliced, not
	 * strictly ordered) -- this write is read cross-task by pdb_link_kill_state/
	 * kill_reason/estop_sense/fill_mirror (called from PlantTask's
	 * host_link_poll_tx). Without this, a context switch mid-memcpy could hand
	 * the reader a torn frame. Same pattern as host_link.c's g_plant_pending_image. */
	plant_crit_enter();
	memcpy(s_last_valid_fb, s_rx_accum, PDB_FRAME_BYTES);
	s_last_rx_ms = HAL_GetTick();
	s_ever_synced = true;
	s_rx_valid++;
	plant_crit_exit();
	s_rx_fill = 0u;
}

static void pdb_service_rx(void)
{
	uint8_t chunk[64];
	uint16_t n;

	while ((n = pdb_rx_drain(chunk, sizeof(chunk))) > 0u) {
		uint16_t off = 0u;

		while (off < n) {
			uint16_t need = (uint16_t)(PDB_FRAME_BYTES - s_rx_fill);
			uint16_t take = (uint16_t)(n - off);

			if (take > need)
				take = need;
			memcpy(&s_rx_accum[s_rx_fill], &chunk[off], take);
			s_rx_fill = (uint16_t)(s_rx_fill + take);
			off = (uint16_t)(off + take);

			if (s_rx_fill >= PDB_FRAME_BYTES)
				pdb_rx_consume_ready();
		}
	}
}

/* -------------------------------------------------------------------------- */
/* TX: non-blocking, rate-limited to PDB_TX_PERIOD_MS                         */
/* -------------------------------------------------------------------------- */
static void pdb_build_cmd_frame(uint8_t *buf)
{
	uint16_t crc;

	memset(buf, 0, PDB_FRAME_BYTES);
	buf[0] = (uint8_t)(PDB_MAGIC_CMD & 0xFFu);
	buf[1] = (uint8_t)((PDB_MAGIC_CMD >> 8) & 0xFFu);
	buf[2] = (uint8_t)((PDB_MAGIC_CMD >> 16) & 0xFFu);
	buf[3] = (uint8_t)((PDB_MAGIC_CMD >> 24) & 0xFFu);
	buf[PDB_OFF_VERSION] = PDB_VERSION;
	buf[PDB_OFF_SEQ] = s_tx_seq++;
	/* bit0: host/local ESTOP request latch (PDU still owns hard wire on PB7). */
	buf[PDB_OFF_FLAGS] = s_estop_requested ? 0x01u : 0u;

	buf[PDB_CMD_OFF_RAIL_ENABLE] = s_rail_enable_cmd;
	buf[PDB_CMD_OFF_KILL_REQ] = s_kill_request;
	buf[PDB_CMD_OFF_HEARTBEAT] = s_tx_heartbeat++;

	crc = pdb_crc16(buf, PDB_OFF_CRC);
	buf[PDB_OFF_CRC] = (uint8_t)(crc & 0xFFu);
	buf[PDB_OFF_CRC + 1u] = (uint8_t)((crc >> 8) & 0xFFu);
}

static void pdb_tx_kick_byte(void)
{
	if (s_tx_idx >= PDB_FRAME_BYTES) {
		s_tx_active = false;
		return;
	}
	s_tx_busy = true;
	if (HAL_UART_Transmit_IT(&huart4, &s_tx_frame[s_tx_idx], 1u) != HAL_OK)
		s_tx_busy = false;
}

static void pdb_service_tx(void)
{
	uint32_t now = HAL_GetTick();

#if UART4_TX_PACE_BYTES
	/* Continue paced frame: one byte per service call (~1 ms HostTask). */
	if (s_tx_active) {
		if (!s_tx_busy)
			pdb_tx_kick_byte();
		return;
	}
	if (s_last_tx_ms != 0u && (now - s_last_tx_ms) < PDB_TX_PERIOD_MS)
		return;
	s_last_tx_ms = (now == 0u) ? 1u : now;
	pdb_build_cmd_frame(s_tx_frame);
	s_tx_idx = 0u;
	s_tx_active = true;
	pdb_tx_kick_byte();
#else
	if (s_tx_busy)
		return;
	if (s_last_tx_ms != 0u && (now - s_last_tx_ms) < PDB_TX_PERIOD_MS)
		return;
	s_last_tx_ms = (now == 0u) ? 1u : now;

	pdb_build_cmd_frame(s_tx_frame);
	s_tx_busy = true;
	if (HAL_UART_Transmit_IT(&huart4, s_tx_frame, PDB_FRAME_BYTES) != HAL_OK)
		s_tx_busy = false;
#endif
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */
void pdb_link_init(void)
{
	GPIO_InitTypeDef gi = {0};
	uint32_t sel;

	s_rx_head = 0u;
	s_rx_tail = 0u;
	s_rx_fill = 0u;
	memset(s_last_valid_fb, 0, sizeof(s_last_valid_fb));
	s_last_rx_ms = 0u;
	s_ever_synced = false;

	s_tx_busy = false;
	s_tx_active = false;
	s_tx_idx = 0u;
	s_last_tx_ms = 0u;
	s_tx_seq = 0u;
	s_tx_heartbeat = 0u;

	s_rail_enable_cmd = 0u;
	s_kill_request = (uint8_t)PDB_KILL_NORMAL;
	s_estop_requested = false;

	/* PDU drives hard-ESTOP — high-Z input only (no MCU drive on this net).
	 * Pull-up: open/undriven = released (1); PDU asserts by driving LOW. */
	gi.Pin = PDB_ESTOP_GPIO_PIN;
	gi.Mode = GPIO_MODE_INPUT;
	gi.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(PDB_ESTOP_GPIO_PORT, &gi);

	/* Use MX_UART4_Init as-is (PCLK1, 115200 8N1, no invert). Snapshot only. */
	s_uart_brr = (uint16_t)(huart4.Instance->BRR & 0xFFFFu);
	sel = (RCC->CCIPR & RCC_CCIPR_UART4SEL_Msk) >> RCC_CCIPR_UART4SEL_Pos;
	s_uart_clk_src = (sel == 2u) ? 1u : (sel == 0u) ? 2u : (uint8_t)(10u + sel);
	if (sel == 2u)
		s_uart_kernel_hz = HSI_VALUE;
	else if (sel == 0u)
		s_uart_kernel_hz = HAL_RCC_GetPCLK1Freq();
	else
		s_uart_kernel_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_UART4);

	s_rx_bytes = 0u;
	s_rx_events = 0u;
	s_rx_valid = 0u;
	s_rx_crc_fail = 0u;
	s_rx_last_word = 0u;
	s_tx_complete = 0u;
	s_uart_err_sticky = 0u;

	pdb_uart_rearm_rx();

#if UART4_LOCAL_LOOPBACK_TEST
	/* Blocking one-shot: proves MX_UART4 + RX IT + framing without Jetson.
	 * Short PC10↔PC11 at the Controls connector, reset, read CDC last_rx. */
	{
		uint8_t frame[PDB_FRAME_BYTES];
		uint32_t t0;
		pdb_build_cmd_frame(frame);
		(void)HAL_UART_AbortReceive(&huart4);
		if (HAL_UART_Transmit(&huart4, frame, PDB_FRAME_BYTES, 50u) == HAL_OK) {
			if (HAL_UART_Receive(&huart4, s_rx_accum, PDB_FRAME_BYTES, 50u) ==
			    HAL_OK) {
				s_rx_bytes += PDB_FRAME_BYTES;
				s_rx_events += PDB_FRAME_BYTES;
				s_rx_last_word = (uint32_t)s_rx_accum[0] |
				                 ((uint32_t)s_rx_accum[1] << 8) |
				                 ((uint32_t)s_rx_accum[2] << 16) |
				                 ((uint32_t)s_rx_accum[3] << 24);
				if (pdb_frame_valid(s_rx_accum) ||
				    pdb_magic_at(s_rx_accum, PDB_MAGIC_CMD)) {
					/* CMD magic on loopback — accept as pass. */
					s_rx_valid++;
					memcpy(s_last_valid_fb, s_rx_accum, PDB_FRAME_BYTES);
					/* Mirror expects PDBF; keep raw for CDC last_rx. */
				} else {
					s_rx_crc_fail++;
				}
			} else {
				s_uart_err_sticky |= 0x8u; /* mark timeout/fail */
			}
		}
		t0 = HAL_GetTick();
		while ((HAL_GetTick() - t0) < 2u) {
		}
		pdb_uart_rearm_rx();
	}
#endif
}

uint8_t pdb_link_uart_clk_src(void)
{
	return s_uart_clk_src;
}

uint16_t pdb_link_uart_brr(void)
{
	return s_uart_brr;
}

uint32_t pdb_link_uart_kernel_hz(void)
{
	return s_uart_kernel_hz;
}

uint32_t pdb_link_rx_byte_count(void)
{
	return s_rx_bytes;
}

uint32_t pdb_link_rx_event_count(void)
{
	return s_rx_events;
}

uint32_t pdb_link_rx_valid_count(void)
{
	return s_rx_valid;
}

uint32_t pdb_link_uart_err_sticky(void)
{
	return s_uart_err_sticky;
}

uint32_t pdb_link_rx_crc_fail_count(void)
{
	return s_rx_crc_fail;
}

uint32_t pdb_link_rx_last_word(void)
{
	return s_rx_last_word;
}

uint32_t pdb_link_tx_complete_count(void)
{
	return s_tx_complete;
}

uint32_t pdb_link_uart_cr1(void)
{
	return huart4.Instance->CR1;
}

uint32_t pdb_link_uart_cr2(void)
{
	return huart4.Instance->CR2;
}

uint32_t pdb_link_uart_cr3(void)
{
	return huart4.Instance->CR3;
}

void pdb_link_service(void)
{
	pdb_service_rx();
	pdb_service_tx();
}

void pdb_link_set_rail_enable_cmd(uint8_t mask)
{
	s_rail_enable_cmd = mask;
}

void pdb_link_request_estop(bool assert)
{
	s_estop_requested = assert;
}

void pdb_link_set_soft_kill_ready(bool ready)
{
	s_kill_request = ready ? (uint8_t)PDB_KILL_SOFT_READY : (uint8_t)PDB_KILL_NORMAL;
}

bool pdb_link_is_fresh(void)
{
	bool synced;
	uint32_t last_ms;

	plant_crit_enter();
	synced = s_ever_synced;
	last_ms = s_last_rx_ms;
	plant_crit_exit();

	return synced && (HAL_GetTick() - last_ms) <= PDB_STALE_MS;
}

bool pdb_link_ever_synced(void)
{
	bool synced;

	plant_crit_enter();
	synced = s_ever_synced;
	plant_crit_exit();
	return synced;
}

/* Snapshot freshness + the one status byte together so a Host-task update
 * landing between the two reads can't hand back a freshness verdict paired
 * with a byte from a different frame. */
static uint8_t pdb_link_fresh_byte(uint32_t offset, uint8_t stale_value)
{
	bool synced;
	uint32_t last_ms;
	uint8_t value;

	plant_crit_enter();
	synced = s_ever_synced;
	last_ms = s_last_rx_ms;
	value = s_last_valid_fb[offset];
	plant_crit_exit();

	return (synced && (HAL_GetTick() - last_ms) <= PDB_STALE_MS) ? value : stale_value;
}

/* V/I acceptability on a validated fresh PDBF payload.
 * Returns PDB_KILL_REASON_NONE if OK, else UNDERVOLTAGE or OVERCURRENT.
 * OC wins when both would trip. Unused pack slots (v==0) are skipped.
 * Only rail_v[0] (central 48 V) has a voltage window; other rails are
 * current-checked when active (contactor bit or non-zero voltage). */
static uint8_t pdb_vi_reject_reason(const uint8_t *fb)
{
	uint8_t contactor = fb[PDB_FB_OFF_CONTACTOR];
	uint8_t reason = (uint8_t)PDB_KILL_REASON_NONE;
	uint8_t i;

	for (i = 0u; i < 4u; i++) {
		uint16_t pv = pdb_rd_u16(fb, PDB_FB_OFF_PACK_V + (uint32_t)i * 2u);
		uint16_t pi = pdb_rd_u16(fb, PDB_FB_OFF_PACK_I + (uint32_t)i * 2u);
		uint16_t rv = pdb_rd_u16(fb, PDB_FB_OFF_RAIL_V + (uint32_t)i * 2u);
		uint16_t ri = pdb_rd_u16(fb, PDB_FB_OFF_RAIL_I + (uint32_t)i * 2u);
		bool pack_on = (pv != 0u);
		bool rail_on = ((contactor & (uint8_t)(1u << i)) != 0u) || (rv != 0u);

		if (pack_on && pi > PDB_VI_I_ABS_MAX_COUNTS)
			return (uint8_t)PDB_KILL_REASON_OVERCURRENT;
		if (rail_on && ri > PDB_VI_I_ABS_MAX_COUNTS)
			return (uint8_t)PDB_KILL_REASON_OVERCURRENT;

		if (pack_on &&
		    (pv < PDB_VI_PACK_V_MIN_COUNTS || pv > PDB_VI_PACK_V_MAX_COUNTS))
			reason = (uint8_t)PDB_KILL_REASON_UNDERVOLTAGE;

		if (i == 0u && rail_on &&
		    (rv < PDB_VI_RAIL48_V_MIN_COUNTS || rv > PDB_VI_RAIL48_V_MAX_COUNTS))
			reason = (uint8_t)PDB_KILL_REASON_UNDERVOLTAGE;
	}

	return reason;
}

/* Freshness + peer kill + V/I overlay in one snapshot so USB system.kill_state
 * and kill_reason always agree. Stale still → HARD + COMMS_LOSS. Overlay only
 * when peer reports NORMAL (do not demote HARD / SOFT_READY / peer SOFT_REQ). */
static void pdb_link_eval_kill(uint8_t *state_out, uint8_t *reason_out)
{
	uint8_t fb[PDB_FRAME_BYTES];
	bool synced;
	uint32_t last_ms;
	uint8_t peer_state;
	uint8_t peer_reason;
	uint8_t vi_reason;

	plant_crit_enter();
	synced = s_ever_synced;
	last_ms = s_last_rx_ms;
	memcpy(fb, s_last_valid_fb, PDB_FRAME_BYTES);
	plant_crit_exit();

	if (!synced || (HAL_GetTick() - last_ms) > PDB_STALE_MS) {
#if PDB_STALE_FAILSAFE
		*state_out = (uint8_t)PDB_KILL_HARD_ESTOP;
		*reason_out = (uint8_t)PDB_KILL_REASON_COMMS_LOSS;
#else
		/* Bench: no PDU peer — do not latch HARD on USB/LED traffic-light. */
		*state_out = (uint8_t)PDB_KILL_NORMAL;
		*reason_out = (uint8_t)PDB_KILL_REASON_NONE;
#endif
		return;
	}

	peer_state = fb[PDB_FB_OFF_KILL_STATE];
	peer_reason = fb[PDB_FB_OFF_KILL_REASON];

	if (peer_state != (uint8_t)PDB_KILL_NORMAL) {
		*state_out = peer_state;
		*reason_out = peer_reason;
		return;
	}

	vi_reason = pdb_vi_reject_reason(fb);
	if (vi_reason != (uint8_t)PDB_KILL_REASON_NONE) {
		*state_out = (uint8_t)PDB_KILL_SOFT_REQ;
		*reason_out = vi_reason;
		return;
	}

	*state_out = peer_state;
	*reason_out = peer_reason;
}

uint8_t pdb_link_kill_state(void)
{
	uint8_t state;
	uint8_t reason;

	pdb_link_eval_kill(&state, &reason);
	(void)reason;
	return state;
}

uint8_t pdb_link_kill_reason(void)
{
	uint8_t state;
	uint8_t reason;

	pdb_link_eval_kill(&state, &reason);
	(void)state;
	return reason;
}

uint8_t pdb_link_estop_sense(void)
{
	/* Local wire sense (PDU-driven PB7). 1 = HIGH / power allowed, 0 = LOW /
	 * asserted — same polarity as the PDBF estop_sense field. */
	return (HAL_GPIO_ReadPin(PDB_ESTOP_GPIO_PORT, PDB_ESTOP_GPIO_PIN) ==
	        GPIO_PIN_SET)
	           ? 1u
	           : 0u;
}

uint8_t pdb_link_peer_estop_sense(void)
{
	/* Stale default 1: do not double-fault LEDs; kill_state already → HARD. */
	return pdb_link_fresh_byte(PDB_FB_OFF_ESTOP_SENSE, 1u);
}

void pdb_link_fill_mirror(uint8_t *out)
{
	if (out == NULL)
		return;
	plant_crit_enter();
	memcpy(out, s_last_valid_fb, PDB_FRAME_BYTES);
	plant_crit_exit();
}

/* -------------------------------------------------------------------------- */
/* HAL callbacks -- only compiled when UART4_MODE_PDB is selected, so there   */
/* is no duplicate-symbol clash with host_transport_uart.c / host_uart_bridge.c */
/* -------------------------------------------------------------------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance != UART4)
		return;

	s_rx_events++;
	pdb_rx_push(&s_rx_byte, 1u);
	pdb_uart_rearm_rx();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance != UART4)
		return;

	s_tx_busy = false;
	s_tx_complete++;
#if UART4_TX_PACE_BYTES
	if (s_tx_idx < PDB_FRAME_BYTES)
		s_tx_idx++;
	if (s_tx_idx >= PDB_FRAME_BYTES)
		s_tx_active = false;
#endif
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance != UART4)
		return;

	s_uart_err_sticky |= HAL_UART_GetError(huart);
	__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF |
	                               UART_CLEAR_NEF | UART_CLEAR_OREF);
	huart->ErrorCode = HAL_UART_ERROR_NONE;
	pdb_uart_rearm_rx();
}

#else /* UART4_MODE != UART4_MODE_PDB -- inert stubs so callers never need an #if */

void pdb_link_init(void) { }
void pdb_link_service(void) { }
void pdb_link_set_rail_enable_cmd(uint8_t mask) { (void)mask; }
void pdb_link_request_estop(bool assert) { (void)assert; }
void pdb_link_set_soft_kill_ready(bool ready) { (void)ready; }
bool pdb_link_is_fresh(void) { return false; }
bool pdb_link_ever_synced(void) { return false; }
uint8_t pdb_link_kill_state(void) { return (uint8_t)PDB_KILL_HARD_ESTOP; }
uint8_t pdb_link_kill_reason(void) { return (uint8_t)PDB_KILL_REASON_COMMS_LOSS; }
uint8_t pdb_link_estop_sense(void) { return 0u; }
uint8_t pdb_link_peer_estop_sense(void) { return 1u; }
void pdb_link_fill_mirror(uint8_t *out)
{
	if (out != NULL)
		memset(out, 0, PDB_FRAME_BYTES);
}
uint8_t pdb_link_uart_clk_src(void) { return 0u; }
uint16_t pdb_link_uart_brr(void) { return 0u; }
uint32_t pdb_link_uart_kernel_hz(void) { return 0u; }
uint32_t pdb_link_rx_byte_count(void) { return 0u; }
uint32_t pdb_link_rx_event_count(void) { return 0u; }
uint32_t pdb_link_rx_valid_count(void) { return 0u; }
uint32_t pdb_link_uart_err_sticky(void) { return 0u; }
uint32_t pdb_link_rx_crc_fail_count(void) { return 0u; }
uint32_t pdb_link_rx_last_word(void) { return 0u; }
uint32_t pdb_link_tx_complete_count(void) { return 0u; }

#endif /* UART4_MODE == UART4_MODE_PDB */
