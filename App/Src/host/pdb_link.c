#include "host/pdb_link.h"
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
/* Hard-ESTOP GPIO -- PLACEHOLDER, confirm against schematic before hardware  */
/* bring-up (see docs/pdb-uart-v1.md open items). PA0 chosen only because it  */
/* is unclaimed anywhere else in this firmware today.                        */
/* -------------------------------------------------------------------------- */
#define PDB_ESTOP_GPIO_PORT GPIOA
#define PDB_ESTOP_GPIO_PIN  GPIO_PIN_0

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
#define PDB_RX_CHUNK_BYTES 64u

static uint8_t           s_rx_ring[PDB_RX_RING_BYTES];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static uint8_t           s_rx_chunk[PDB_RX_CHUNK_BYTES];

static uint8_t  s_rx_accum[PDB_FRAME_BYTES];
static uint16_t s_rx_fill;

static uint8_t  s_last_valid_fb[PDB_FRAME_BYTES];
static uint32_t s_last_rx_ms;
static bool     s_ever_synced;

static uint8_t           s_tx_frame[PDB_FRAME_BYTES];
static volatile bool     s_tx_busy;
static uint32_t          s_last_tx_ms;
static uint8_t           s_tx_seq;

static uint8_t  s_rail_enable_cmd;
static uint8_t  s_kill_request;   /* PDB_KILL_NORMAL / SOFT_REQ(ack) / SOFT_READY as sent */
static bool     s_estop_requested;
static uint8_t  s_tx_heartbeat;

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
	/* A corrupt/mismatched frame is treated as "no frame this cycle" -- it
	 * must NOT refresh s_last_rx_ms, so a stream of garbage degrades to the
	 * same fail-safe path as silence, not a false "link is fine". */
	if (!pdb_frame_valid(s_rx_accum)) {
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
	buf[PDB_OFF_FLAGS] = 0u;

	buf[PDB_CMD_OFF_RAIL_ENABLE] = s_rail_enable_cmd;
	buf[PDB_CMD_OFF_KILL_REQ] = s_kill_request;
	buf[PDB_CMD_OFF_HEARTBEAT] = s_tx_heartbeat++;

	crc = pdb_crc16(buf, PDB_OFF_CRC);
	buf[PDB_OFF_CRC] = (uint8_t)(crc & 0xFFu);
	buf[PDB_OFF_CRC + 1u] = (uint8_t)((crc >> 8) & 0xFFu);
}

static void pdb_service_tx(void)
{
	uint32_t now = HAL_GetTick();

	if (s_tx_busy)
		return;
	if (s_last_tx_ms != 0u && (now - s_last_tx_ms) < PDB_TX_PERIOD_MS)
		return;

	s_last_tx_ms = (now == 0u) ? 1u : now;
	pdb_build_cmd_frame(s_tx_frame);
	s_tx_busy = true;
	if (HAL_UART_Transmit_IT(&huart4, s_tx_frame, PDB_FRAME_BYTES) != HAL_OK)
		s_tx_busy = false;
}

/* -------------------------------------------------------------------------- */
/* Hard-ESTOP GPIO -- single owner. Active-low: LOW = asserted/power cut.     */
/* -------------------------------------------------------------------------- */
static void pdb_drive_estop_gpio(bool power_allowed)
{
	HAL_GPIO_WritePin(PDB_ESTOP_GPIO_PORT, PDB_ESTOP_GPIO_PIN,
	                  power_allowed ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */
void pdb_link_init(void)
{
	GPIO_InitTypeDef gi = {0};

	s_rx_head = 0u;
	s_rx_tail = 0u;
	s_rx_fill = 0u;
	memset(s_last_valid_fb, 0, sizeof(s_last_valid_fb));
	s_last_rx_ms = 0u;
	s_ever_synced = false;

	s_tx_busy = false;
	s_last_tx_ms = 0u;
	s_tx_seq = 0u;
	s_tx_heartbeat = 0u;

	s_rail_enable_cmd = 0u;
	s_kill_request = (uint8_t)PDB_KILL_NORMAL;
	s_estop_requested = false;

	/* Fail-safe default: asserted (LOW) before the link has ever synced.
	 * External pull-down on this net (schematic-dependent) should already
	 * hold it asserted before firmware runs at all -- this just keeps the
	 * MCU's own drive consistent with that from the first instruction. */
	gi.Pin = PDB_ESTOP_GPIO_PIN;
	gi.Mode = GPIO_MODE_OUTPUT_PP;
	gi.Pull = GPIO_PULLDOWN;
	gi.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(PDB_ESTOP_GPIO_PORT, &gi);
	pdb_drive_estop_gpio(false);

	(void)HAL_UARTEx_ReceiveToIdle_IT(&huart4, s_rx_chunk, sizeof(s_rx_chunk));
}

void pdb_link_service(void)
{
	bool fresh;

	pdb_service_rx();

	fresh = pdb_link_is_fresh();

	/* Fail-safe: link stale (including "never synced") or an explicit
	 * request -> assert. Otherwise allow power. This is the unconditional
	 * backstop -- it does not wait for the soft-kill handshake. */
	pdb_drive_estop_gpio(fresh && !s_estop_requested);

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

uint8_t pdb_link_kill_state(void)
{
	return pdb_link_fresh_byte(PDB_FB_OFF_KILL_STATE, (uint8_t)PDB_KILL_HARD_ESTOP);
}

uint8_t pdb_link_kill_reason(void)
{
	return pdb_link_fresh_byte(PDB_FB_OFF_KILL_REASON, (uint8_t)PDB_KILL_REASON_COMMS_LOSS);
}

uint8_t pdb_link_estop_sense(void)
{
	uint8_t value;

	plant_crit_enter();
	value = s_last_valid_fb[PDB_FB_OFF_ESTOP_SENSE];
	plant_crit_exit();
	return value;
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
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	if (huart->Instance != UART4)
		return;

	pdb_rx_push(s_rx_chunk, size);
	(void)HAL_UARTEx_ReceiveToIdle_IT(&huart4, s_rx_chunk, sizeof(s_rx_chunk));
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance != UART4)
		return;

	s_tx_busy = false;
}

#else /* UART4_MODE != UART4_MODE_PDB -- inert stubs so callers never need an #if */

void pdb_link_init(void) { }
void pdb_link_service(void) { }
void pdb_link_set_rail_enable_cmd(uint8_t mask) { (void)mask; }
void pdb_link_request_estop(bool assert) { (void)assert; }
void pdb_link_set_soft_kill_ready(bool ready) { (void)ready; }
bool pdb_link_is_fresh(void) { return false; }
uint8_t pdb_link_kill_state(void) { return (uint8_t)PDB_KILL_HARD_ESTOP; }
uint8_t pdb_link_kill_reason(void) { return (uint8_t)PDB_KILL_REASON_COMMS_LOSS; }
uint8_t pdb_link_estop_sense(void) { return 0u; }
void pdb_link_fill_mirror(uint8_t *out)
{
	if (out != NULL)
		memset(out, 0, PDB_FRAME_BYTES);
}

#endif /* UART4_MODE == UART4_MODE_PDB */
