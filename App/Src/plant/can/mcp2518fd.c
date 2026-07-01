#include "plant/can/mcp2518fd.h"
#include "plant/can/spi_can_port.h"
#include "plant/can/can_frame.h"

#include "main.h"
#include <string.h>

#define MCP_SPI_READ   0x03u
#define MCP_SPI_WRITE  0x02u

#define REG_C1CON      0x000u
#define REG_C1NBTCFG   0x004u
#define REG_C1TREC     0x034u
#define REG_C1BDIAG1   0x038u
#define REG_C1INT      0x01Cu
#define REG_C1FLTCON0  0x1D0u
#define REG_C1FLTOBJ0  0x1F0u
#define REG_C1MASK0    0x1F4u
#define REG_OSC        0xE00u
#define REG_IOCON      0xE04u
#define REG_ECCCON     0xE0Cu

#define C1INT_RXIE     (1u << 17)
#define IOCON_INTOD    (1u << 30)

#define MCP_RAM_SIZE      2048u

/* MCP2518FD: 0x050 = CiTXQCON (TX Queue). CiFIFOCON1 @ 0x05C = FIFO 1. */
#define MCP_RX_FIFO    1u

#define MCP_TXQ_CON    0x050u
#define MCP_TXQ_STA    0x054u
#define MCP_TXQ_UA     0x058u

#define MCP_FIFO_CON(fifo)  (0x050u + (uint16_t)(fifo) * 12u)
#define MCP_FIFO_STA(fifo)  (0x054u + (uint16_t)(fifo) * 12u)
#define MCP_FIFO_UA(fifo)   (0x058u + (uint16_t)(fifo) * 12u)

/* CiFIFOCON / CiTXQCON bits */
#define MCP_FIFO_TFNRFNIE (1u << 0)  /* CON: not-full/not-empty IRQ enable */
#define MCP_FIFO_TXEN     (1u << 7)  /* CON: 1=TX FIFO (not used on TXQ) */
#define MCP_FIFO_UINC     (1u << 8)  /* CON: advance head/tail pointer */
#define MCP_FIFO_TXREQ    (1u << 9)  /* CON: request TX */
#define MCP_FIFO_FRESET   (1u << 10) /* CON: FIFO/TXQ reset (config mode) */
/* CiFIFOSTA / CiTXQSTA: bit0 = not-full (TX) / not-empty (RX); bit2 = empty (TX). */
#define MCP_FIFO_TFNRFNIF (1u << 0)
#define MCP_FIFO_TXEMPTY  (1u << 2)
#define MCP_TXQ_STA_EMPTY (1u << 2) /* CiTXQSTA.TXQEIF */
#define MCP_TXQ_STA_DONE  (0xF0u) /* TXABT|TXLARB|TXERR|TXATIF */

#define OSC_PLLEN         (1u << 0)
#define OSC_OSCDIS        (1u << 2)
#define OSC_SCLKDIV       (1u << 5)
#define OSC_OSCREADY      (1u << 10)

/* TXQ: 1 slot × 8 B; RX FIFO1: 4 slots × 8 B. */
#define MCP_TXQ_1X8       ((0u << 29) | (0u << 24) | MCP_FIFO_TXEN)
#define MCP_RX_FIFO_4X8   ((0u << 29) | (3u << 24))

#define MCP_RAM_BASE      0x400u  /* CiFIFOUA holds a RAM offset; add this for SFR addr (Eq. 4-1). */


#define C1CON_REQOP(n)   (((uint32_t)(n) & 0x7u) << 24)
#define C1CON_REQOP_MASK (0x7u << 24)
#define C1CON_TXQEN      (1u << 20)
#define C1CON_RTXAT      (1u << 16) /* honour CiTXQCON.TXAT (single-shot when TXAT=00) */
#define C1CON_BRSDIS     (1u << 12) /* classic CAN 2.0 only */
#define C1CON_ABAT       (1u << 29)
#define C1CON_OPMOD_SHIFT 21u
#define C1CON_OPMOD_CONFIG 4u
#define C1CON_OPMOD_CAN20  6u

typedef struct {
	uint8_t rail;
	bool initialized;
	volatile bool rx_irq_pending;
	uint8_t tx_ok;
	uint8_t tx_fail;
} mcp2518_dev_t;

static mcp2518_dev_t g_dev[MCP2518_RAIL_COUNT];

static bool mcp_read_buf(mcp2518_dev_t *d, uint16_t addr, uint8_t *out, uint16_t len)
{
	if (out == NULL || len == 0u || len > 32u)
		return false;

	uint8_t tx[2 + 32];
	uint8_t rx[2 + 32];

	/* MCP2518FD SPI header: Byte0 = (CMD[3:0]<<4) | ADDR[11:8], Byte1 = ADDR[7:0]. */
	tx[0] = (MCP_SPI_READ << 4) | (uint8_t)((addr >> 8) & 0x0Fu);
	tx[1] = (uint8_t)addr;
	memset(&tx[2], 0, len);

	if (!spi_can_port_xfer(d->rail, tx, rx, (uint16_t)(2u + len)))
		return false;

	memcpy(out, &rx[2], len);
	return true;
}

static bool mcp_write_buf(mcp2518_dev_t *d, uint16_t addr, const uint8_t *data, uint16_t len)
{
	if (data == NULL || len == 0u || len > 32u)
		return false;

	uint8_t tx[2 + 32];

	tx[0] = (MCP_SPI_WRITE << 4) | (uint8_t)((addr >> 8) & 0x0Fu);
	tx[1] = (uint8_t)addr;
	memcpy(&tx[2], data, len);

	return spi_can_port_xfer(d->rail, tx, NULL, (uint16_t)(2u + len));
}

static uint32_t mcp_read32(mcp2518_dev_t *d, uint16_t addr)
{
	uint8_t b[4];

	if (!mcp_read_buf(d, addr, b, 4u))
		return 0xFFFFFFFFu;

	return (uint32_t)b[0] |
	       ((uint32_t)b[1] << 8) |
	       ((uint32_t)b[2] << 16) |
	       ((uint32_t)b[3] << 24);
}

static void mcp_write32(mcp2518_dev_t *d, uint16_t addr, uint32_t val)
{
	uint8_t b[4] = {
		(uint8_t)(val),
		(uint8_t)(val >> 8),
		(uint8_t)(val >> 16),
		(uint8_t)(val >> 24),
	};

	(void)mcp_write_buf(d, addr, b, 4u);
}

static void mcp_reset(mcp2518_dev_t *d)
{
	uint8_t tx[2] = { 0x00u, 0x00u };

	(void)spi_can_port_xfer(d->rail, tx, NULL, 2);
	HAL_Delay(5);
}

static bool mcp_wait_osc_ready(mcp2518_dev_t *d)
{
	for (int i = 0; i < 200; i++) {
		uint32_t osc = mcp_read32(d, REG_OSC);

		if ((osc & OSC_OSCREADY) != 0u)
			return true;
		HAL_Delay(1);
	}
	return false;
}

static void mcp_ecc_enable(mcp2518_dev_t *d)
{
	uint32_t ecc = mcp_read32(d, REG_ECCCON);

	ecc |= 0x01u;
	mcp_write32(d, REG_ECCCON, ecc);
}

static void mcp_ram_init(mcp2518_dev_t *d)
{
	uint8_t fill[32];

	memset(fill, 0xFF, sizeof(fill));

	for (uint16_t off = 0u; off < MCP_RAM_SIZE; off += 32u)
		(void)mcp_write_buf(d, (uint16_t)(MCP_RAM_BASE + off), fill, 32u);
}

static bool mcp_wait_opmod(mcp2518_dev_t *d, uint32_t want)
{
	for (int i = 0; i < 200; i++) {
		uint32_t opmod = (mcp_read32(d, REG_C1CON) >> C1CON_OPMOD_SHIFT) & 0x07u;
		if (opmod == want)
			return true;
		HAL_Delay(1);
	}
	return false;
}

static bool mcp_enter_config(mcp2518_dev_t *d)
{
	uint32_t con = mcp_read32(d, REG_C1CON);

	con &= ~C1CON_REQOP_MASK;
	con |= C1CON_REQOP(4u);
	mcp_write32(d, REG_C1CON, con);
	return mcp_wait_opmod(d, 4u);
}

static bool mcp_enter_normal(mcp2518_dev_t *d)
{
	uint32_t con = mcp_read32(d, REG_C1CON);

	con &= ~C1CON_REQOP_MASK;
	/* REQOP=110 → Normal Classic CAN 2.0 (RobStride uses 1 Mbps classic). */
	con |= C1CON_REQOP(C1CON_OPMOD_CAN20);
	mcp_write32(d, REG_C1CON, con);
	return mcp_wait_opmod(d, C1CON_OPMOD_CAN20);
}

static bool mcp_recover_bus_off(mcp2518_dev_t *d)
{
	if (!d->initialized)
		return false;

	if (!mcp_enter_config(d))
		return false;

	uint32_t con = mcp_read32(d, REG_C1CON);
	con |= C1CON_ABAT;
	mcp_write32(d, REG_C1CON, con);
	HAL_Delay(2);

	return mcp_enter_normal(d);
}

static void mcp_config_osc_20mhz(mcp2518_dev_t *d)
{
	uint32_t osc = mcp_read32(d, REG_OSC);

	/* 20 MHz crystal: SYSCLK = XTAL (PLLEN=0). PLL is for 4 MHz × 10 only. */
	osc &= ~OSC_PLLEN;
	osc &= ~OSC_SCLKDIV;
	osc &= ~OSC_OSCDIS;
	mcp_write32(d, REG_OSC, osc);
	(void)mcp_wait_osc_ready(d);
}

static void mcp_config_bitrate(mcp2518_dev_t *d)
{
	(void)MCP2518_SYSCLK_HZ;
	(void)MCP2518_CAN_BAUD;

	/* 20 MHz / 20 TQ = 1 Mbps. Sample point 18/20 ≈ 90% (FDCAN CH1 is 15/17 ≈ 88%). */
	const uint32_t nbtcfg = (2u << 24) | (17u << 16) | (2u << 8) | 0u;
	mcp_write32(d, REG_C1NBTCFG, nbtcfg);
}

static void mcp_config_c1con_classic(mcp2518_dev_t *d)
{
	uint32_t con = mcp_read32(d, REG_C1CON);

	con |= C1CON_BRSDIS | C1CON_RTXAT;
	mcp_write32(d, REG_C1CON, con);
}

/* Reserve TXQ RAM and configure TX Queue (Microchip init order: TXQ then FIFO1..N). */
static void mcp_enable_txq(mcp2518_dev_t *d)
{
	uint32_t con = mcp_read32(d, REG_C1CON);

	con |= C1CON_TXQEN;
	mcp_write32(d, REG_C1CON, con);
}

static void mcp_config_txq(mcp2518_dev_t *d)
{
	uint32_t txqcon = MCP_TXQ_1X8 | MCP_FIFO_FRESET;

	mcp_write32(d, MCP_TXQ_CON, txqcon);
	txqcon = MCP_TXQ_1X8;
	mcp_write32(d, MCP_TXQ_CON, txqcon);
}

static void mcp_read_trec(mcp2518_dev_t *d, uint8_t *tec, uint8_t *rec)
{
	uint32_t trec = mcp_read32(d, REG_C1TREC);

	if (tec != NULL)
		*tec = (uint8_t)((trec >> 8) & 0xFFu);
	if (rec != NULL)
		*rec = (uint8_t)(trec & 0xFFu);
}

static void mcp_config_rx_fifo(mcp2518_dev_t *d)
{
	uint32_t rxcon = MCP_RX_FIFO_4X8 | MCP_FIFO_TFNRFNIE | MCP_FIFO_FRESET;

	mcp_write32(d, MCP_FIFO_CON(MCP_RX_FIFO), rxcon);
	rxcon = MCP_RX_FIFO_4X8 | MCP_FIFO_TFNRFNIE;
	mcp_write32(d, MCP_FIFO_CON(MCP_RX_FIFO), rxcon);
}

static void mcp_txq_uinc(mcp2518_dev_t *d)
{
	uint8_t con_b1 = 0x01u;

	(void)mcp_write_buf(d, (uint16_t)(MCP_TXQ_CON + 1u), &con_b1, 1u);
}

/* Dequeue completed/failed TXQ head — required after RTXAT single-shot NACK. */
static void mcp_txq_service(mcp2518_dev_t *d)
{
	for (int i = 0; i < 20; i++) {
		uint8_t sta = 0u;

		if (!mcp_read_buf(d, MCP_TXQ_STA, &sta, 1u))
			break;

		if ((sta & MCP_TXQ_STA_EMPTY) != 0u)
			break;

		if ((sta & MCP_TXQ_STA_DONE) != 0u) {
			mcp_txq_uinc(d);
			HAL_Delay(1);
			continue;
		}

		uint8_t con_b1 = 0u;

		if (mcp_read_buf(d, (uint16_t)(MCP_TXQ_CON + 1u), &con_b1, 1u) &&
		    ((con_b1 & 0x02u) != 0u)) {
			HAL_Delay(1);
			continue;
		}

		break;
	}
}

static bool mcp_txq_wait_done(mcp2518_dev_t *d, uint16_t timeout_ms)
{
	for (uint16_t t = 0; t < timeout_ms; t++) {
		uint8_t sta = 0u;

		if (!mcp_read_buf(d, MCP_TXQ_STA, &sta, 1u))
			return false;

		if ((sta & MCP_TXQ_STA_EMPTY) != 0u)
			return true;

		if ((sta & MCP_TXQ_STA_DONE) != 0u) {
			mcp_txq_uinc(d);
			return true;
		}

		HAL_Delay(1);
	}

	mcp_txq_service(d);
	return false;
}

static bool mcp_txq_has_space(mcp2518_dev_t *d)
{
	mcp_txq_service(d);

	uint8_t sta = 0u;

	if (!mcp_read_buf(d, MCP_TXQ_STA, &sta, 1u))
		return false;

	return ((sta & MCP_FIFO_TFNRFNIF) != 0u) || ((sta & MCP_TXQ_STA_EMPTY) != 0u);
}

static void mcp_txq_request(mcp2518_dev_t *d)
{
	/* Microchip API writes byte1 only: UINC|TXREQ (DS20005678 §4.16). */
	uint8_t con_b1 = 0x03u;

	(void)mcp_write_buf(d, (uint16_t)(MCP_TXQ_CON + 1u), &con_b1, 1u);
}

static void mcp_config_filters(mcp2518_dev_t *d)
{
	/* Accept-all (std + ext) → RX FIFO 1 — Microchip pattern: object/mask 0, FLTEN|FBP. */
	mcp_write32(d, REG_C1FLTCON0, 0u);
	mcp_write32(d, REG_C1FLTOBJ0, 0u);
	mcp_write32(d, REG_C1MASK0, 0u);
	mcp_write32(d, REG_C1FLTCON0, 0x80u | MCP_RX_FIFO);
}

static void mcp_config_iocon(mcp2518_dev_t *d)
{
	uint32_t iocon = mcp_read32(d, REG_IOCON);

	iocon |= IOCON_INTOD;
	mcp_write32(d, REG_IOCON, iocon);
}

static void mcp_enable_module_irq(mcp2518_dev_t *d)
{
	uint32_t ciint = mcp_read32(d, REG_C1INT);

	ciint |= C1INT_RXIE;
	mcp_write32(d, REG_C1INT, ciint);
}

static bool mcp_rx_fifo_not_empty(mcp2518_dev_t *d)
{
	uint8_t sta = 0u;

	if (!mcp_read_buf(d, MCP_FIFO_STA(MCP_RX_FIFO), &sta, 1u))
		return false;

	return (sta & MCP_FIFO_TFNRFNIF) != 0u;
}

static void mcp_rx_fifo_pop(mcp2518_dev_t *d)
{
	uint8_t con_b1 = 0x01u; /* UINC in CiFIFOCON byte 1 */

	(void)mcp_write_buf(d, (uint16_t)(MCP_FIFO_CON(MCP_RX_FIFO) + 1u), &con_b1, 1u);
}

static uint32_t mcp_pack_t0(uint32_t id, bool ext)
{
	if (!ext)
		return id & 0x7FFu;

	uint32_t sid = (id >> 18) & 0x7FFu;
	uint32_t eid = id & 0x3FFFFu;
	return (eid << 11) | sid;
}

static uint32_t mcp_pack_t1(uint8_t dlc, bool ext)
{
	uint32_t t1 = (uint32_t)(dlc & 0xFu);
	if (ext)
		t1 |= (1u << 4);
	return t1;
}

static void mcp_unpack_to_frame(uint32_t r0, uint32_t r1,
								uint32_t r2, uint32_t r3,
								can_frame_t *frame)
{
	bool ext = ((r1 >> 4) & 1u) != 0u;
	if (ext) {
		uint32_t sid = r0 & 0x7FFu;
		uint32_t eid = (r0 >> 11) & 0x3FFFFu;
		frame->id = (sid <<18) | eid;
		frame->id_type = CAN_ID_EXT;
	} else {
		frame->id = r0 & 0x7FFu;
		frame->id_type = CAN_ID_STD;
	}

	frame->dlc = CAN_MAX_DATA_LEN;
	memset(frame->data, 0, sizeof(frame->data));

	/* MCP2518FD stores DB0 at the lowest RAM address (little-endian).
	 * mcp_read32 returns LSB-first, so DB0 is in bits [7:0] of r2. */
	frame->data[0] = (uint8_t)(r2);
	frame->data[1] = (uint8_t)(r2 >> 8);
	frame->data[2] = (uint8_t)(r2 >> 16);
	frame->data[3] = (uint8_t)(r2 >> 24);
	frame->data[4] = (uint8_t)(r3);
	frame->data[5] = (uint8_t)(r3 >> 8);
	frame->data[6] = (uint8_t)(r3 >> 16);
	frame->data[7] = (uint8_t)(r3 >> 24);
}

bool mcp2518_init_all(void)
{
	bool any_ok = false;

	spi_can_port_init();
	spi_can_port_irq_init();

	for (uint8_t r = 0; r < MCP2518_RAIL_COUNT; r++) {
		mcp2518_dev_t *d = &g_dev[r];

		d->rail = r;
		d->rx_irq_pending = false;
		d->tx_ok = 0u;
		d->tx_fail = 0u;
		d->initialized = false;

		mcp_reset(d);
		if (!mcp_wait_osc_ready(d))
			continue;

		mcp_ecc_enable(d);
		mcp_ram_init(d);

		if (!mcp_enter_config(d))
			continue;

		mcp_config_osc_20mhz(d);
		mcp_enable_txq(d);
		mcp_config_bitrate(d);
		mcp_config_c1con_classic(d);
		mcp_config_txq(d);
		mcp_config_rx_fifo(d);
		mcp_config_filters(d);
		mcp_config_iocon(d);

		if (!mcp_enter_normal(d))
			continue;

		mcp_enable_module_irq(d);

		/* TXQ should accept a message after FRESET + normal mode. */
		if (!mcp_txq_has_space(d))
			continue;

		d->initialized = true;
		any_ok = true;
	}
	return any_ok;
}

uint8_t mcp2518_init_mask(void)
{
	uint8_t mask = 0u;

	for (uint8_t r = 0; r < MCP2518_RAIL_COUNT; r++) {
		if (g_dev[r].initialized)
			mask |= (uint8_t)(1u << r);
	}
	return mask;
}

uint8_t mcp2518_rail_opmod(uint8_t rail)
{
	if (rail >= MCP2518_RAIL_COUNT || !g_dev[rail].initialized)
		return 0xFFu;

	mcp2518_dev_t *d = &g_dev[rail];
	return (uint8_t)((mcp_read32(d, REG_C1CON) >> C1CON_OPMOD_SHIFT) & 0x07u);
}

void mcp2518_isr_rx_pending(uint8_t rail)
{
	if (rail < MCP2518_RAIL_COUNT)
		g_dev[rail].rx_irq_pending = true;
}

bool mcp2518_send(can_bus_id_t bus, const can_frame_t *frame)
{
	if (bus < CAN_BUS_CH4 || frame == NULL || frame->dlc > CAN_MAX_DATA_LEN)
		return false;

	uint8_t rail = (uint8_t)(bus - CAN_BUS_CH4);
	mcp2518_dev_t *d = &g_dev[rail];

	if (!d->initialized)
		return false;

	uint8_t tec = 0u;
	mcp_read_trec(d, &tec, NULL);
	if (tec >= 128u)
		(void)mcp_recover_bus_off(d);

	/* TXQ must have space before loading RAM (CiTXQSTA, DS20005678 §4.3). */
	for (int attempt = 0; attempt < 8; attempt++) {
		if (mcp_txq_has_space(d))
			break;
		if (attempt == 7) {
			if (d->tx_fail < 0xFFu)
				d->tx_fail++;
			return false;
		}
		HAL_Delay(1);
	}

	uint32_t ua = mcp_read32(d, MCP_TXQ_UA);
	uint16_t ram = (uint16_t)(MCP_RAM_BASE + (ua & 0x0FFFu));

	bool ext = (frame->id_type == CAN_ID_EXT);
	uint32_t id = ext ? (frame->id & CAN_EXT_MASK) : (frame->id & CAN_STD_ID_MASK);
	uint32_t t0 = mcp_pack_t0(id, ext);
	uint32_t t1 = mcp_pack_t1(frame->dlc, ext);

	/* Pack payload little-endian: DB0 at LSB so mcp_write32 puts it at lowest RAM address. */
	uint32_t d0 = (uint32_t)frame->data[0] | ((uint32_t)frame->data[1] << 8)
				| ((uint32_t)frame->data[2] << 16) | ((uint32_t)frame->data[3] << 24);
	uint32_t d1 = (uint32_t)frame->data[4] | ((uint32_t)frame->data[5] << 8)
				| ((uint32_t)frame->data[6] << 16) | ((uint32_t)frame->data[7] << 24);

	mcp_write32(d, ram + 0u, t0);
	mcp_write32(d, ram + 4u, t1);
	mcp_write32(d, ram + 8u, d0);
	mcp_write32(d, ram + 12u, d1);

	mcp_txq_request(d);
	(void)mcp_txq_wait_done(d, 15u);

	if (d->tx_ok < 0xFFu)
		d->tx_ok++;
	return true;
}

bool mcp2518_recv(can_bus_id_t bus, can_frame_t *frame)
{
	if (bus < CAN_BUS_CH4 || frame == NULL)
		return false;

	uint8_t rail = (uint8_t)(bus - CAN_BUS_CH4);
	mcp2518_dev_t *d = &g_dev[rail];

	if (!d->initialized)
		return false;

	/* Bit 0 = TFNRFNIF: RX not empty (byte 0 of CiFIFOSTA). */
	if (!mcp_rx_fifo_not_empty(d))
		return false;

	/* CiFIFOUA holds a 12-bit RAM offset; SFR address = 0x400 + offset (Eq. 4-1). */
	uint32_t ua = mcp_read32(d, MCP_FIFO_UA(MCP_RX_FIFO));
	uint16_t ram = (uint16_t)(MCP_RAM_BASE + (ua & 0x0FFFu));

	uint32_t r0 = mcp_read32(d, ram + 0u);
	uint32_t r1 = mcp_read32(d, ram + 4u);
	uint32_t r2 = mcp_read32(d, ram + 8u);
	uint32_t r3 = mcp_read32(d, ram + 12u);

	mcp_unpack_to_frame(r0, r1, r2, r3, frame);

	mcp_rx_fifo_pop(d);

	return true;
}

void mcp2518_drain_rx(can_bus_id_t bus)
{
	if (bus < CAN_BUS_CH4)
		return;

	uint8_t rail = (uint8_t)(bus - CAN_BUS_CH4);
	mcp2518_dev_t *d = &g_dev[rail];

	d->rx_irq_pending = false;

	/* Always service RX FIFO / INT flags — do not gate on INT GPIO (polling fallback). */
	uint8_t scratch = 0u;

	(void)mcp_read_buf(d, MCP_FIFO_STA(MCP_RX_FIFO), &scratch, 1u);
	(void)mcp_read32(d, REG_C1INT);
}

void mcp2518_get_tx_stats(uint8_t rail, uint8_t *tx_ok, uint8_t *tx_fail)
{
	if (rail >= MCP2518_RAIL_COUNT)
		return;

	if (tx_ok != NULL)
		*tx_ok = g_dev[rail].tx_ok;
	if (tx_fail != NULL)
		*tx_fail = g_dev[rail].tx_fail;
}

void mcp2518_rail_trec(uint8_t rail, uint8_t *tec, uint8_t *rec)
{
	if (rail >= MCP2518_RAIL_COUNT || !g_dev[rail].initialized)
		return;

	mcp_read_trec(&g_dev[rail], tec, rec);
}

bool mcp2518_recover_bus(can_bus_id_t bus)
{
	if (bus < CAN_BUS_CH4)
		return true;

	uint8_t rail = (uint8_t)(bus - CAN_BUS_CH4);
	return mcp_recover_bus_off(&g_dev[rail]);
}

void mcp2518_recover_all(void)
{
	for (uint8_t r = 0; r < MCP2518_RAIL_COUNT; r++) {
		if (g_dev[r].initialized)
			(void)mcp_recover_bus_off(&g_dev[r]);
	}
}

bool mcp2518_bus_smoke(can_bus_id_t bus, const can_frame_t *frame,
                       uint16_t listen_ms, mcp2518_smoke_result_t *out)
{
	if (bus < CAN_BUS_CH4 || frame == NULL || out == NULL)
		return false;

	memset(out, 0, sizeof(*out));

	uint8_t rail = (uint8_t)(bus - CAN_BUS_CH4);
	mcp2518_dev_t *d = &g_dev[rail];

	if (!d->initialized)
		return false;

	{
		uint8_t tec_pre = 0u;
		mcp_read_trec(d, &tec_pre, NULL);
		if (tec_pre >= 128u)
			(void)mcp_recover_bus_off(d);
	}

	can_frame_t rx;
	uint32_t c1con = mcp_read32(d, REG_C1CON);
	uint32_t osc = mcp_read32(d, REG_OSC);
	uint32_t nbt = mcp_read32(d, REG_C1NBTCFG);
	uint32_t bdiag1 = mcp_read32(d, REG_C1BDIAG1);

	out->c1con_b2 = (uint8_t)((c1con >> 16) & 0xFFu);
	out->osc_b0 = (uint8_t)(osc & 0xFFu);
	out->osc_b1 = (uint8_t)((osc >> 8) & 0xFFu);
	out->nbt_tseg1 = (uint8_t)((nbt >> 16) & 0xFFu);
	out->bdiag1_b0 = (uint8_t)(bdiag1 & 0xFFu);
	out->tx_fifo_sta = (uint8_t)(mcp_read32(d, MCP_TXQ_STA) & 0xFFu);
	out->tx_fifo_con = (uint8_t)(mcp_read32(d, MCP_TXQ_CON) & 0xFFu);

	/* One TX — matches FDCAN AutoRetransmission=DISABLE (no error-frame storm on NACK). */
	if (mcp2518_send(bus, frame)) {
		if (out->tx_ok < 0xFFu)
			out->tx_ok++;
	} else if (out->tx_fail < 0xFFu) {
		out->tx_fail++;
	}
	HAL_Delay(5);

	out->tx_fifo_sta = (uint8_t)(mcp_read32(d, MCP_TXQ_STA) & 0xFFu);
	out->tx_fifo_con = (uint8_t)(mcp_read32(d, MCP_TXQ_CON) & 0xFFu);
	mcp_read_trec(d, &out->tec, &out->rec);
	bdiag1 = mcp_read32(d, REG_C1BDIAG1);
	out->bdiag1_b0 = (uint8_t)(bdiag1 & 0xFFu);

	for (uint16_t i = 0; i < listen_ms; i++) {
		mcp2518_drain_rx(bus);
		while (mcp2518_recv(bus, &rx)) {
			if (out->rx_frames < 0xFFu)
				out->rx_frames++;
		}
		HAL_Delay(1);
	}

	mcp_read_trec(d, &out->tec, &out->rec);
	return (out->tx_ok > 0u);
}
