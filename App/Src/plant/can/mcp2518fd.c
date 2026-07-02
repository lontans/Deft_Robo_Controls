#include "plant/can/mcp2518fd.h"
#include "plant/can/spi_can_port.h"
#include "plant/can/can_frame.h"
#include "plant/can/can_router.h"
#include "plant/plant_diag.h"

#include "main.h"
#include <string.h>

/* SPI instruction opcodes (DS20006027 Table 4-1). */
#define MCP_SPI_RESET  0x00u
#define MCP_SPI_WRITE  0x02u
#define MCP_SPI_READ   0x03u

/* SFR addresses (DS20006027 Table 3-1). */
#define REG_C1CON      0x000u
#define REG_C1NBTCFG   0x004u
#define REG_C1INT      0x01Cu
#define REG_C1TXATIF   0x02Cu
#define REG_C1TXREQ    0x030u
#define REG_C1TREC     0x034u
#define REG_C1BDIAG1   0x038u

#define BDIAG1_NACKERR   (1u << 18)
#define REG_C1FLTCON0  0x1D0u
#define REG_C1FLTOBJ0  0x1F0u
#define REG_C1MASK0    0x1F4u
#define REG_OSC        0xE00u
#define REG_IOCON      0xE04u
#define REG_ECCCON     0xE0Cu
#define REG_DEVID      0xE14u

/* CiTXQ @ 0x050 — NOT CiFIFOCON1 @ 0x05C (RM 20005678). */
#define REG_C1TXQCON   0x050u
#define REG_C1TXQSTA   0x054u
#define REG_C1TXQUA    0x058u

#define REG_C1FIFOCON(n)  (0x05Cu + (uint16_t)(n) * 12u)
#define REG_C1FIFOSTA(n)  (0x060u + (uint16_t)(n) * 12u)
#define REG_C1FIFOUA(n)   (0x064u + (uint16_t)(n) * 12u)

#define MCP_RX_FIFO_IDX  1u

#define MCP_RAM_BASE     0x400u
#define MCP_RAM_SIZE     2048u

/* OSC register (REGISTER 3-1). */
#define OSC_PLLEN        (1u << 0)
#define OSC_OSCDIS       (1u << 2)
#define OSC_SCLKDIV      (1u << 5)
#define OSC_PLLRDY       (1u << 8)
#define OSC_OSCRDY       (1u << 10)

/* CiCON (REGISTER 3-7). */
#define C1CON_REQOP(n)   (((uint32_t)(n) & 0x7u) << 24)
#define C1CON_REQOP_MASK (0x7u << 24)
#define C1CON_OPMOD_SHIFT 21u
#define C1CON_OPMOD_CFG     4u
#define C1CON_OPMOD_EXT_LB  5u
#define C1CON_OPMOD_CAN20   6u
#define C1CON_TXQEN      (1u << 20)
#define C1CON_RTXAT      (1u << 16)
#define C1CON_BRSDIS     (1u << 12)
#define C1CON_ABAT       (1u << 29)

/* CiINT (REGISTER 3-14). */
#define C1INT_RXIE       (1u << 17)
#define C1INT_RXIF       (1u << 1)

/* CiTXQCON / CiFIFOCON bits (REGISTER 3-26). */
#define FIFO_TXEN        (1u << 7)
#define FIFO_UINC        (1u << 8)
#define FIFO_TXREQ       (1u << 9)
#define FIFO_FRESET      (1u << 10)
#define FIFO_TFNRFNIE    (1u << 0)

/* CiTXQSTA (REGISTER 3-27). */
#define TXQ_STA_TXQNIF   (1u << 0) /* queue not full */
#define TXQ_STA_TXQEIF   (1u << 2) /* queue empty */
#define TXQ_STA_TXATIF   (1u << 4)
#define TXQ_STA_TXERR    (1u << 5)
#define TXQ_STA_TXLARB   (1u << 6)
#define TXQ_STA_TXABT    (1u << 7)
#define TXQ_STA_BUSERR   (TXQ_STA_TXERR | TXQ_STA_TXLARB | TXQ_STA_TXABT)
#define TXQ_STA_DONE     (TXQ_STA_BUSERR | TXQ_STA_TXATIF)

#define TXQ_CON_TXREQ    0x02u /* byte1 bit9 — request send (RM: set TXREQ after RAM load) */
#define TXQ_CON_UINC     0x01u /* byte1 bit8 — advance after bus attempt completes */

/* CiFIFOSTA (REGISTER 3-28) — TFNRFNIF = not empty for RX FIFO. */
#define FIFO_STA_TFNRFNIF (1u << 0)

/* IOCON byte @ REG_IOCON+3 (bits 31:24): INTOD=bit6, TXCANOD=bit4, PM1/PM0=bits1:0. */
#define IOCON_B3_INTOD   (1u << 6)
/* IOCON byte @ REG_IOCON+1 (bits 15:8): XSTBYEN=bit6. */
#define IOCON_B1_XSTBYEN (1u << 6)

/* CiNBTCFG pack (REGISTER 3-8). */
#define MCP_NBTCFG_PACK(brp, tseg1, tseg2, sjw) \
	(((uint32_t)(brp) << 24) | ((uint32_t)(tseg1) << 16) | \
	 ((uint32_t)(tseg2) << 8) | (uint32_t)(sjw))

/* TXQ: PLSIZE=0 (8 B), FSIZE=0 (1 slot), TXEN=1. */
#define MCP_TXQ_1X8      ((0u << 29) | (0u << 24) | FIFO_TXEN)
/* RX FIFO1: 4×8 B receive-only. */
#define MCP_RX_FIFO_4X8  ((0u << 29) | (3u << 24))

#define MCP_T1_FDF  (1u << 7)
#define MCP_T1_BRS  (1u << 6)
#define MCP_T1_RTR  (1u << 5)
#define MCP_T1_IDE  (1u << 4)

typedef struct {
	uint8_t rail;
	bool initialized;
	volatile bool rx_irq_pending;
	uint8_t tx_ok;
	uint8_t tx_fail;
	uint8_t tx_nack;
	mcp2518_init_diag_t init_diag;
} mcp2518_dev_t;

typedef struct {
	bool completed;
	bool bus_err;
	uint8_t sta;
} mcp_txq_done_t;

static mcp2518_dev_t g_dev[MCP2518_RAIL_COUNT];

static bool mcp_read_buf(mcp2518_dev_t *d, uint16_t addr, uint8_t *out, uint16_t len);
static void mcp_read_trec(mcp2518_dev_t *d, uint8_t *tec, uint8_t *rec);
static bool mcp_trec_read(mcp2518_dev_t *d, uint8_t *tec, uint8_t *rec);

/* -------------------------------------------------------------------------- */
/* Low-level SPI                                                              */
/* -------------------------------------------------------------------------- */

static bool mcp_xfer(mcp2518_dev_t *d, const uint8_t *tx, uint8_t *rx, uint16_t len)
{
	return spi_can_port_xfer(d->rail, tx, rx, len);
}

static bool mcp_read_buf(mcp2518_dev_t *d, uint16_t addr, uint8_t *out, uint16_t len)
{
	if (out == NULL || len == 0u || len > 32u)
		return false;

	uint8_t tx[2 + 32];
	uint8_t rx[2 + 32];

	tx[0] = (uint8_t)((MCP_SPI_READ << 4) | ((addr >> 8) & 0x0Fu));
	tx[1] = (uint8_t)addr;
	memset(&tx[2], 0, len);

	if (!mcp_xfer(d, tx, rx, (uint16_t)(2u + len)))
		return false;

	memcpy(out, &rx[2], len);
	return true;
}

static bool mcp_write_buf(mcp2518_dev_t *d, uint16_t addr, const uint8_t *data, uint16_t len)
{
	if (data == NULL || len == 0u || len > 32u)
		return false;

	uint8_t tx[2 + 32];

	tx[0] = (uint8_t)((MCP_SPI_WRITE << 4) | ((addr >> 8) & 0x0Fu));
	tx[1] = (uint8_t)addr;
	memcpy(&tx[2], data, len);

	return mcp_xfer(d, tx, NULL, (uint16_t)(2u + len));
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

static bool mcp_write_byte(mcp2518_dev_t *d, uint16_t addr, uint8_t val)
{
	return mcp_write_buf(d, addr, &val, 1u);
}

static void mcp_read_trec(mcp2518_dev_t *d, uint8_t *tec, uint8_t *rec)
{
	uint8_t b[2];

	if (!mcp_read_buf(d, REG_C1TREC, b, 2u)) {
		if (tec != NULL)
			*tec = 0u;
		if (rec != NULL)
			*rec = 0u;
		return;
	}

	if (rec != NULL)
		*rec = b[0];
	if (tec != NULL)
		*tec = b[1];
}

static bool mcp_trec_read(mcp2518_dev_t *d, uint8_t *tec, uint8_t *rec)
{
	uint8_t b[2];

	if (!mcp_read_buf(d, REG_C1TREC, b, 2u))
		return false;

	if (rec != NULL)
		*rec = b[0];
	if (tec != NULL)
		*tec = b[1];
	return true;
}

static void mcp_spi_reset(mcp2518_dev_t *d)
{
	uint8_t tx[2] = { MCP_SPI_RESET, 0x00u };

	(void)mcp_xfer(d, tx, NULL, 2u);
	HAL_Delay(5);
}

/* -------------------------------------------------------------------------- */
/* Oscillator / mode                                                          */
/* -------------------------------------------------------------------------- */

static bool mcp_wait_oscrdy(mcp2518_dev_t *d)
{
	for (int i = 0; i < 200; i++) {
		uint32_t osc = mcp_read32(d, REG_OSC);

		if ((osc & OSC_OSCRDY) != 0u)
			return true;
		HAL_Delay(1);
	}
	return false;
}

static uint8_t mcp_opmod_read(mcp2518_dev_t *d)
{
	return (uint8_t)((mcp_read32(d, REG_C1CON) >> C1CON_OPMOD_SHIFT) & 0x07u);
}

static bool mcp_wait_opmod(mcp2518_dev_t *d, uint8_t want)
{
	for (int i = 0; i < 200; i++) {
		if (mcp_opmod_read(d) == want)
			return true;
		HAL_Delay(1);
	}
	return false;
}

static bool mcp_request_opmod(mcp2518_dev_t *d, uint8_t reqop, uint8_t want_opmod)
{
	uint32_t con = mcp_read32(d, REG_C1CON);

	con &= ~C1CON_REQOP_MASK;
	con |= C1CON_REQOP(reqop);
	mcp_write32(d, REG_C1CON, con);
	return mcp_wait_opmod(d, want_opmod);
}

static bool mcp_enter_config(mcp2518_dev_t *d)
{
	return mcp_request_opmod(d, C1CON_OPMOD_CFG, C1CON_OPMOD_CFG);
}

static bool mcp_enter_normal_can20(mcp2518_dev_t *d)
{
	return mcp_request_opmod(d, C1CON_OPMOD_CAN20, C1CON_OPMOD_CAN20);
}

static bool mcp_enter_ext_loopback(mcp2518_dev_t *d)
{
	return mcp_request_opmod(d, C1CON_OPMOD_EXT_LB, C1CON_OPMOD_EXT_LB);
}

static void mcp_config_osc_20mhz_xtal(mcp2518_dev_t *d)
{
	uint32_t osc = mcp_read32(d, REG_OSC);

	osc &= ~(OSC_PLLEN | OSC_SCLKDIV | OSC_OSCDIS);
	mcp_write32(d, REG_OSC, osc);
	(void)mcp_wait_oscrdy(d);
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

static bool mcp_read_devid(mcp2518_dev_t *d, uint8_t *devid, uint8_t *rev)
{
	uint32_t raw = mcp_read32(d, REG_DEVID);

	if (raw == 0xFFFFFFFFu)
		return false;

	if (devid != NULL)
		*devid = (uint8_t)(raw & 0xFFu);
	if (rev != NULL)
		*rev = (uint8_t)((raw >> 8) & 0xFFu);
	return true;
}

/* -------------------------------------------------------------------------- */
/* Bit timing / CiCON                                                         */
/* -------------------------------------------------------------------------- */

static void mcp_config_bitrate_1mbps(mcp2518_dev_t *d)
{
	const uint32_t nbt = MCP_NBTCFG_PACK(MCP2518_NBT_BRP_EXPECT,
	                                     MCP2518_NBT_TSEG1_EXPECT,
	                                     MCP2518_NBT_TSEG2_EXPECT,
	                                     MCP2518_NBT_SJW_EXPECT);

	mcp_write32(d, REG_C1NBTCFG, nbt);
}

static bool mcp_nbtcfg_readback_ok(mcp2518_dev_t *d, uint8_t *brp, uint8_t *tseg1)
{
	uint32_t nbt = mcp_read32(d, REG_C1NBTCFG);

	if (nbt == 0xFFFFFFFFu)
		return false;

	uint8_t rb_brp = (uint8_t)((nbt >> 24) & 0xFFu);
	uint8_t rb_t1 = (uint8_t)((nbt >> 16) & 0xFFu);

	if (brp != NULL)
		*brp = rb_brp;
	if (tseg1 != NULL)
		*tseg1 = rb_t1;

	return (rb_brp == MCP2518_NBT_BRP_EXPECT &&
	        rb_t1 == MCP2518_NBT_TSEG1_EXPECT);
}

static void mcp_config_c1con_classic(mcp2518_dev_t *d)
{
	uint32_t con = mcp_read32(d, REG_C1CON);

	con |= C1CON_BRSDIS | C1CON_RTXAT;
	mcp_write32(d, REG_C1CON, con);
}

static void mcp_enable_txq(mcp2518_dev_t *d)
{
	uint32_t con = mcp_read32(d, REG_C1CON);

	con |= C1CON_TXQEN;
	mcp_write32(d, REG_C1CON, con);
}

/* -------------------------------------------------------------------------- */
/* IOCON — byte-wise SFR writes per DS20006027 Note on REGISTER 3-2.          */
/* -------------------------------------------------------------------------- */

static void mcp_config_iocon(mcp2518_dev_t *d)
{
	/* Byte 3 (bits 31:24): INTOD=1, TXCANOD=0, PM1=PM0=0 (INT pins, not GPIO). */
	(void)mcp_write_byte(d, (uint16_t)(REG_IOCON + 3u), IOCON_B3_INTOD);

	/* Byte 1 (bits 15:8): XSTBYEN=0 — pin 9 NC, transceiver STBY hardwired GND. */
	(void)mcp_write_byte(d, (uint16_t)(REG_IOCON + 1u), 0x00u);
}

/* -------------------------------------------------------------------------- */
/* TX Queue                                                                   */
/* -------------------------------------------------------------------------- */

static void mcp_config_txq(mcp2518_dev_t *d)
{
	uint32_t txqcon = MCP_TXQ_1X8 | FIFO_FRESET;

	mcp_write32(d, REG_C1TXQCON, txqcon);
	mcp_write32(d, REG_C1TXQCON, MCP_TXQ_1X8);
}

static void mcp_txq_uinc(mcp2518_dev_t *d)
{
	(void)mcp_write_byte(d, (uint16_t)(REG_C1TXQCON + 1u), TXQ_CON_UINC);
}

static void mcp_txq_clear_atif(mcp2518_dev_t *d)
{
	/* CiTXATIF is W1C — clear pending attempt flag before the next frame. */
	(void)mcp_write_byte(d, REG_C1TXATIF, 0x01u);
}

static void mcp_txq_release_after_tx(mcp2518_dev_t *d)
{
	/* Drain DONE/ATIF first, then hard-reset the 1-deep TXQ for the next load. */
	for (int i = 0; i < 16; i++) {
		uint8_t sta = 0u;

		if (!mcp_read_buf(d, REG_C1TXQSTA, &sta, 1u))
			break;

		if ((sta & TXQ_STA_DONE) != 0u)
			mcp_txq_uinc(d);
		else if ((sta & TXQ_STA_TXQEIF) != 0u)
			break;

		HAL_Delay(1);
	}

	mcp_config_txq(d);
	mcp_txq_clear_atif(d);

	for (int i = 0; i < 20; i++) {
		uint8_t sta = 0u;

		if (mcp_read_buf(d, REG_C1TXQSTA, &sta, 1u) &&
		    ((sta & TXQ_STA_TXQEIF) != 0u))
			return;

		HAL_Delay(1);
	}
}

static void mcp_txq_commit_and_request(mcp2518_dev_t *d)
{
	/* Byte writes match RM / mcp_txq_uinc — 32-bit RMW can miss on SPI SFRs. */
	(void)mcp_write_byte(d, (uint16_t)(REG_C1TXQCON + 1u), TXQ_CON_UINC);

	for (int i = 0; i < 20; i++) {
		uint8_t sta = 0u;

		if (mcp_read_buf(d, REG_C1TXQSTA, &sta, 1u) &&
		    ((sta & TXQ_STA_TXQEIF) == 0u))
			break;

		HAL_Delay(1);
	}

	(void)mcp_write_byte(d, (uint16_t)(REG_C1TXQCON + 1u), TXQ_CON_TXREQ);
}

static bool mcp_txq_wait_done(mcp2518_dev_t *d, uint16_t timeout_ms,
                              uint8_t tec_before, mcp_txq_done_t *out)
{
	bool tx_started = false;

	(void)tec_before;

	if (out != NULL) {
		out->completed = false;
		out->bus_err = false;
		out->sta = 0u;
	}

	for (uint32_t i = 0; i < (uint32_t)timeout_ms * 200u; i++) {
		uint8_t con_b1 = 0u;
		uint8_t sta = 0u;
		uint8_t txatif = 0u;

		if (!mcp_read_buf(d, (uint16_t)(REG_C1TXQCON + 1u), &con_b1, 1u))
			return false;
		if (!mcp_read_buf(d, REG_C1TXQSTA, &sta, 1u))
			return false;
		(void)mcp_read_buf(d, REG_C1TXATIF, &txatif, 1u);

		/* After TXREQ, queue is non-empty until the MAC finishes. */
		if ((con_b1 & TXQ_CON_TXREQ) != 0u ||
		    (sta & TXQ_STA_TXQEIF) == 0u)
			tx_started = true;

		if (!tx_started)
			goto spin;

		bool atif = ((sta & TXQ_STA_TXATIF) != 0u) ||
		              ((txatif & 0x01u) != 0u);
		bool buserr = (sta & TXQ_STA_BUSERR) != 0u;

		/* RM: wait for transmission attempt complete (TXATIF), not TXREQ clear. */
		if (atif || buserr) {
			if (out != NULL) {
				out->completed = true;
				out->bus_err = buserr;
				out->sta = sta;
			}
			mcp_txq_release_after_tx(d);
			return true;
		}

spin:
		if ((i % 200u) == 199u)
			HAL_Delay(1);
	}

	return false;
}

static bool mcp_txq_has_space(mcp2518_dev_t *d)
{
	uint8_t sta = 0u;

	if (!mcp_read_buf(d, REG_C1TXQSTA, &sta, 1u))
		return false;

	return ((sta & TXQ_STA_TXQNIF) != 0u) || ((sta & TXQ_STA_TXQEIF) != 0u);
}

static void mcp_txq_force_ready(mcp2518_dev_t *d)
{
	uint8_t sta = 0u;

	if (!mcp_read_buf(d, REG_C1TXQSTA, &sta, 1u))
		return;

	if ((sta & TXQ_STA_TXQEIF) != 0u)
		return;

	mcp_txq_release_after_tx(d);
}

/* -------------------------------------------------------------------------- */
/* RX FIFO + filters                                                          */
/* -------------------------------------------------------------------------- */

static void mcp_config_rx_fifo(mcp2518_dev_t *d)
{
	uint32_t rxcon = MCP_RX_FIFO_4X8 | FIFO_TFNRFNIE | FIFO_FRESET;

	mcp_write32(d, REG_C1FIFOCON(MCP_RX_FIFO_IDX), rxcon);
	mcp_write32(d, REG_C1FIFOCON(MCP_RX_FIFO_IDX),
	            MCP_RX_FIFO_4X8 | FIFO_TFNRFNIE);
}

#define FLTOBJ_EXIDE  (1u << 30)
#define MASK_MIDE     (1u << 30)

static void mcp_config_filters_accept_all(mcp2518_dev_t *d)
{
	/* Disable filter 0, accept all 29-bit extended frames into RX FIFO1. */
	(void)mcp_write_byte(d, REG_C1FLTCON0, 0x00u);
	mcp_write32(d, REG_C1FLTOBJ0, FLTOBJ_EXIDE);
	mcp_write32(d, REG_C1MASK0, MASK_MIDE); /* MIDE=1 + zero ID mask → any ext ID */
	(void)mcp_write_byte(d, REG_C1FLTCON0,
	                     (uint8_t)(0x80u | MCP_RX_FIFO_IDX)); /* FLTEN0 | FIFO1 */
}

static void mcp_enable_rx_irq(mcp2518_dev_t *d)
{
	uint32_t ciint = mcp_read32(d, REG_C1INT);

	ciint |= C1INT_RXIE;
	mcp_write32(d, REG_C1INT, ciint);
}

static bool mcp_rx_fifo_not_empty(mcp2518_dev_t *d)
{
	uint8_t sta = 0u;

	if (!mcp_read_buf(d, REG_C1FIFOSTA(MCP_RX_FIFO_IDX), &sta, 1u))
		return false;

	return (sta & FIFO_STA_TFNRFNIF) != 0u;
}

static void mcp_rx_fifo_uinc(mcp2518_dev_t *d)
{
	(void)mcp_write_byte(d, (uint16_t)(REG_C1FIFOCON(MCP_RX_FIFO_IDX) + 1u), 0x01u);
}

static void mcp_clear_rx_irq_flag(mcp2518_dev_t *d)
{
	/* RXIF / RFIF clear via CiFIFOCON.UINC after pop — do not RMW-write C1INT. */
	(void)d;
}

/* -------------------------------------------------------------------------- */
/* TREC / bus-off recovery                                                    */
/* -------------------------------------------------------------------------- */

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

	mcp_config_txq(d);
	mcp_config_rx_fifo(d);
	mcp_config_filters_accept_all(d);
	mcp_enable_rx_irq(d);
	return mcp_enter_normal_can20(d);
}

/* -------------------------------------------------------------------------- */
/* Message RAM pack/unpack (RM 20005678 Table 4-1)                          */
/* -------------------------------------------------------------------------- */

uint32_t mcp2518_pack_t0(uint32_t can_id, bool ext)
{
	if (!ext)
		return can_id & CAN_STD_ID_MASK;

	uint32_t sid = (can_id >> 18) & CAN_STD_ID_MASK;
	uint32_t eid = can_id & 0x3FFFFu;
	return (eid << 11) | sid;
}

uint32_t mcp2518_unpack_t0(uint32_t t0, bool ext)
{
	if (!ext)
		return t0 & CAN_STD_ID_MASK;

	uint32_t sid = t0 & CAN_STD_ID_MASK;
	uint32_t eid = (t0 >> 11) & 0x3FFFFu;
	return (sid << 18) | eid;
}

uint32_t mcp2518_pack_t1(uint8_t dlc, bool ext, bool remote)
{
	uint32_t t1 = (uint32_t)(dlc & 0x0Fu);

	if (ext)
		t1 |= MCP_T1_IDE;
	if (remote)
		t1 |= MCP_T1_RTR;
	return t1;
}

bool mcp2518_id_roundtrip(uint32_t can_id)
{
	uint32_t t0 = mcp2518_pack_t0(can_id, true);
	return mcp2518_unpack_t0(t0, true) == (can_id & CAN_EXT_MASK);
}

#define MCP_PACK_T0_EXT(id) \
	(((((uint32_t)(id)) & 0x3FFFFu) << 11) | ((((uint32_t)(id)) >> 18) & CAN_STD_ID_MASK))
#define MCP_UNPACK_T0_EXT(t0) \
	(((((uint32_t)(t0)) & CAN_STD_ID_MASK) << 18) | ((((uint32_t)(t0)) >> 11) & 0x3FFFFu))

_Static_assert(MCP_UNPACK_T0_EXT(MCP_PACK_T0_EXT(0x0300FD70u)) == 0x0300FD70u,
               "MCP2518 extended ID pack must match RobStride/FDCAN flat 29-bit ID");

static uint8_t mcp_dlc_from_r1(uint32_t r1)
{
	uint8_t dlc = (uint8_t)(r1 & 0x0Fu);
	return (dlc > CAN_MAX_DATA_LEN) ? CAN_MAX_DATA_LEN : dlc;
}

static void mcp_unpack_to_frame(uint32_t r0, uint32_t r1,
                                uint32_t r2, uint32_t r3,
                                can_frame_t *frame)
{
	bool ext = ((r1 >> 4) & 1u) != 0u;

	frame->id = mcp2518_unpack_t0(r0, ext);
	frame->id_type = ext ? CAN_ID_EXT : CAN_ID_STD;

	uint8_t dlc = mcp_dlc_from_r1(r1);
	frame->dlc = dlc;
	memset(frame->data, 0, sizeof(frame->data));

	frame->data[0] = (uint8_t)(r2);
	frame->data[1] = (uint8_t)(r2 >> 8);
	frame->data[2] = (uint8_t)(r2 >> 16);
	frame->data[3] = (uint8_t)(r2 >> 24);
	frame->data[4] = (uint8_t)(r3);
	frame->data[5] = (uint8_t)(r3 >> 8);
	frame->data[6] = (uint8_t)(r3 >> 16);
	frame->data[7] = (uint8_t)(r3 >> 24);
}

static bool mcp_hw_pop_rx(mcp2518_dev_t *d, can_frame_t *frame)
{
	if (!mcp_rx_fifo_not_empty(d))
		return false;

	uint32_t ua = mcp_read32(d, REG_C1FIFOUA(MCP_RX_FIFO_IDX));
	uint16_t ram = (uint16_t)(MCP_RAM_BASE + (ua & 0x0FFFu));

	uint32_t r0 = mcp_read32(d, ram + 0u);
	uint32_t r1 = mcp_read32(d, ram + 4u);
	uint32_t r2 = mcp_read32(d, ram + 8u);
	uint32_t r3 = mcp_read32(d, ram + 12u);

	if (frame != NULL)
		mcp_unpack_to_frame(r0, r1, r2, r3, frame);

	mcp_rx_fifo_uinc(d);
	mcp_clear_rx_irq_flag(d);
	return true;
}

static bool mcp_hw_txq_load(mcp2518_dev_t *d, const can_frame_t *frame)
{
	uint32_t ua = mcp_read32(d, REG_C1TXQUA);
	uint16_t ram = (uint16_t)(MCP_RAM_BASE + (ua & 0x0FFFu));

	bool ext = (frame->id_type == CAN_ID_EXT);
	uint32_t id = ext ? (frame->id & CAN_EXT_MASK) : (frame->id & CAN_STD_ID_MASK);
	uint32_t t0 = mcp2518_pack_t0(id, ext);
	uint32_t t1 = mcp2518_pack_t1(frame->dlc, ext, false);

	uint32_t d0 = (uint32_t)frame->data[0] | ((uint32_t)frame->data[1] << 8)
	            | ((uint32_t)frame->data[2] << 16) | ((uint32_t)frame->data[3] << 24);
	uint32_t d1 = (uint32_t)frame->data[4] | ((uint32_t)frame->data[5] << 8)
	            | ((uint32_t)frame->data[6] << 16) | ((uint32_t)frame->data[7] << 24);

	mcp_write32(d, ram + 0u, t0);
	mcp_write32(d, ram + 4u, t1);
	mcp_write32(d, ram + 8u, d0);
	mcp_write32(d, ram + 12u, d1);

	mcp_txq_commit_and_request(d);
	return true;
}

static uint8_t mcp_tec_nack_count(uint8_t tec_before, uint8_t tec_after)
{
	if (tec_after <= tec_before)
		return 0u;

	uint16_t delta = (uint16_t)(tec_after - tec_before);
	uint8_t nack = (uint8_t)((delta + 7u) / 8u);

	return (nack > 0u) ? 1u : 0u; /* single-frame smoke: saturate at 1 */
}

static void mcp_read_tec_after_tx(mcp2518_dev_t *d, uint8_t tec_before,
                                  uint8_t *tec_out, bool *nack_out)
{
	uint8_t tec = tec_before;
	bool nack = false;

	for (int i = 0; i < 20; i++) {
		HAL_Delay(1);
		if (!mcp_trec_read(d, &tec, NULL))
			break;
		if (tec > tec_before)
			break;
	}

	uint32_t bdiag1 = mcp_read32(d, REG_C1BDIAG1);
	if (tec > tec_before || ((bdiag1 & BDIAG1_NACKERR) != 0u))
		nack = true;

	if (tec_out != NULL)
		*tec_out = tec;
	if (nack_out != NULL)
		*nack_out = nack;
}

static void mcp_note_tx_result(mcp2518_dev_t *d, uint8_t tec_before)
{
	uint8_t tec_after = tec_before;
	bool nack = false;

	mcp_read_tec_after_tx(d, tec_before, &tec_after, &nack);

	if (nack) {
		uint8_t by_tec = mcp_tec_nack_count(tec_before, tec_after);
		uint8_t hits = (by_tec > 0u) ? by_tec : 1u;
		if (d->tx_nack < 0xFFu)
			d->tx_nack = (uint8_t)(d->tx_nack + hits);
	}
}

/* -------------------------------------------------------------------------- */
/* External loopback self-test (TXCAN↔RXCAN, no transceiver)                */
/* -------------------------------------------------------------------------- */

static bool mcp_ext_loopback_test(mcp2518_dev_t *d)
{
	can_frame_t tx = {
		.id = 0x123u,
		.id_type = CAN_ID_STD,
		.dlc = 2u,
		.data = { 0xDEu, 0xADu },
	};

	if (!mcp_enter_ext_loopback(d))
		return false;

	/* Drain any stale RX before test. */
	while (mcp_hw_pop_rx(d, NULL))
		;

	if (!mcp_txq_has_space(d))
		goto fail_reconfig;

	(void)mcp_hw_txq_load(d, &tx);

	mcp_txq_done_t done = { 0 };
	if (!mcp_txq_wait_done(d, 20u, 0u, &done))
		goto fail_reconfig;

	for (int i = 0; i < 20; i++) {
		can_frame_t rx;

		if (mcp_hw_pop_rx(d, &rx)) {
			bool ok = (rx.id == tx.id && rx.dlc == tx.dlc &&
			           rx.data[0] == tx.data[0] && rx.data[1] == tx.data[1]);
			(void)mcp_enter_config(d);
			return ok;
		}
		HAL_Delay(1);
	}

fail_reconfig:
	(void)mcp_enter_config(d);
	return false;
}

/* -------------------------------------------------------------------------- */
/* Diagnostics                                                                */
/* -------------------------------------------------------------------------- */

static void mcp_fill_smoke_diag(mcp2518_dev_t *d, mcp2518_smoke_result_t *out)
{
	uint32_t c1con = mcp_read32(d, REG_C1CON);
	uint32_t osc = mcp_read32(d, REG_OSC);
	uint32_t nbt = mcp_read32(d, REG_C1NBTCFG);
	uint32_t bdiag1 = mcp_read32(d, REG_C1BDIAG1);

	out->c1con_b2 = (uint8_t)((c1con >> 16) & 0xFFu);
	out->osc_b0 = (uint8_t)(osc & 0xFFu);
	out->osc_b1 = (uint8_t)((osc >> 8) & 0xFFu);
	out->nbt_brp = (uint8_t)((nbt >> 24) & 0xFFu);
	out->nbt_tseg1 = (uint8_t)((nbt >> 16) & 0xFFu);
	out->bdiag1_b0 = (uint8_t)(bdiag1 & 0xFFu);
	out->bdiag1_b1 = (uint8_t)((bdiag1 >> 16) & 0xFFu);
	out->tx_fifo_sta = (uint8_t)(mcp_read32(d, REG_C1TXQSTA) & 0xFFu);
	out->tx_fifo_con = (uint8_t)(mcp_read32(d, REG_C1TXQCON) & 0xFFu);
	out->init_fail_bits = (uint8_t)(d->init_diag.fail_bits & 0xFFu);
	out->ext_loopback_ok = d->init_diag.ext_loopback_ok;
	out->devid = d->init_diag.devid;
}

void mcp2518_get_init_diag(uint8_t rail, mcp2518_init_diag_t *out)
{
	if (rail >= MCP2518_RAIL_COUNT || out == NULL)
		return;

	*out = g_dev[rail].init_diag;
}

void mcp2518_refresh_smoke_diag(can_bus_id_t bus, mcp2518_smoke_result_t *out)
{
	if (bus < CAN_BUS_CH4 || out == NULL)
		return;

	uint8_t rail = (uint8_t)(bus - CAN_BUS_CH4);
	mcp2518_dev_t *d = &g_dev[rail];

	if (!d->initialized)
		return;

	mcp_fill_smoke_diag(d, out);
	mcp_read_trec(d, &out->tec, &out->rec);
	out->tec_before = out->tec;
	out->tx_ok = d->tx_ok;
	out->tx_fail = d->tx_fail;
	out->tx_nack = d->tx_nack;
}

/* -------------------------------------------------------------------------- */
/* Init sequence (handoff spec steps 1–12)                                    */
/* -------------------------------------------------------------------------- */

static bool mcp_rail_init_hw(mcp2518_dev_t *d)
{
	memset(&d->init_diag, 0, sizeof(d->init_diag));

	/* 1. SPI RESET → Configuration mode. */
	mcp_spi_reset(d);

	/* 2. Wait OSCRDY (PLLEN=0 — do not wait PLLRDY). */
	if (!mcp_wait_oscrdy(d)) {
		d->init_diag.fail_bits |= MCP_INIT_FAIL_OSC_READY;
		return false;
	}

	uint32_t osc = mcp_read32(d, REG_OSC);
	d->init_diag.osc_b0 = (uint8_t)(osc & 0xFFu);
	d->init_diag.osc_b1 = (uint8_t)((osc >> 8) & 0xFFu);

	if (!mcp_read_devid(d, &d->init_diag.devid, &d->init_diag.rev)) {
		d->init_diag.fail_bits |= MCP_INIT_FAIL_DEVID;
		return false;
	}

	mcp_ecc_enable(d);
	mcp_ram_init(d);

	/* 3. Configuration mode (OPMOD=4). */
	if (!mcp_enter_config(d)) {
		d->init_diag.fail_bits |= MCP_INIT_FAIL_ENTER_CFG;
		return false;
	}

	/* 4. 20 MHz crystal, PLLEN=0. */
	mcp_config_osc_20mhz_xtal(d);

	/* 5. TXQ enable + CiTXQCON. */
	mcp_enable_txq(d);
	mcp_config_txq(d);

	/* 6. CiNBTCFG @ 1 Mbps + readback gate. */
	mcp_config_bitrate_1mbps(d);
	if (!mcp_nbtcfg_readback_ok(d, &d->init_diag.nbt_brp, &d->init_diag.nbt_tseg1)) {
		d->init_diag.fail_bits |= MCP_INIT_FAIL_NBT_READBACK;
		return false;
	}

	/* 7. CiCON classic CAN 2.0. */
	mcp_config_c1con_classic(d);

	/* 8. RX FIFO + accept-all filter. */
	mcp_config_rx_fifo(d);
	mcp_config_filters_accept_all(d);

	/* 9. IOCON. */
	mcp_config_iocon(d);

	/* Optional: external loopback proves MCP2518 TX/RX without MCP2562. */
	if (mcp_ext_loopback_test(d)) {
		d->init_diag.ext_loopback_ok = 1u;
	} else {
		d->init_diag.fail_bits |= MCP_INIT_FAIL_EXT_LB;
		/* Non-fatal for bench — continue to real bus. */
	}

	/* Re-apply TXQ/RX after loopback returned to config. */
	mcp_enable_txq(d);
	mcp_config_txq(d);
	mcp_config_rx_fifo(d);
	mcp_config_filters_accept_all(d);
	mcp_config_iocon(d);
	mcp_txq_force_ready(d);

	/* 10. Normal CAN 2.0 (REQOP=6, OPMOD=6). */
	if (!mcp_enter_normal_can20(d)) {
		d->init_diag.fail_bits |= MCP_INIT_FAIL_ENTER_NORMAL;
		return false;
	}

	d->init_diag.opmod = mcp_opmod_read(d);

	/* 11. RX interrupt to INT pin (PB10). */
	mcp_enable_rx_irq(d);

	/* 12. TXQ must have space before marking initialized. */
	if (!mcp_txq_has_space(d)) {
		d->init_diag.fail_bits |= MCP_INIT_FAIL_TXQ_SPACE;
		return false;
	}

	return true;
}

bool mcp2518_init_all(void)
{
	bool any_ok = false;

	spi_can_port_init();
	spi_can_port_irq_init();

	for (uint8_t r = 0; r < MCP2518_RAIL_COUNT; r++) {
		if (mcp2518_reinit_rail((can_bus_id_t)(CAN_BUS_CH4 + r)))
			any_ok = true;
	}
	return any_ok;
}

bool mcp2518_reinit_rail(can_bus_id_t bus)
{
	if (bus < CAN_BUS_CH4)
		return false;

	uint8_t rail = (uint8_t)(bus - CAN_BUS_CH4);
	mcp2518_dev_t *d = &g_dev[rail];

	d->rail = rail;
	d->rx_irq_pending = false;
	d->tx_ok = 0u;
	d->tx_fail = 0u;
	d->tx_nack = 0u;
	d->initialized = false;

	if (!mcp_rail_init_hw(d))
		return false;

	d->initialized = true;
	return true;
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

	return mcp_opmod_read(&g_dev[rail]);
}

void mcp2518_isr_rx_pending(uint8_t rail)
{
	if (rail < MCP2518_RAIL_COUNT)
		g_dev[rail].rx_irq_pending = true;
}

/* -------------------------------------------------------------------------- */
/* TX / RX API                                                                */
/* -------------------------------------------------------------------------- */

bool mcp2518_send(can_bus_id_t bus, const can_frame_t *frame)
{
	if (bus < CAN_BUS_CH4 || frame == NULL || frame->dlc > CAN_MAX_DATA_LEN)
		return false;

	uint8_t rail = (uint8_t)(bus - CAN_BUS_CH4);
	mcp2518_dev_t *d = &g_dev[rail];

	if (!d->initialized)
		return false;

	uint8_t tec_before = 0u;
	mcp_read_trec(d, &tec_before, NULL);
	if (tec_before >= 128u)
		(void)mcp_recover_bus_off(d);

	mcp_txq_force_ready(d);

	for (int attempt = 0; attempt < 32; attempt++) {
		if (mcp_txq_has_space(d))
			break;

		mcp_txq_force_ready(d);

		if (attempt == 31) {
			if (d->tx_fail < 0xFFu)
				d->tx_fail++;
			return false;
		}
		HAL_Delay(1);
	}

	if (!mcp_hw_txq_load(d, frame)) {
		if (d->tx_fail < 0xFFu)
			d->tx_fail++;
		return false;
	}

	mcp_txq_done_t done = { 0 };
	if (!mcp_txq_wait_done(d, 50u, tec_before, &done)) {
		mcp_txq_force_ready(d);
		if (d->tx_fail < 0xFFu)
			d->tx_fail++;
		return false;
	}

	if (d->tx_ok < 0xFFu)
		d->tx_ok++;

	mcp_note_tx_result(d, tec_before);
	can_router_mark_traffic(bus);
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

	if (!mcp_hw_pop_rx(d, frame))
		return false;

	d->rx_irq_pending = false;
	return true;
}

void mcp2518_drain_rx(can_bus_id_t bus)
{
	if (bus < CAN_BUS_CH4)
		return;

	uint8_t rail = (uint8_t)(bus - CAN_BUS_CH4);
	mcp2518_dev_t *d = &g_dev[rail];

	if (!d->initialized)
		return;

	d->rx_irq_pending = false;

	while (mcp_hw_pop_rx(d, NULL))
		;
}

void mcp2518_poll_rx(can_bus_id_t bus)
{
	if (bus < CAN_BUS_CH4)
		return;

	uint8_t rail = (uint8_t)(bus - CAN_BUS_CH4);
	mcp2518_dev_t *d = &g_dev[rail];

	if (!d->initialized)
		return;

	if (g_dev[rail].rx_irq_pending ||
	    spi_can_port_int_active(rail) ||
	    mcp_rx_fifo_not_empty(d))
		g_dev[rail].rx_irq_pending = true;
}

void mcp2518_reset_tx_stats(can_bus_id_t bus)
{
	if (bus < CAN_BUS_CH4)
		return;

	mcp2518_dev_t *d = &g_dev[(uint8_t)(bus - CAN_BUS_CH4)];

	d->tx_ok = 0u;
	d->tx_fail = 0u;
	d->tx_nack = 0u;
}

void mcp2518_prepare_tx(can_bus_id_t bus)
{
	if (bus < CAN_BUS_CH4)
		return;

	uint8_t rail = (uint8_t)(bus - CAN_BUS_CH4);
	mcp2518_dev_t *d = &g_dev[rail];

	if (!d->initialized)
		return;

	uint8_t tec = 0u;
	mcp_read_trec(d, &tec, NULL);
	if (tec >= 128u)
		(void)mcp_recover_bus_off(d);

	mcp_txq_force_ready(d);
}

void mcp2518_get_tx_stats(uint8_t rail, uint8_t *tx_ok, uint8_t *tx_fail, uint8_t *tx_nack)
{
	if (rail >= MCP2518_RAIL_COUNT)
		return;

	if (tx_ok != NULL)
		*tx_ok = g_dev[rail].tx_ok;
	if (tx_fail != NULL)
		*tx_fail = g_dev[rail].tx_fail;
	if (tx_nack != NULL)
		*tx_nack = g_dev[rail].tx_nack;
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

	return mcp_recover_bus_off(&g_dev[(uint8_t)(bus - CAN_BUS_CH4)]);
}

bool mcp2518_recover_if_busoff(can_bus_id_t bus)
{
	if (bus < CAN_BUS_CH4)
		return true;

	uint8_t rail = (uint8_t)(bus - CAN_BUS_CH4);
	mcp2518_dev_t *d = &g_dev[rail];

	if (!d->initialized)
		return false;

	uint8_t tec = 0u;
	mcp_read_trec(d, &tec, NULL);
	if (tec < 128u)
		return true;

	return mcp_recover_bus_off(d);
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

	uint8_t tec_pre = 0u;
	mcp_read_trec(d, &tec_pre, NULL);
	if (tec_pre >= 128u)
		(void)mcp_recover_bus_off(d);

	d->tx_ok = 0u;
	d->tx_fail = 0u;
	d->tx_nack = 0u;

	mcp2518_drain_rx(bus);
	mcp_fill_smoke_diag(d, out);
	out->tec_before = tec_pre;

	if (mcp2518_send(bus, frame)) {
		out->tx_ok = d->tx_ok;
		out->tx_nack = d->tx_nack;
	} else {
		out->tx_fail = d->tx_fail;
	}

	mcp_read_trec(d, &out->tec, &out->rec);
	out->tec_before = tec_pre;

	if (out->tec > tec_pre) {
		uint8_t nack_by_tec = mcp_tec_nack_count(tec_pre, out->tec);
		if (nack_by_tec > out->tx_nack)
			out->tx_nack = nack_by_tec;
	}

	can_frame_t rx;
	for (uint16_t i = 0; i < listen_ms; i++) {
		mcp2518_poll_rx(bus);
		while (mcp2518_recv(bus, &rx)) {
			if (out->rx_frames < 0xFFu)
				out->rx_frames++;
		}
		plant_diag_yield_usb();
		HAL_Delay(1);
	}

	for (uint8_t settle = 0; settle < 30u; settle++) {
		mcp_read_trec(d, &out->tec, &out->rec);
		if (out->tec > tec_pre)
			break;
		HAL_Delay(1);
	}

	if (out->tec > tec_pre) {
		uint8_t nack_by_tec = mcp_tec_nack_count(tec_pre, out->tec);
		if (nack_by_tec > out->tx_nack)
			out->tx_nack = nack_by_tec;
	}

	mcp_fill_smoke_diag(d, out);
	out->tec_before = tec_pre;

	return (out->tx_ok > 0u &&
	        out->nbt_brp == MCP2518_NBT_BRP_EXPECT &&
	        out->nbt_tseg1 == MCP2518_NBT_TSEG1_EXPECT);
}
