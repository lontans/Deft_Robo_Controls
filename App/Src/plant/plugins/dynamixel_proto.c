#include <stdbool.h>
#include <string.h>
#include "plant/plugins/dynamixel.h"

#define DXL_TX_BUF_SIZE 128
#define DXL_RX_BUF_SIZE 128

static uint8_t g_dxl_tx[DXL_TX_BUF_SIZE];
static uint8_t g_dxl_rx[DXL_RX_BUF_SIZE];

static const uint32_t g_dxl_baud_candidates[] = {
	57600u,    /* factory default (control table 8 = 1) */
	9600u,
	1000000u,
	115200u,
	2000000u,
	3000000u,
	4000000u,
};

static unsigned short updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr,
                                uint16_t data_blk_size);

static bool dxl_send_packet(uint8_t *pkt, uint16_t body_len)
{
	uint16_t total = (uint16_t)(7u + body_len);
	uint16_t crc;

	if (total > DXL_TX_BUF_SIZE)
		return false;

	pkt[DXL_HDR0]     = 0xFF;
	pkt[DXL_HDR1]     = 0xFF;
	pkt[DXL_HDR2]     = 0xFD;
	pkt[DXL_RESERVED] = 0x00;
	pkt[DXL_LEN_L]    = DXL_LOBYTE(body_len);
	pkt[DXL_LEN_H]    = DXL_HIBYTE(body_len);

	crc = updateCRC(0, pkt, (uint16_t)(total - 2u));
	pkt[total - 2u] = DXL_LOBYTE(crc);
	pkt[total - 1u] = DXL_HIBYTE(crc);

	dxl_port_flush_rx();
	return dxl_port_write(pkt, (int)total) == (int)total;
}

static bool dxl_recv_status_packet(uint8_t *buf, uint16_t *out_total,
                                   uint32_t timeout_ms)
{
	uint32_t t0 = dxl_port_millis();
	uint16_t got = 0;

	while ((dxl_port_millis() - t0) < timeout_ms) {
		uint8_t b;
		int hdr = -1;
		uint16_t i;
		uint16_t body_len;
		uint16_t need;
		uint16_t crc_rx;

		if (dxl_port_read_byte(&b, 1) != 1)
			continue;

		if (got < DXL_RX_BUF_SIZE)
			buf[got++] = b;

		if (got < 11u)
			continue;

		for (i = 0; i + 3u < got; i++) {
			if (buf[i] == 0xFF && buf[i + 1] == 0xFF &&
			    buf[i + 2] == 0xFD && buf[i + 3] != 0xFD) {
				hdr = (int)i;
				break;
			}
		}

		if (hdr < 0) {
			if (got > 3u) {
				buf[0] = buf[got - 3u];
				buf[1] = buf[got - 2u];
				buf[2] = buf[got - 1u];
				got = 3u;
			}
			continue;
		}

		if (hdr > 0) {
			memmove(buf, &buf[hdr], got - (uint16_t)hdr);
			got = (uint16_t)(got - (uint16_t)hdr);
		}

		body_len = (uint16_t)buf[DXL_LEN_L] |
		           ((uint16_t)buf[DXL_LEN_H] << 8);
		need = (uint16_t)(7u + body_len);

		if (need > DXL_RX_BUF_SIZE)
			return false;
		if (got < need)
			continue;

		if (buf[DXL_RESERVED] != 0x00)
			return false;
		if (buf[DXL_STATUS] != DXL_STATUS_PKT)
			return false;

		crc_rx = (uint16_t)buf[need - 2u] |
		         ((uint16_t)buf[need - 1u] << 8);
		if (updateCRC(0, buf, (uint16_t)(need - 2u)) != crc_rx)
			return false;

		*out_total = need;
		return true;
	}

	return false;
}

static bool dxl_ping_ex(uint8_t id, uint16_t *model_out)
{
	uint16_t rx_len = 0;

	g_dxl_tx[DXL_ID]   = id;
	g_dxl_tx[DXL_INST] = DXL_INST_PING;

	if (!dxl_send_packet(g_dxl_tx, 3u))
		return false;
	if (!dxl_recv_status_packet(g_dxl_rx, &rx_len, DXL_RX_TIMEOUT_MS))
		return false;
	if (g_dxl_rx[DXL_ID] != id)
		return false;
	if (g_dxl_rx[DXL_ERR] != 0u)
		return false;

	if (model_out != NULL)
		*model_out = (uint16_t)g_dxl_rx[9] |
		             ((uint16_t)g_dxl_rx[10] << 8);

	return true;
}

bool dxl_ping(uint8_t id)
{
	return dxl_ping_ex(id, NULL);
}

uint16_t dxl_ping_model_number(uint8_t id)
{
	uint16_t model = 0u;

	if (!dxl_ping_ex(id, &model))
		return 0u;

	return model;
}

bool dxl_write_u8(uint8_t id, uint16_t addr, uint8_t value)  // Helper function for ID migration, writes to EEPROM addr 8
{
	uint16_t rx_len = 0;

	g_dxl_tx[DXL_ID]		  = id;
	g_dxl_tx[DXL_INST]	      = DXL_INST_WRITE;
	g_dxl_tx[DXL_PARAM0]	  = DXL_LOBYTE(addr);
	g_dxl_tx[DXL_PARAM0 + 1u] = DXL_HIBYTE(addr);
	g_dxl_tx[DXL_PARAM0 + 2u] = value;

	if (!dxl_send_packet(g_dxl_tx, 6u))
		return false;
	if (!dxl_recv_status_packet(g_dxl_rx, &rx_len, DXL_RX_TIMEOUT_MS))
		return false;
	if (g_dxl_rx[DXL_ID] != id)
		return false;
	if (g_dxl_rx[DXL_ERR] != 0u)
		return false;

	return true;
}

bool dxl_write_u16(uint8_t id, uint16_t addr, uint16_t value)
{
	uint16_t rx_len = 0;

	g_dxl_tx[DXL_ID]            = id;
	g_dxl_tx[DXL_INST]          = DXL_INST_WRITE;
	g_dxl_tx[DXL_PARAM0]        = DXL_LOBYTE(addr);
	g_dxl_tx[DXL_PARAM0 + 1u]   = DXL_HIBYTE(addr);
	g_dxl_tx[DXL_PARAM0 + 2u]   = DXL_LOBYTE(value);
	g_dxl_tx[DXL_PARAM0 + 3u]   = DXL_HIBYTE(value);

	if (!dxl_send_packet(g_dxl_tx, 7u))
		return false;
	if (!dxl_recv_status_packet(g_dxl_rx, &rx_len, DXL_RX_TIMEOUT_MS))
		return false;
	if (g_dxl_rx[DXL_ID] != id)
		return false;
	if (g_dxl_rx[DXL_ERR] != 0u)
		return false;

	return true;
}

bool dxl_write_u32(uint8_t id, uint16_t addr, uint32_t value)
{
	uint16_t rx_len = 0;

	g_dxl_tx[DXL_ID]            = id;
	g_dxl_tx[DXL_INST]          = DXL_INST_WRITE;
	g_dxl_tx[DXL_PARAM0]        = DXL_LOBYTE(addr);
	g_dxl_tx[DXL_PARAM0 + 1u]   = DXL_HIBYTE(addr);
	g_dxl_tx[DXL_PARAM0 + 2u]   = (uint8_t)(value & 0xFFu);
	g_dxl_tx[DXL_PARAM0 + 3u]   = (uint8_t)((value >> 8) & 0xFFu);
	g_dxl_tx[DXL_PARAM0 + 4u]   = (uint8_t)((value >> 16) & 0xFFu);
	g_dxl_tx[DXL_PARAM0 + 5u]   = (uint8_t)((value >> 24) & 0xFFu);

	if (!dxl_send_packet(g_dxl_tx, 9u))
		return false;
	if (!dxl_recv_status_packet(g_dxl_rx, &rx_len, DXL_RX_TIMEOUT_MS))
		return false;
	if (g_dxl_rx[DXL_ID] != id)
		return false;
	if (g_dxl_rx[DXL_ERR] != 0u)
		return false;

	return true;
}

bool dxl_read_u32(uint8_t id, uint16_t addr, uint32_t *value_out)
{
	uint16_t rx_len = 0;

	if (value_out == NULL)
		return false;

	g_dxl_tx[DXL_ID]            = id;
	g_dxl_tx[DXL_INST]          = DXL_INST_READ;
	g_dxl_tx[DXL_PARAM0]        = DXL_LOBYTE(addr);
	g_dxl_tx[DXL_PARAM0 + 1u]   = DXL_HIBYTE(addr);
	g_dxl_tx[DXL_PARAM0 + 2u]   = 4u;
	g_dxl_tx[DXL_PARAM0 + 3u]   = 0u;

	if (!dxl_send_packet(g_dxl_tx, 7u))
		return false;
	if (!dxl_recv_status_packet(g_dxl_rx, &rx_len, DXL_RX_TIMEOUT_MS))
		return false;
	if (g_dxl_rx[DXL_ID] != id)
		return false;
	if (g_dxl_rx[DXL_ERR] != 0u)
		return false;

	*value_out = (uint32_t)g_dxl_rx[9]
	           | ((uint32_t)g_dxl_rx[10] << 8)
	           | ((uint32_t)g_dxl_rx[11] << 16)
	           | ((uint32_t)g_dxl_rx[12] << 24);

	return true;
}

static bool dxl_any_id_pings(uint8_t id_start, uint8_t id_end)
{
	uint16_t id;

	for (id = id_start; id <= id_end; id++) {
		if (dxl_ping((uint8_t)id))
			return true;
	}

	return false;
}

static bool dxl_all_ids_ping(uint8_t id_start, uint8_t id_end)
{
	uint16_t id;

	for (id = id_start; id <= id_end; id++) {
		if (!dxl_ping((uint8_t)id))
			return false;
	}

	return true;
}

/* Probe candidate list until any ID in range replies; returns wire baud in *baud_out. */
static bool dxl_find_working_baud(uint8_t id_start, uint8_t id_end,
                                  uint32_t *baud_out)
{
	size_t n_candidates = sizeof(g_dxl_baud_candidates) /
	                      sizeof(g_dxl_baud_candidates[0]);
	size_t i;

	if (baud_out == NULL)
		return false;

	for (i = 0; i < n_candidates; i++) {
		if (!dxl_port_set_baud(g_dxl_baud_candidates[i]))
			continue;

		dxl_port_flush_rx();
		dxl_port_delay_ms(DXL_BAUD_SETTLE_MS);

		if (dxl_any_id_pings(id_start, id_end)) {
			*baud_out = g_dxl_baud_candidates[i];
			return true;
		}
	}

	return false;
}

/*
 * Toggle bus between 1M and 2M (XL330 addr 8 indices 3 and 4).
 * If currently at 1M -> 2M; at 2M -> 1M; anything else -> 2M.
 * Uses candidate scan to find current wire baud before writing EEPROM.
 */
bool dxl_toggle_ids_baud(uint8_t id_start, uint8_t id_end, uint32_t *new_baud_out)
{
	uint32_t current_baud = 0u;
	uint32_t target_baud;
	uint8_t target_num;
	uint16_t id;

	if (id_start < 1u)
		id_start = 1u;
	if (id_end > 253u)
		id_end = 253u;
	if (id_start > id_end)
		return false;

	if (!dxl_find_working_baud(id_start, id_end, &current_baud))
		return false;

	if (current_baud == DXL_BAUD_TOGGLE_1M) {
		target_baud = DXL_BAUD_TOGGLE_2M;
		target_num  = DXL_BAUD_NUM_2M;
	} else if (current_baud == DXL_BAUD_TOGGLE_2M) {
		target_baud = DXL_BAUD_TOGGLE_1M;
		target_num  = DXL_BAUD_NUM_1M;
	} else {
		target_baud = DXL_BAUD_TOGGLE_2M;
		target_num  = DXL_BAUD_NUM_2M;
	}

	for (id = id_start; id <= id_end; id++) {
		if (!dxl_write_u8((uint8_t)id, DXL_ADDR_BAUD_RATE, target_num))
			return false;
		dxl_port_delay_ms(10u);
	}

	if (!dxl_port_set_baud(target_baud))
		return false;

	if (!dxl_all_ids_ping(id_start, id_end))
		return false;

	if (new_baud_out != NULL)
		*new_baud_out = target_baud;

	return true;
}

static bool dxl_broadcast_ping_any(void)
{
	uint16_t rx_len = 0;

	g_dxl_tx[DXL_ID]   = DXL_ID_BROADCAST;
	g_dxl_tx[DXL_INST] = DXL_INST_PING;

	if (!dxl_send_packet(g_dxl_tx, 3u))
		return false;
	if (!dxl_recv_status_packet(g_dxl_rx, &rx_len, DXL_RX_TIMEOUT_MS))
		return false;
	if (g_dxl_rx[DXL_ERR] != 0u)
		return false;

	return true;
}

static bool dxl_ping_id_range(uint8_t id_start, uint8_t id_end)
{
	uint16_t id;

	if (id_start < 1u)
		id_start = 1u;
	if (id_end > 253u)
		id_end = 253u;
	if (id_start > id_end)
		return false;

	for (id = id_start; id <= id_end; id++) {
		if (dxl_ping((uint8_t)id))
			return true;
	}

	return false;
}

bool dxl_find_baud(uint32_t *baud_out, uint8_t id_start, uint8_t id_end)
{
	size_t n_candidates = sizeof(g_dxl_baud_candidates) /
	                      sizeof(g_dxl_baud_candidates[0]);
	size_t i;

	if (baud_out == NULL)
		return false;

	if (id_start < 1u)
		id_start = DXL_BAUD_PROBE_ID_START;
	if (id_end > 253u)
		id_end = DXL_BAUD_PROBE_ID_END;
	if (id_start > id_end) {
		id_start = DXL_BAUD_PROBE_ID_START;
		id_end = DXL_BAUD_PROBE_ID_END;
	}

	for (i = 0; i < n_candidates; i++) {
		if (!dxl_port_set_baud(g_dxl_baud_candidates[i]))
			continue;

		dxl_port_flush_rx();
		dxl_port_delay_ms(DXL_BAUD_SETTLE_MS);

		if (dxl_broadcast_ping_any()) {
			*baud_out = g_dxl_baud_candidates[i];
			return true;
		}

		/* Unicast fallback when broadcast gets no reply (common on reused servos).
		 * Always run this even for wide host ID scans; cap IDs probed per baud. */
		{
			uint8_t seq_end = id_end;
			uint16_t span = (uint16_t)(id_end - id_start + 1u);

			if (span > DXL_BAUD_SEQ_PROBE_MAX)
				seq_end = (uint8_t)(id_start + DXL_BAUD_SEQ_PROBE_MAX - 1u);

			if (dxl_ping_id_range(id_start, seq_end)) {
				*baud_out = g_dxl_baud_candidates[i];
				return true;
			}
		}
	}

	return false;
}

uint8_t dxl_scan_ids(uint8_t id_start, uint8_t id_end,
                     dxl_scan_hit_t *hits, uint8_t max_hits)
{
	uint8_t found = 0;
	uint16_t id;

	if (hits == NULL || max_hits == 0)
		return 0;

	if (id_start < 1u)
		id_start = 1u;
	if (id_end > 253u)
		id_end = 253u;

	for (id = id_start; id <= id_end && found < max_hits; id++) {
		uint16_t model = 0u;

		if (!dxl_ping_ex((uint8_t)id, &model))
			continue;

		hits[found].id = (uint8_t)id;
		hits[found].model_number = model;
		found++;
	}

	return found;
}

bool dxl_sync_write_tx(uint16_t start_addr, uint16_t data_len,
                       const uint8_t *param, uint16_t param_len)
{
	/* P2.0 LENGTH = instruction + parameters + CRC16 (same as dxl_write_u8 body 6). */
	uint16_t body_len = (uint16_t)(param_len + 7u);
	uint16_t total    = (uint16_t)(7u + body_len);

	if (total > DXL_TX_BUF_SIZE || param == NULL)
		return false;

	g_dxl_tx[DXL_ID]              = DXL_ID_BROADCAST;
	g_dxl_tx[DXL_INST]            = DXL_INST_SYNC_WRITE;
	g_dxl_tx[DXL_PARAM0 + 0u]     = DXL_LOBYTE(start_addr);
	g_dxl_tx[DXL_PARAM0 + 1u]     = DXL_HIBYTE(start_addr);
	g_dxl_tx[DXL_PARAM0 + 2u]     = DXL_LOBYTE(data_len);
	g_dxl_tx[DXL_PARAM0 + 3u]     = DXL_HIBYTE(data_len);
	memcpy(&g_dxl_tx[DXL_PARAM0 + 4u], param, param_len);

	return dxl_send_packet(g_dxl_tx, body_len);
}

bool dxl_sync_read_tx(uint16_t start_addr, uint16_t data_len,
                      const uint8_t *ids, uint8_t id_count)
{
	uint16_t body_len = (uint16_t)(id_count + 7u);
	uint16_t total    = (uint16_t)(7u + body_len);

	if (total > DXL_TX_BUF_SIZE || ids == NULL || id_count == 0u)
		return false;

	g_dxl_tx[DXL_ID]   = DXL_ID_BROADCAST;
	g_dxl_tx[DXL_INST] = DXL_INST_SYNC_READ;

	g_dxl_tx[DXL_PARAM0 + 0u] = DXL_LOBYTE(start_addr);
	g_dxl_tx[DXL_PARAM0 + 1u] = DXL_HIBYTE(start_addr);
	g_dxl_tx[DXL_PARAM0 + 2u] = DXL_LOBYTE(data_len);
	g_dxl_tx[DXL_PARAM0 + 3u] = DXL_HIBYTE(data_len);

	memcpy(&g_dxl_tx[DXL_PARAM0 + 4u], ids, id_count);

	return dxl_send_packet(g_dxl_tx, body_len);
}


bool dxl_sync_read_rx_one(uint8_t *id_out, uint8_t *data, uint16_t data_len,
					      uint32_t timeout_ms)
{
	uint16_t rx_total = 0u;

	// Block until one full valid status frame
	if (!dxl_recv_status_packet(g_dxl_rx, &rx_total, timeout_ms))
		return false;

	// Byte 4 info abt servo transmission id, DXL_ID is a macro for byte 4 in the rx frame
	if (id_out != NULL)
		*id_out = g_dxl_rx[DXL_ID];

	if (g_dxl_rx[DXL_ERR] != 0u)
		return false;

	if (data != NULL && data_len > 0u)
		memcpy(data, &g_dxl_rx[9], data_len);

	return true;
}

void dynamixel_bus_init(void)
{
	dxl_port_init();
	(void)dxl_port_set_baud(DXL_BAUD_RATE);
}

static unsigned short updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr,
                                uint16_t data_blk_size)
{
	uint16_t i;
	uint16_t j;
	static const uint16_t crc_table[256] = {
		0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
		0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
		0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
		0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
		0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
		0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
		0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
		0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
		0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
		0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
		0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
		0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
		0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
		0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
		0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
		0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
		0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
		0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
		0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
		0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
		0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
		0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
		0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
		0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
		0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
		0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
		0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
		0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
		0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
		0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
		0x8220, 0x0225, 0x022F, 0x822A, 0x823B, 0x023E, 0x0234, 0x8231,
		0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202,
	};

	for (j = 0; j < data_blk_size; j++) {
		i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFFu;
		crc_accum = (uint16_t)((crc_accum << 8) ^ crc_table[i]);
	}

	return crc_accum;
}
