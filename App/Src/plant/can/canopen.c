#include "plant/can/canopen.h"
#include "plant/can/mcp2518fd.h"
#include "plant/can/spi_can_router.h"
#include "main.h"
#include <string.h>

void canopen_frame_std(can_frame_t *out, uint16_t cob_id, uint8_t dlc,
                       const uint8_t *data)
{
	if (out == NULL)
		return;

	memset(out, 0, sizeof(*out));
	out->id_type = CAN_ID_STD;
	out->id = cob_id & CAN_STD_ID_MASK;
	out->dlc = (dlc > CAN_MAX_DATA_LEN) ? CAN_MAX_DATA_LEN : dlc;
	if (data != NULL && out->dlc > 0u)
		memcpy(out->data, data, out->dlc);
}

/* MCP2518 SPI-CAN rails (CH4-6): queued enqueue+flush from this blocking bench
 * context was observed to stall (see damiao_probe_tx_now / robstride probes) —
 * bypass the software TX queue with the same prepare+send_now pattern so
 * ZeroErr SDO/NMT bench traffic works on MCP rails, not only FDCAN CH1-3. */
static bool canopen_tx_now(can_bus_id_t bus, const can_frame_t *frame)
{
	if (frame == NULL)
		return false;

	if (bus >= CAN_BUS_CH4) {
		mcp2518_prepare_tx(bus);
		if (spi_can_router_send_now(bus, frame))
			return true;
		mcp2518_prepare_tx(bus);
		return spi_can_router_send_now(bus, frame);
	}

	if (can_tx_enqueue(bus, frame) != CAN_OK)
		return false;
	(void)can_tx_flush(bus);
	return true;
}

bool canopen_nmt_send(can_bus_id_t bus, uint8_t cs, uint8_t node_id)
{
	uint8_t data[2] = { cs, node_id & 0x7Fu };
	can_frame_t frame;

	canopen_frame_std(&frame, CANOPEN_NMT_COB, 2u, data);
	return canopen_tx_now(bus, &frame);
}

bool canopen_sync_send(can_bus_id_t bus)
{
	can_frame_t frame;

	canopen_frame_std(&frame, CANOPEN_SYNC_COB, 0u, NULL);
	return canopen_tx_now(bus, &frame);
}

static bool canopen_sdo_transact(can_bus_id_t bus, uint8_t node,
                                 const uint8_t req[8], uint8_t *resp,
                                 uint32_t timeout_ms)
{
	can_frame_t tx;
	uint16_t expect_id = canopen_cob_sdo_tx(node);
	uint32_t deadline;

	if (req == NULL)
		return false;

	canopen_frame_std(&tx, canopen_cob_sdo_rx(node), 8u, req);
	if (!canopen_tx_now(bus, &tx))
		return false;

	deadline = HAL_GetTick() + (timeout_ms == 0u ? 1u : timeout_ms);

	while ((int32_t)(HAL_GetTick() - deadline) < 0) {
		can_frame_t rx;

		can_router_poll_bus(bus);
		while (can_rx_pop(bus, &rx) == CAN_OK) {
			if (rx.id_type != CAN_ID_STD)
				continue;
			if ((rx.id & CAN_STD_ID_MASK) != expect_id)
				continue;
			if (rx.dlc < 8u)
				continue;
			if (resp != NULL)
				memcpy(resp, rx.data, 8u);
			if (rx.data[0] == CANOPEN_SDO_ABORT)
				return false;
			return true;
		}
	}

	return false;
}

bool canopen_sdo_write_u8(can_bus_id_t bus, uint8_t node, uint16_t index,
                          uint8_t sub, uint8_t value, uint32_t timeout_ms)
{
	uint8_t req[8] = {
		CANOPEN_SDO_CMD_W1,
		(uint8_t)(index & 0xFFu),
		(uint8_t)(index >> 8),
		sub,
		value, 0, 0, 0
	};
	uint8_t resp[8];

	if (!canopen_sdo_transact(bus, node, req, resp, timeout_ms))
		return false;
	return resp[0] == CANOPEN_SDO_RESP_W;
}

bool canopen_sdo_write_u16(can_bus_id_t bus, uint8_t node, uint16_t index,
                           uint8_t sub, uint16_t value, uint32_t timeout_ms)
{
	uint8_t req[8] = {
		CANOPEN_SDO_CMD_W2,
		(uint8_t)(index & 0xFFu),
		(uint8_t)(index >> 8),
		sub,
		(uint8_t)(value & 0xFFu),
		(uint8_t)(value >> 8),
		0, 0
	};
	uint8_t resp[8];

	if (!canopen_sdo_transact(bus, node, req, resp, timeout_ms))
		return false;
	return resp[0] == CANOPEN_SDO_RESP_W;
}

bool canopen_sdo_write_u32(can_bus_id_t bus, uint8_t node, uint16_t index,
                           uint8_t sub, uint32_t value, uint32_t timeout_ms)
{
	uint8_t req[8] = {
		CANOPEN_SDO_CMD_W4,
		(uint8_t)(index & 0xFFu),
		(uint8_t)(index >> 8),
		sub,
		(uint8_t)(value & 0xFFu),
		(uint8_t)((value >> 8) & 0xFFu),
		(uint8_t)((value >> 16) & 0xFFu),
		(uint8_t)((value >> 24) & 0xFFu)
	};
	uint8_t resp[8];

	if (!canopen_sdo_transact(bus, node, req, resp, timeout_ms))
		return false;
	return resp[0] == CANOPEN_SDO_RESP_W;
}

bool canopen_sdo_read_u32(can_bus_id_t bus, uint8_t node, uint16_t index,
                          uint8_t sub, uint32_t *out, uint32_t timeout_ms)
{
	uint8_t req[8] = {
		CANOPEN_SDO_CMD_R,
		(uint8_t)(index & 0xFFu),
		(uint8_t)(index >> 8),
		sub,
		0, 0, 0, 0
	};
	uint8_t resp[8];
	uint32_t val;

	if (out == NULL)
		return false;
	if (!canopen_sdo_transact(bus, node, req, resp, timeout_ms))
		return false;

	if (resp[0] != CANOPEN_SDO_RESP_R1 &&
	    resp[0] != CANOPEN_SDO_RESP_R2 &&
	    resp[0] != CANOPEN_SDO_RESP_R4)
		return false;

	val = (uint32_t)resp[4] |
	      ((uint32_t)resp[5] << 8) |
	      ((uint32_t)resp[6] << 16) |
	      ((uint32_t)resp[7] << 24);
	*out = val;
	return true;
}

void canopen_pack_rxpdo1_cw_pos(can_frame_t *out, uint8_t node,
                                uint16_t controlword, int32_t target_pos)
{
	uint8_t data[6];

	data[0] = (uint8_t)(controlword & 0xFFu);
	data[1] = (uint8_t)(controlword >> 8);
	data[2] = (uint8_t)((uint32_t)target_pos & 0xFFu);
	data[3] = (uint8_t)(((uint32_t)target_pos >> 8) & 0xFFu);
	data[4] = (uint8_t)(((uint32_t)target_pos >> 16) & 0xFFu);
	data[5] = (uint8_t)(((uint32_t)target_pos >> 24) & 0xFFu);
	canopen_frame_std(out, canopen_cob_rxpdo1(node), 6u, data);
}

bool canopen_parse_txpdo1_sw_pos(const can_frame_t *in, uint8_t node,
                                 uint16_t *statusword, int32_t *actual_pos)
{
	uint32_t raw;

	if (in == NULL || statusword == NULL || actual_pos == NULL)
		return false;
	if (in->id_type != CAN_ID_STD)
		return false;
	if ((in->id & CAN_STD_ID_MASK) != canopen_cob_txpdo1(node))
		return false;
	if (in->dlc < 6u)
		return false;

	*statusword = (uint16_t)in->data[0] | ((uint16_t)in->data[1] << 8);
	raw = (uint32_t)in->data[2] |
	      ((uint32_t)in->data[3] << 8) |
	      ((uint32_t)in->data[4] << 16) |
	      ((uint32_t)in->data[5] << 24);
	*actual_pos = (int32_t)raw;
	return true;
}
