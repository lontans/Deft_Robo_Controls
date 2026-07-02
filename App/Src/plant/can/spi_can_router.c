#include "plant/can/spi_can_router.h"
#include "plant/can/spi_can_port.h"
#include "plant/can/mcp2518fd.h"

#include <string.h>

#define SPI_CAN_QUEUE_DEPTH 128u

typedef struct {
	can_frame_t buf[SPI_CAN_QUEUE_DEPTH];
	uint16_t head;
	uint16_t tail;
} spi_can_rx_ring_t;

typedef struct {
	can_frame_t buf[SPI_CAN_QUEUE_DEPTH];
	uint16_t head;
	uint16_t tail;
} spi_can_tx_queue_t;

static spi_can_rx_ring_t rx_rings[CAN_MCP2518_COUNT];
static spi_can_tx_queue_t tx_queues[CAN_MCP2518_COUNT];

static can_bus_id_t spi_rail_to_bus(uint8_t rail)
{
	return (can_bus_id_t)(CAN_BUS_CH4 + rail);
}

static uint8_t spi_bus_to_rail(can_bus_id_t bus)
{
	return (uint8_t)(bus - CAN_BUS_CH4);
}

static void spi_rx_push(can_bus_id_t bus, const can_frame_t *frame)
{
	uint8_t rail = spi_bus_to_rail(bus);

	if ((rx_rings[rail].head + 1u) % SPI_CAN_QUEUE_DEPTH == rx_rings[rail].tail)
		rx_rings[rail].tail = (rx_rings[rail].tail + 1u) % SPI_CAN_QUEUE_DEPTH;

	rx_rings[rail].buf[rx_rings[rail].head] = *frame;
	rx_rings[rail].head = (rx_rings[rail].head + 1u) % SPI_CAN_QUEUE_DEPTH;
}

static can_status_t spi_backend_send(can_bus_id_t bus, const can_frame_t *frame)
{
	if (!spi_can_bus_valid(bus) || frame == NULL)
		return CAN_ERR_PARAM;

	if (!mcp2518_send(bus, frame))
		return CAN_ERR_HAL;

	can_router_mark_traffic(bus);
	return CAN_OK;
}

void spi_can_router_init(void)
{
	for (uint8_t r = 0; r < CAN_MCP2518_COUNT; r++) {
		rx_rings[r].head = 0;
		rx_rings[r].tail = 0;
		tx_queues[r].head = 0;
		tx_queues[r].tail = 0;
	}

	(void)mcp2518_init_all();
}

can_status_t spi_can_router_tx_enqueue(can_bus_id_t bus, const can_frame_t *frame)
{
	if (!spi_can_bus_valid(bus) || frame == NULL)
		return CAN_ERR_PARAM;

	uint8_t rail = spi_bus_to_rail(bus);

	if (((tx_queues[rail].head + 1u) % SPI_CAN_QUEUE_DEPTH) == tx_queues[rail].tail)
		return CAN_ERR_FULL;

	tx_queues[rail].buf[tx_queues[rail].head] = *frame;
	tx_queues[rail].head = (tx_queues[rail].head + 1u) % SPI_CAN_QUEUE_DEPTH;
	return CAN_OK;
}

can_status_t spi_can_router_tx_flush(can_bus_id_t bus)
{
	if (!spi_can_bus_valid(bus))
		return CAN_ERR_PARAM;

	uint8_t rail = spi_bus_to_rail(bus);

	if (tx_queues[rail].head == tx_queues[rail].tail)
		return CAN_ERR_EMPTY;

	can_status_t status = spi_backend_send(bus, &tx_queues[rail].buf[tx_queues[rail].tail]);
	if (status != CAN_OK) {
		mcp2518_prepare_tx(bus);
		status = spi_backend_send(bus, &tx_queues[rail].buf[tx_queues[rail].tail]);
	}

	if (status != CAN_OK) {
		tx_queues[rail].tail = (tx_queues[rail].tail + 1u) % SPI_CAN_QUEUE_DEPTH;
		return status;
	}

	tx_queues[rail].tail = (tx_queues[rail].tail + 1u) % SPI_CAN_QUEUE_DEPTH;
	return CAN_OK;
}

bool spi_can_router_send_now(can_bus_id_t bus, const can_frame_t *frame)
{
	return spi_backend_send(bus, frame) == CAN_OK;
}

can_status_t spi_can_router_rx_pop(can_bus_id_t bus, can_frame_t *frame)
{
	if (!spi_can_bus_valid(bus) || frame == NULL)
		return CAN_ERR_PARAM;

	uint8_t rail = spi_bus_to_rail(bus);

	if (rx_rings[rail].head == rx_rings[rail].tail)
		return CAN_ERR_EMPTY;

	*frame = rx_rings[rail].buf[rx_rings[rail].tail];
	rx_rings[rail].tail = (rx_rings[rail].tail + 1u) % SPI_CAN_QUEUE_DEPTH;
	return CAN_OK;
}

void spi_can_router_rx_drain(can_bus_id_t bus)
{
	can_frame_t junk;

	if (!spi_can_bus_valid(bus))
		return;

	while (spi_can_router_rx_pop(bus, &junk) == CAN_OK)
		;
}

bool spi_can_router_rx_available(can_bus_id_t bus)
{
	if (!spi_can_bus_valid(bus))
		return false;

	uint8_t rail = spi_bus_to_rail(bus);
	return rx_rings[rail].head != rx_rings[rail].tail;
}

void spi_can_router_discard_pending_tx(void)
{
	for (uint8_t r = 0; r < CAN_MCP2518_COUNT; r++) {
		tx_queues[r].head = 0;
		tx_queues[r].tail = 0;
	}
}

static void spi_poll_rx_one(can_bus_id_t bus)
{
	uint8_t rail = spi_bus_to_rail(bus);

	if (spi_can_port_int_active(rail))
		mcp2518_isr_rx_pending(rail);

	/* Always try HW RX FIFO — do not rely on INT pin alone. */
	can_frame_t temp;
	while (mcp2518_recv(bus, &temp)) {
		spi_rx_push(bus, &temp);
		can_router_mark_traffic(bus);
	}
}

static void spi_poll_one(can_bus_id_t bus)
{
	spi_poll_rx_one(bus);

	while (spi_can_router_tx_flush(bus) == CAN_OK)
		;
}

void spi_can_router_poll_bus_rx(can_bus_id_t bus)
{
	if (!spi_can_bus_valid(bus))
		return;

	spi_poll_rx_one(bus);
}

void spi_can_router_poll_bus(can_bus_id_t bus)
{
	if (!spi_can_bus_valid(bus))
		return;

	spi_poll_one(bus);
}

void spi_can_router_poll(void)
{
	for (uint8_t r = 0; r < CAN_MCP2518_COUNT; r++)
		spi_poll_one(spi_rail_to_bus(r));
}
