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
static bool              g_mcp2518_hw_ready;
static uint8_t             g_mcp2518_rail_next;

static void spi_can_router_hw_step(void)
{
	if (g_mcp2518_hw_ready)
		return;

	if (g_mcp2518_rail_next == 0u) {
		spi_can_port_init();
		spi_can_port_irq_init();
	}

	if (g_mcp2518_rail_next < CAN_MCP2518_COUNT) {
		(void)mcp2518_reinit_rail((can_bus_id_t)(CAN_BUS_CH4 + g_mcp2518_rail_next));
		g_mcp2518_rail_next++;
		return;
	}

	g_mcp2518_hw_ready = true;
}

bool spi_can_router_hw_ready(void)
{
	return g_mcp2518_hw_ready;
}

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

can_status_t spi_can_router_rx_push(can_bus_id_t bus, const can_frame_t *frame)
{
	if (!spi_can_bus_valid(bus) || frame == NULL)
		return CAN_ERR_PARAM;

	spi_rx_push(bus, frame);
	return CAN_OK;
}

static can_status_t spi_backend_send(can_bus_id_t bus, const can_frame_t *frame)
{
	if (!spi_can_bus_valid(bus) || frame == NULL)
		return CAN_ERR_PARAM;

	if (!g_mcp2518_hw_ready)
		return CAN_ERR_BUSY;

	if (!mcp2518_send(bus, frame))
		return CAN_ERR_HAL;

	can_router_mark_traffic(bus);
	return CAN_OK;
}

static can_status_t spi_backend_send_try(can_bus_id_t bus, const can_frame_t *frame)
{
	if (!spi_can_bus_valid(bus) || frame == NULL)
		return CAN_ERR_PARAM;

	if (!g_mcp2518_hw_ready)
		return CAN_ERR_BUSY;

	if (!mcp2518_try_send(bus, frame))
		return CAN_ERR_BUSY;

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

	/* Eager init all MCP rails at boot. Lazy one-rail-per-poll never finished
	 * once end-of-lap stopped calling spi_can_router_poll() — plant MCP TX
	 * then hit !initialized and silently dropped every frame. */
	g_mcp2518_hw_ready = false;
	g_mcp2518_rail_next = 0u;
	spi_can_port_init();
	spi_can_port_irq_init();
	for (uint8_t r = 0; r < CAN_MCP2518_COUNT; r++)
		(void)mcp2518_reinit_rail((can_bus_id_t)(CAN_BUS_CH4 + r));
	g_mcp2518_rail_next = CAN_MCP2518_COUNT;
	g_mcp2518_hw_ready = true;
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

	can_status_t status = spi_backend_send_try(bus, &tx_queues[rail].buf[tx_queues[rail].tail]);
	if (status == CAN_ERR_BUSY)
		return CAN_ERR_BUSY;

	if (status != CAN_OK) {
		mcp2518_prepare_tx(bus);
		status = spi_backend_send_try(bus, &tx_queues[rail].buf[tx_queues[rail].tail]);
		if (status == CAN_ERR_BUSY)
			return CAN_ERR_BUSY;
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

	/*
	 * INT is level-low while the RX FIFO is non-empty; EXTI only catches
	 * falling edges into rx_irq_pending. Gate SPI on (level || SW flag) so
	 * idle rails pay 0 SPI. No critical section: after drain, clear the flag
	 * and re-check GPIO — sticky INT self-heals a torn bool or lost edge
	 * (contrast g_control_ticks_pending in control_loop.c, which needs a CS
	 * because a lost increment is not level-sticky).
	 */
	for (;;) {
		if (!spi_can_port_int_active(rail) &&
		    !mcp2518_rx_irq_is_pending(rail))
			return;

		can_frame_t temp;
		bool got = false;

		while (mcp2518_recv(bus, &temp)) {
			spi_rx_push(bus, &temp);
			can_router_mark_traffic(bus);
			got = true;
		}

		mcp2518_rx_irq_clear(rail);

		/* More frames keep INT low without a new edge — loop. */
		if (!spi_can_port_int_active(rail))
			return;
		/* INT stuck or empty after SPI: avoid spin; next poll/EXTI retries. */
		if (!got)
			return;
	}
}

#define SPI_POLL_TX_MAX 2u

static void spi_poll_one(can_bus_id_t bus)
{
	spi_poll_rx_one(bus);

	/* Plant already coalesced one flush/rail; drain at most one more SW
	 * queued MIT (2 slots/rail, 1-deep TXQ). Was 4 — busy empty-bus CH4–6
	 * paid repeated try_send/service SPI for no gain. */
	for (uint8_t n = 0; n < SPI_POLL_TX_MAX; n++) {
		if (spi_can_router_tx_flush(bus) != CAN_OK)
			break;
	}
}

void spi_can_router_poll_bus_rx(can_bus_id_t bus)
{
	if (!spi_can_bus_valid(bus))
		return;

	spi_can_router_hw_step();
	spi_poll_rx_one(bus);
}

void spi_can_router_poll_bus(can_bus_id_t bus)
{
	if (!spi_can_bus_valid(bus))
		return;

	spi_can_router_hw_step();
	spi_poll_one(bus);
}

void spi_can_router_poll(void)
{
	spi_can_router_hw_step();
	for (uint8_t r = 0; r < CAN_MCP2518_COUNT; r++)
		spi_poll_one(spi_rail_to_bus(r));
}
