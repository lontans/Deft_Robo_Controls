#pragma once

#include <stdbool.h>

#include "plant/can/can_frame.h"
#include "plant/can/can_router.h"

/* MCP2518 SPI-CAN rails (schematic CH4–CH6). Separate from FDCAN can_router loop. */

void spi_can_router_init(void);
void spi_can_router_poll(void);
void spi_can_router_poll_bus(can_bus_id_t bus);
void spi_can_router_poll_bus_rx(can_bus_id_t bus);
void spi_can_router_discard_pending_tx(void);

can_status_t spi_can_router_tx_enqueue(can_bus_id_t bus, const can_frame_t *frame);
can_status_t spi_can_router_tx_flush(can_bus_id_t bus);

/* Bypass software TX queue — for probes/bench where wedged queue stalls listen loops. */
bool spi_can_router_send_now(can_bus_id_t bus, const can_frame_t *frame);

can_status_t spi_can_router_rx_pop(can_bus_id_t bus, can_frame_t *frame);
void spi_can_router_rx_drain(can_bus_id_t bus);
bool spi_can_router_rx_available(can_bus_id_t bus);

static inline bool spi_can_bus_valid(can_bus_id_t bus)
{
	return (bus >= CAN_BUS_CH4 && bus < CAN_BUS_COUNT);
}
