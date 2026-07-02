#pragma once
#include <stdbool.h>
#include "plant/can/can_frame.h"

#define CAN_FDCAN_COUNT   3u
#define CAN_MCP2518_COUNT 3u
#define CAN_BACKEND_COUNT (CAN_FDCAN_COUNT + CAN_MCP2518_COUNT)

typedef enum {
	CAN_OK = 0,
	CAN_ERR_PARAM,
	CAN_ERR_FULL,
	CAN_ERR_EMPTY,
	CAN_ERR_HAL,
} can_status_t;

void can_router_init(void);
void can_router_poll(void);
void can_router_poll_bus(can_bus_id_t bus);
void can_router_poll_bus_rx(can_bus_id_t bus);
void can_router_discard_pending_tx(void);
void can_router_mark_traffic(can_bus_id_t bus);

can_status_t can_tx_enqueue(can_bus_id_t bus, const can_frame_t *frame);
can_status_t can_tx_flush(can_bus_id_t bus);

can_status_t can_rx_pop(can_bus_id_t bus, can_frame_t *frame);
void can_rx_drain(can_bus_id_t bus);
bool can_rx_available(can_bus_id_t bus);
