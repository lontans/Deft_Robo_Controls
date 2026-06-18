// can_router.h — TX queue + RX ring per bus; HAL/MCP2518 backend glue
#pragma once
#include <stdbool.h>
#include "can_frame.h"

typedef enum {
	CAN_OK = 0,
	CAN_ERR_PARAM,
	CAN_ERR_FULL,  // Full TX queue
	CAN_ERR_EMPTY, // Empty RX ring
	CAN_ERR_HAL,
} can_status_t;

void can_router_init(void);
void can_router_poll(void);

can_status_t can_tx_enqueue(can_bus_id_t bus, const can_frame_t *frame); // Append a frame to queue
can_status_t can_tx_flush(can_bus_id_t bus); // Drain the full queue

can_status_t can_rx_pop(can_bus_id_t bus, can_frame_t *frame); // Pop the frame from the bus
bool can_rx_available(can_bus_id_t bus); // Returns whether rx is active on the bus
