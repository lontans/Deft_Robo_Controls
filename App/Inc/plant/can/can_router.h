#pragma once
#include <stdbool.h>
#include "plant/can/can_frame.h"

typedef enum {
	CAN_OK = 0,
	CAN_ERR_PARAM,
	CAN_ERR_FULL,
	CAN_ERR_EMPTY,
	CAN_ERR_HAL,
} can_status_t;

void can_router_init(void);
void can_router_poll(void);

can_status_t can_tx_enqueue(can_bus_id_t bus, const can_frame_t *frame);
can_status_t can_tx_flush(can_bus_id_t bus);

can_status_t can_rx_pop(can_bus_id_t bus, can_frame_t *frame);
void can_rx_drain(can_bus_id_t bus);
bool can_rx_available(can_bus_id_t bus);
