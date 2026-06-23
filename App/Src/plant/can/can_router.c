#include "plant/can/can_router.h"
#include <string.h>
#include "fdcan.h"
#include "main.h"

#define CAN_ACTIVITY_PORT  GPIOC
#define CAN_ACTIVITY_PIN   GPIO_PIN_7

#define CAN_QUEUE_DEPTH 32

typedef struct {
	can_frame_t buf[CAN_QUEUE_DEPTH];
	uint16_t head;
	uint16_t tail;
} can_rx_ring_t;

typedef struct {
	can_frame_t buf[CAN_QUEUE_DEPTH];
	uint16_t head;
	uint16_t tail;
} can_tx_queue_t;

static can_rx_ring_t rx_rings[CAN_BUS_COUNT];
static can_tx_queue_t tx_queues[CAN_BUS_COUNT];

static uint32_t dlc_to_hal(uint8_t dlc)
{
	static const uint32_t table[] = {
		FDCAN_DLC_BYTES_0,
		FDCAN_DLC_BYTES_1,
		FDCAN_DLC_BYTES_2,
		FDCAN_DLC_BYTES_3,
		FDCAN_DLC_BYTES_4,
		FDCAN_DLC_BYTES_5,
		FDCAN_DLC_BYTES_6,
		FDCAN_DLC_BYTES_7,
		FDCAN_DLC_BYTES_8,
	};

	if (dlc > CAN_MAX_DATA_LEN) {
		return FDCAN_DLC_BYTES_0;
	}
	return table[dlc];
}

static uint8_t dlc_from_hal(uint32_t hal_dlc)
{
	uint32_t n = hal_dlc >> 16;
	return (n > CAN_MAX_DATA_LEN) ? CAN_MAX_DATA_LEN : (uint8_t)n;
}

void can_router_init(void)
{
	for (int i = 0; i < CAN_BUS_COUNT; i++) {
		rx_rings[i].head = 0;
		rx_rings[i].tail = 0;
		tx_queues[i].head = 0;
		tx_queues[i].tail = 0;
	}

	FDCAN_FilterTypeDef filter = {0};
	filter.IdType = FDCAN_EXTENDED_ID;
	filter.FilterIndex = 0;
	filter.FilterType = FDCAN_FILTER_MASK;
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	filter.FilterID1 = 0x00000000;
	filter.FilterID2 = 0x00000000;

	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
			FDCAN_REJECT, FDCAN_REJECT,
			FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}
}

can_status_t can_tx_enqueue(can_bus_id_t bus, const can_frame_t *frame)
{
	if (bus >= CAN_BUS_COUNT || frame == NULL) {
		return CAN_ERR_PARAM;
	}
	else if (((tx_queues[bus].head + 1) % CAN_QUEUE_DEPTH) == tx_queues[bus].tail) {
		return CAN_ERR_FULL;
	}
	else {
		tx_queues[bus].buf[tx_queues[bus].head] = *frame;
		tx_queues[bus].head = (tx_queues[bus].head + 1) % CAN_QUEUE_DEPTH;
		return CAN_OK;
	}
}

static can_status_t backend_send(can_bus_id_t bus, const can_frame_t *frame)
{
	if (bus != CAN_BUS_CH1 || frame == NULL) {
		return CAN_ERR_PARAM;
	}
	else if (frame->dlc > CAN_MAX_DATA_LEN) {
		return CAN_ERR_PARAM;
	}

	FDCAN_TxHeaderTypeDef tx_header = {0};
	if (frame->id_type == CAN_ID_EXT) {
		tx_header.Identifier = frame->id & CAN_EXT_MASK;
		tx_header.IdType = FDCAN_EXTENDED_ID;
	}
	else {
		tx_header.Identifier = frame->id & CAN_STD_ID_MASK;
		tx_header.IdType = FDCAN_STANDARD_ID;
	}

	tx_header.TxFrameType = FDCAN_DATA_FRAME;
	tx_header.DataLength = dlc_to_hal(frame->dlc);
	tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

	uint8_t tx_data[CAN_MAX_DATA_LEN];
	memcpy(tx_data, frame->data, frame->dlc);

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data) != HAL_OK) {
		return CAN_ERR_HAL;
	}

	HAL_GPIO_WritePin(CAN_ACTIVITY_PORT, CAN_ACTIVITY_PIN, GPIO_PIN_SET);
	return CAN_OK;
}

static can_status_t backend_recv(can_bus_id_t bus, can_frame_t *frame)
{
	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[CAN_MAX_DATA_LEN];

	if (bus != CAN_BUS_CH1 || frame == NULL) {
		return CAN_ERR_PARAM;
	}

	if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0) {
		return CAN_ERR_EMPTY;
	}

	if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
		return CAN_ERR_HAL;
	}

	frame->id = rx_header.Identifier;
	if (rx_header.IdType == FDCAN_EXTENDED_ID) {
		frame->id_type = CAN_ID_EXT;
	}
	else {
		frame->id_type = CAN_ID_STD;
	}

	frame->dlc = dlc_from_hal(rx_header.DataLength);

	memset(frame->data, 0, sizeof(frame->data));
	memcpy(frame->data, rx_data, frame->dlc);

	HAL_GPIO_WritePin(CAN_ACTIVITY_PORT, CAN_ACTIVITY_PIN, GPIO_PIN_RESET);
	return CAN_OK;
}

can_status_t can_tx_flush(can_bus_id_t bus)
{
	if (bus >= CAN_BUS_COUNT) {
		return CAN_ERR_PARAM;
	}
	else if (tx_queues[bus].head == tx_queues[bus].tail) {
		return CAN_ERR_EMPTY;
	}
	else {
		can_status_t status;
		status = backend_send(bus, &tx_queues[bus].buf[tx_queues[bus].tail]);
		if (status != CAN_OK) {
			return status;
		}
		tx_queues[bus].tail = (tx_queues[bus].tail + 1) % CAN_QUEUE_DEPTH;
		return CAN_OK;
	}
}

can_status_t can_rx_pop(can_bus_id_t bus, can_frame_t *frame)
{
	if (bus >= CAN_BUS_COUNT || frame == NULL) {
		return CAN_ERR_PARAM;
	}
	else if (rx_rings[bus].head == rx_rings[bus].tail) {
		return CAN_ERR_EMPTY;
	}
	else {
		*frame = rx_rings[bus].buf[rx_rings[bus].tail];
		rx_rings[bus].tail = (rx_rings[bus].tail + 1) % CAN_QUEUE_DEPTH;
		return CAN_OK;
	}
}

bool can_rx_available(can_bus_id_t bus)
{
	if (bus >= CAN_BUS_COUNT) {
		return false;
	}
	return rx_rings[bus].head != rx_rings[bus].tail;
}

void can_router_poll(void)
{
	for (int i = 0; i < CAN_BUS_COUNT; i++) {
		can_bus_id_t bus = (can_bus_id_t)i;
		can_frame_t temp;
		while (backend_recv(bus, &temp) == CAN_OK) {
			if ((rx_rings[bus].head + 1) % CAN_QUEUE_DEPTH == rx_rings[bus].tail) {
				break;
			}
			rx_rings[bus].buf[rx_rings[bus].head] = temp;
			rx_rings[bus].head = (rx_rings[bus].head + 1) % CAN_QUEUE_DEPTH;
		}

		while (can_tx_flush(bus) == CAN_OK) {
			;
		}
	}
}
