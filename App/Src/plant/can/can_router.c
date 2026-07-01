#include "plant/can/can_router.h"
#include "plant/can/mcp2518fd.h"
#include "plant/can/spi_can_port.h"

#include <string.h>

#include "fdcan.h"
#include "main.h"

/* CANx_ACT activity LEDs (active-high idle ON; blink on TX/RX). */
#define CAN1_ACT_PORT  GPIOC
#define CAN1_ACT_PIN   GPIO_PIN_7
#define CAN2_ACT_PORT  GPIOC
#define CAN2_ACT_PIN   GPIO_PIN_6
#define CAN3_ACT_PORT  GPIOB
#define CAN3_ACT_PIN   GPIO_PIN_15
#define CAN4_ACT_PORT  GPIOB
#define CAN4_ACT_PIN   GPIO_PIN_14
#define CAN5_ACT_PORT  GPIOB
#define CAN5_ACT_PIN   GPIO_PIN_2
#define CAN6_ACT_PORT  GPIOC
#define CAN6_ACT_PIN   GPIO_PIN_5

#define CAN_LED_ACTIVE_MS   250u
#define CAN_LED_BLINK_MS    125u
#define CAN_QUEUE_DEPTH     128u

typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
} can_act_led_t;

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

static FDCAN_HandleTypeDef *bus_handle[] = {
	&hfdcan1, /* CAN_BUS_CH1: PB8/PB9  (FDCAN1) */
	&hfdcan3, /* CAN_BUS_CH2: PA8/PA15 (sheet CH2) — not Cube "FDCAN2" */
	&hfdcan2, /* CAN_BUS_CH3: PB12/PB13 (sheet CH3) — not Cube "FDCAN3" */
};

static const can_act_led_t can_act_led[CAN_BACKEND_COUNT] = {
	{ CAN1_ACT_PORT, CAN1_ACT_PIN },
	{ CAN2_ACT_PORT, CAN2_ACT_PIN },
	{ CAN3_ACT_PORT, CAN3_ACT_PIN },
	{ CAN4_ACT_PORT, CAN4_ACT_PIN },
	{ CAN5_ACT_PORT, CAN5_ACT_PIN },
	{ CAN6_ACT_PORT, CAN6_ACT_PIN },
};

static can_rx_ring_t rx_rings[CAN_BUS_COUNT];
static can_tx_queue_t tx_queues[CAN_BUS_COUNT];
static uint32_t g_last_traffic_ms[CAN_BACKEND_COUNT];
static uint32_t g_blink_last_ms[CAN_BACKEND_COUNT];
static bool g_blink_on[CAN_BACKEND_COUNT];

static bool bus_is_mcp2518(can_bus_id_t bus)
{
	return (bus >= CAN_BUS_CH4 && bus < CAN_BUS_COUNT);
}

static void fdcan_bus_start(FDCAN_HandleTypeDef *h)
{
	FDCAN_FilterTypeDef filter = {0};

	filter.IdType = FDCAN_EXTENDED_ID;
	filter.FilterIndex = 0;
	filter.FilterType = FDCAN_FILTER_MASK;
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	filter.FilterID1 = 0;
	filter.FilterID2 = 0;

	if (HAL_FDCAN_ConfigFilter(h, &filter) != HAL_OK)
		Error_Handler();

	if (HAL_FDCAN_ConfigGlobalFilter(h,
			FDCAN_REJECT, FDCAN_REJECT,
			FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
		Error_Handler();

	if (HAL_FDCAN_Start(h) != HAL_OK)
		Error_Handler();
}

static void can_led_mark_traffic(can_bus_id_t bus)
{
	if (bus < CAN_BACKEND_COUNT)
		g_last_traffic_ms[bus] = HAL_GetTick();
}

void can_router_mark_traffic(can_bus_id_t bus)
{
	can_led_mark_traffic(bus);
}

static void can_led_poll(void)
{
	uint32_t now = HAL_GetTick();

	for (uint8_t i = 0; i < CAN_BACKEND_COUNT; i++) {
		if (g_last_traffic_ms[i] != 0u &&
		    (now - g_last_traffic_ms[i]) <= CAN_LED_ACTIVE_MS) {
			if ((now - g_blink_last_ms[i]) >= CAN_LED_BLINK_MS) {
				g_blink_last_ms[i] = now;
				g_blink_on[i] = !g_blink_on[i];
				HAL_GPIO_WritePin(can_act_led[i].port, can_act_led[i].pin,
				                  g_blink_on[i] ? GPIO_PIN_SET : GPIO_PIN_RESET);
			}
		} else {
			HAL_GPIO_WritePin(can_act_led[i].port, can_act_led[i].pin, GPIO_PIN_SET);
			g_blink_on[i] = true;
		}
	}
}

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

	if (dlc > CAN_MAX_DATA_LEN)
		return FDCAN_DLC_BYTES_0;
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

	for (uint8_t i = 0; i < CAN_BACKEND_COUNT; i++) {
		g_last_traffic_ms[i] = 0u;
		g_blink_last_ms[i] = 0u;
		g_blink_on[i] = true;
		HAL_GPIO_WritePin(can_act_led[i].port, can_act_led[i].pin, GPIO_PIN_SET);
	}

	fdcan_bus_start(&hfdcan1);
	fdcan_bus_start(&hfdcan2);
	fdcan_bus_start(&hfdcan3);

	(void)mcp2518_init_all();
}

can_status_t can_tx_enqueue(can_bus_id_t bus, const can_frame_t *frame)
{
	if (bus >= CAN_BUS_COUNT || frame == NULL)
		return CAN_ERR_PARAM;

	if (((tx_queues[bus].head + 1) % CAN_QUEUE_DEPTH) == tx_queues[bus].tail)
		return CAN_ERR_FULL;

	tx_queues[bus].buf[tx_queues[bus].head] = *frame;
	tx_queues[bus].head = (tx_queues[bus].head + 1) % CAN_QUEUE_DEPTH;
	return CAN_OK;
}

static can_status_t backend_send(can_bus_id_t bus, const can_frame_t *frame)
{
	if (bus >= CAN_BACKEND_COUNT || frame == NULL)
		return CAN_ERR_PARAM;

	if (bus_is_mcp2518(bus))
	{
		if (!mcp2518_send(bus, frame))
			return CAN_ERR_HAL;
		can_led_mark_traffic(bus);
		return CAN_OK;
	}

	if (frame->dlc > CAN_MAX_DATA_LEN)
		return CAN_ERR_PARAM;

	FDCAN_TxHeaderTypeDef tx_header = {0};

	if (frame->id_type == CAN_ID_EXT) {
		tx_header.Identifier = frame->id & CAN_EXT_MASK;
		tx_header.IdType = FDCAN_EXTENDED_ID;
	} else {
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

	if (HAL_FDCAN_AddMessageToTxFifoQ(bus_handle[bus], &tx_header, tx_data) != HAL_OK)
		return CAN_ERR_HAL;

	can_led_mark_traffic(bus);
	return CAN_OK;
}

static can_status_t backend_recv(can_bus_id_t bus, can_frame_t *frame)
{
	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[CAN_MAX_DATA_LEN];

	if (bus >= CAN_BACKEND_COUNT || frame == NULL)
		return CAN_ERR_PARAM;

	if (bus_is_mcp2518(bus)) {
		if (!mcp2518_recv(bus, frame))
			return CAN_ERR_EMPTY;
		can_led_mark_traffic(bus);
		return CAN_OK;
	}

	if (HAL_FDCAN_GetRxFifoFillLevel(bus_handle[bus], FDCAN_RX_FIFO0) == 0)
		return CAN_ERR_EMPTY;

	if (HAL_FDCAN_GetRxMessage(bus_handle[bus], FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
		return CAN_ERR_HAL;

	frame->id = rx_header.Identifier;
	frame->id_type = (rx_header.IdType == FDCAN_EXTENDED_ID) ? CAN_ID_EXT : CAN_ID_STD;

	/* RS02: identifier is the signal; pass full 8-byte buffer like FDCAN path. */
	(void)dlc_from_hal(rx_header.DataLength);
	frame->dlc = CAN_MAX_DATA_LEN;
	memset(frame->data, 0, sizeof(frame->data));
	memcpy(frame->data, rx_data, CAN_MAX_DATA_LEN);

	can_led_mark_traffic(bus);
	return CAN_OK;
}

can_status_t can_tx_flush(can_bus_id_t bus)
{
	if (bus >= CAN_BUS_COUNT)
		return CAN_ERR_PARAM;

	if (tx_queues[bus].head == tx_queues[bus].tail)
		return CAN_ERR_EMPTY;

	can_status_t status = backend_send(bus, &tx_queues[bus].buf[tx_queues[bus].tail]);
	if (status != CAN_OK)
		return status;

	tx_queues[bus].tail = (tx_queues[bus].tail + 1) % CAN_QUEUE_DEPTH;
	return CAN_OK;
}

void can_rx_drain(can_bus_id_t bus)
{
	can_frame_t junk;

	if (bus >= CAN_BUS_COUNT)
		return;

	while (can_rx_pop(bus, &junk) == CAN_OK)
		;
}

can_status_t can_rx_pop(can_bus_id_t bus, can_frame_t *frame)
{
	if (bus >= CAN_BUS_COUNT || frame == NULL)
		return CAN_ERR_PARAM;

	if (rx_rings[bus].head == rx_rings[bus].tail)
		return CAN_ERR_EMPTY;

	*frame = rx_rings[bus].buf[rx_rings[bus].tail];
	rx_rings[bus].tail = (rx_rings[bus].tail + 1) % CAN_QUEUE_DEPTH;
	return CAN_OK;
}

bool can_rx_available(can_bus_id_t bus)
{
	if (bus >= CAN_BUS_COUNT)
		return false;
	return rx_rings[bus].head != rx_rings[bus].tail;
}

void can_router_poll(void)
{
	for (int i = 0; i < (int)CAN_BACKEND_COUNT; i++) {
		can_bus_id_t bus = (can_bus_id_t)i;

		if (bus_is_mcp2518(bus))
			mcp2518_drain_rx(bus);

		can_frame_t temp;
		while (backend_recv(bus, &temp) == CAN_OK) {
			/* Drop oldest on overflow. */
			if ((rx_rings[bus].head + 1) % CAN_QUEUE_DEPTH == rx_rings[bus].tail)
				rx_rings[bus].tail = (rx_rings[bus].tail + 1) % CAN_QUEUE_DEPTH;

			rx_rings[bus].buf[rx_rings[bus].head] = temp;
			rx_rings[bus].head = (rx_rings[bus].head + 1) % CAN_QUEUE_DEPTH;
		}

		while (can_tx_flush(bus) == CAN_OK)
			;
	}

	can_led_poll();
}
