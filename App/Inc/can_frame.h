// can_frame.h — protocol-agnostic CAN 2.0 Classic frame types and constants
#pragma once
#include <stdint.h>
#define CAN_MAX_DATA_LEN 8U
#define CAN_STD_ID_MASK 0x7FFU    /*11 bit standard CAN 2.0*/
#define CAN_EXT_MASK 0x1FFFFFFFU  /*29 bit extended CAN 2.0*/


typedef enum {
	CAN_ID_STD = 0,
	CAN_ID_EXT,
} can_id_type_t;

#define CAN_DEFAULT_ID_TYPE CAN_ID_EXT

typedef enum {
	CAN_BUS_CH1 = 0,
	CAN_BUS_CH2,
	CAN_BUS_CH3,
	CAN_BUS_CH4,
	CAN_BUS_CH5,
	CAN_BUS_CH6,
	CAN_BUS_COUNT,
} can_bus_id_t;

typedef struct {
	uint32_t id;
	can_id_type_t id_type;
	uint8_t  dlc;
	uint8_t  data[CAN_MAX_DATA_LEN];
} can_frame_t;
