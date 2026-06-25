#pragma once
#include <stdint.h>

#define DXL_PROTO_VERSION 2.0f // Using DynaMixel 2.0 Protocol

#define DXL_ADDR_OPERATING_MODE    11u
#define DXL_ADDR_TORQUE_ENABLE     64u
#define DXL_ADDR_LED               65u
#define DXL_ADDR_PROFILE_ACCEL     108u
#define DXL_ADDR_PROFILE_VELOCITY  112u
#define DXL_ADDR_GOAL_POSITION     116u
#define DXL_ADDR_MOVING            122u
#define DXL_ADDR_HW_ERROR_STATUS   70u
#define DXL_ADDR_PRESENT_VELOCITY  128u
#define DXL_ADDR_PRESENT_POSITION  132u

#define DXL_MODE_POSITION          3u
#define DXL_TORQUE_ON              1u
#define DXL_TORQUE_OFF             0u

#define DXL_POS_TICKS_PER_REV      4096
#define DXL_POS_MAX_TICK           4095

#define DXL_BAUD_RATE              1000000u
#define DXL_TX_TIMEOUT_MS          5u
#define DXL_RX_TIMEOUT_MS          5u

#define SERVO_COUNT                HOST_EXCHANGE_SERVO_SLOTS

typedef struct {
	uint8_t  id;
	bool     enabled;
	uint16_t pos_min;
	uint16_t pos_max;
	uint16_t default_profile_vel;
} servo_config_t;

