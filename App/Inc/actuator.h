// actuator.h — actuator_table[], desire[], state[] runtime tables
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "can_frame.h"

// Protocol list
typedef enum {
	PROTO_NONE = 0,
	PROTO_ROBSTRIDE,
	PROTO_CUBEMARS,
	PROTO_COUNT,
} protocol_t;

// TODO: Add limits/desires later (pos_min, pos_max, etc)
typedef struct {
	can_bus_id_t bus;
	protocol_t protocol;
	uint32_t motor_id;
	bool enabled;
} actuator_config_t ;

typedef struct {
	float position;
	float velocity;
	float kp;
	float kd;
	float torque;
} desire_t ;

typedef struct {
	float position;
	float velocity;
	float torque;
	uint32_t fault;

} state_t;

#define ACTUATOR_COUNT 1 // Currently 1, TODO add more actuators

// Declare external arrays which are defined in actuator.c. Three arrays in parallel for actuator config/desire/state per actuator
extern actuator_config_t actuator_table[ACTUATOR_COUNT];
extern desire_t          desire[ACTUATOR_COUNT];
extern state_t           state[ACTUATOR_COUNT];

void actuator_init(void);
