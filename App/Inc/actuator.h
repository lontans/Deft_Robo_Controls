// actuator.h — actuator_table[], actuator_desire[], actuator_state[]
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "can_frame.h"
#include "host_exchange.h"

typedef enum {
	PROTO_NONE = 0,
	PROTO_ROBSTRIDE,
	PROTO_CUBEMARS,
	PROTO_COUNT,
} protocol_t;

typedef struct {
	can_bus_id_t bus;
	protocol_t protocol;
	uint32_t motor_id;
	bool enabled;
} actuator_config_t;

typedef host_actuator_command_t  actuator_desire_t;
typedef host_actuator_feedback_t actuator_state_t;

#define ACTUATOR_COUNT 1u

extern actuator_config_t    actuator_table[ACTUATOR_COUNT];
extern actuator_desire_t    actuator_desire[ACTUATOR_COUNT];
extern actuator_state_t     actuator_state[ACTUATOR_COUNT];

void actuator_init(void);
void actuator_stage_desires(const host_command_image_t *cmd); // buffer desires

void actuator_apply(void);    // read desires, poll (flush tx, receive rx)
void actuator_consume(void);  // publish states to staging

void actuator_state_snapshot(host_actuator_feedback_t *dst, uint8_t count);
