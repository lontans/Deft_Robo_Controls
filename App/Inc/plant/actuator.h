#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "plant/can/can_frame.h"
#include "host/host_exchange_schema.h"

typedef enum {
	PROTO_NONE = 0,
	PROTO_ROBSTRIDE,
	PROTO_CUBEMARS,
	PROTO_DAMIAO,
	PROTO_ZEROERR,
	PROTO_COUNT,
} protocol_t;

typedef struct {
	can_bus_id_t bus;
	protocol_t protocol;
	uint32_t motor_id;
	uint32_t master_id; /* feedback CAN ID; protocol-specific (Damiao: 0 or DM_MASTER_ID_AUTO) */
	bool enabled;
} actuator_config_t;

typedef host_actuator_command_t  actuator_desire_t;
typedef host_actuator_feedback_t actuator_state_t;

/* Plant table size matches host exchange wire slots (672 B layout v2). */
#define ACTUATOR_COUNT HOST_EXCHANGE_ACTUATOR_SLOTS

extern actuator_config_t actuator_table[ACTUATOR_COUNT];
extern actuator_desire_t actuator_desire_live[ACTUATOR_COUNT];
extern actuator_state_t  actuator_state_live[ACTUATOR_COUNT];

void actuator_init(void);
void actuator_command_mount(const host_command_image_t *cmd);

void actuator_apply_desire(void);
void actuator_capture_state(void);

void actuator_feedback_snapshot(host_actuator_feedback_t *dst, uint8_t count);

void actuator_desire_clear(void);
void plant_recovery_all(void);

bool actuator_any_non_idle_live(void);

/* Bit N set ⇒ schematic CH(N+1) was polled in the last apply tick. */
uint32_t actuator_last_apply_poll_buses(void);
