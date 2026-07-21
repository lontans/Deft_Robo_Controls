#pragma once
/*
 * Duplicated-and-edited from the live App/Inc/plant/actuator.h — only
 * change is adding PROTO_ZEROERR. Diff against the live file before
 * merging; nothing else in this header should differ.
 *
 * PROTO_ZEROERR is a RESERVED SLOT, not a working plugin — see
 * App/Inc/plant/plugins/zeroerr.h in this workstream for exactly what's
 * unknown (no vendor CAN protocol doc in this repo; ZeroErr's actuators may
 * turn out to be CANopen-based, which would need a materially different
 * integration shape than the single-frame RobStride/Damiao/CubeMars
 * pattern below — do not assume it's a drop-in until that's confirmed).
 * Adding the enum value now means the generic plugin_pack_tx/parse_rx
 * dispatch (plugin_table.c) already handles PROTO_ZEROERR safely (cleanly
 * returns PLUGIN_ERR_UNSUPPORTED) the moment it's assigned to a slot via
 * CFG SET, instead of that being undefined/a later header change.
 */
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

#define ACTUATOR_COUNT 14u

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
