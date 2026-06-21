#include "actuator.h"
#include "plugin_table.h"
#include "can_router.h"
#include <string.h>

static actuator_desire_t desire_staging[ACTUATOR_COUNT];
static actuator_state_t  state_staging[ACTUATOR_COUNT];
static volatile bool desire_pending;

actuator_config_t actuator_table[ACTUATOR_COUNT];
actuator_desire_t actuator_desire[ACTUATOR_COUNT];
actuator_state_t  actuator_state[ACTUATOR_COUNT];

void actuator_init(void)
{
	memset(actuator_table, 0, sizeof(actuator_table));
	memset(actuator_desire, 0, sizeof(actuator_desire));// Copied by actuator apply from desire staging, executed
	memset(actuator_state, 0, sizeof(actuator_state));  // Write by actuator consume, packaged
	memset(desire_staging, 0, sizeof(desire_staging));  // Write by host, read by actuator apply, not executed directly
	memset(state_staging, 0, sizeof(state_staging));    // Read by host, write by control tick
	desire_pending = false;
}

void actuator_stage_desires(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return;

	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++)
		desire_staging[i] = cmd->actuator_commands[i];

	desire_pending = true;
}

void actuator_apply(void)
{
	if (desire_pending) {
		for (uint8_t i = 0; i < ACTUATOR_COUNT; i++)
			actuator_desire[i] = desire_staging[i];
		desire_pending = false;
	}

	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		if (!actuator_table[i].enabled)
			continue;

		can_frame_t tx;
		if (plugin_pack_tx(&actuator_table[i], &actuator_desire[i], &tx) == PLUGIN_OK)
			can_tx_enqueue(actuator_table[i].bus, &tx);
	}

	can_router_poll();

	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		can_frame_t rx;
		while (can_rx_pop(actuator_table[i].bus, &rx) == CAN_OK)
			plugin_parse_rx(&actuator_table[i], &rx, &actuator_state[i]);
	}
}

void actuator_consume(void)
{
	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++)
		state_staging[i] = actuator_state[i];
}

void actuator_state_snapshot(host_actuator_feedback_t *dst, uint8_t count)
{
	if (dst == NULL || count == 0)
		return;

	uint8_t n = (count < ACTUATOR_COUNT) ? count : ACTUATOR_COUNT;

	for (uint8_t i = 0; i < n; i++)
		dst[i] = state_staging[i];

	for (uint8_t i = n; i < count; i++)
		memset(&dst[i], 0, sizeof(dst[i]));
}
