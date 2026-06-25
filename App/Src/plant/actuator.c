#include "plant/actuator.h"
#include "plant/plugin_schema/plugin_table.h"
#include "plant/can/can_router.h"
#include "plant/plugins/robstride.h"
#include "plant/plant_diag.h"
#include "host/host_link.h"
#include "main.h"
#include <string.h>

#define ACTUATOR_HOST_STALE_MS 500u

static actuator_desire_t actuator_desire_stage[ACTUATOR_COUNT];
static actuator_state_t  actuator_state_stage[ACTUATOR_COUNT];
static volatile bool     actuator_desire_pending;

actuator_config_t actuator_table[ACTUATOR_COUNT];
actuator_desire_t actuator_desire_live[ACTUATOR_COUNT];
actuator_state_t  actuator_state_live[ACTUATOR_COUNT];

void actuator_init(void)
{
	memset(actuator_table, 0, sizeof(actuator_table));
	memset(actuator_desire_live, 0, sizeof(actuator_desire_live));
	memset(actuator_state_live, 0, sizeof(actuator_state_live));
	memset(actuator_desire_stage, 0, sizeof(actuator_desire_stage));
	memset(actuator_state_stage, 0, sizeof(actuator_state_stage));
	actuator_desire_pending = false;
}

void actuator_command_mount(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return;

	__disable_irq();
	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++)
		actuator_desire_stage[i] = cmd->actuator_commands[i];
	actuator_desire_pending = true;
	__enable_irq();
}

void actuator_desire_clear(void)
{
	__disable_irq();
	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++)
		memset(&actuator_desire_live[i], 0, sizeof(actuator_desire_t));
	memset(actuator_desire_stage, 0, sizeof(actuator_desire_stage));
	actuator_desire_pending = false;
	__enable_irq();
}

void plant_recovery_all(void)
{
	can_frame_t frame;

	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		if (!actuator_table[i].enabled)
			continue;
		if (actuator_table[i].protocol != PROTO_ROBSTRIDE)
			continue;

		if (robstride_send_reset(&actuator_table[i], &frame) == PLUGIN_OK)
			(void)can_tx_enqueue(actuator_table[i].bus, &frame);
	}

	can_router_poll();
	actuator_desire_clear();
}

void actuator_apply_desire(void)
{
	__disable_irq();
	if (actuator_desire_pending) {
		for (uint8_t i = 0; i < ACTUATOR_COUNT; i++)
			actuator_desire_live[i] = actuator_desire_stage[i];
		actuator_desire_pending = false;
	}
	__enable_irq();

	if (plant_diag_skip_actuator_can())
		return;

	if (!host_link_command_is_fresh(ACTUATOR_HOST_STALE_MS))
		return;

	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		if (!actuator_table[i].enabled)
			continue;

		if (actuator_table[i].protocol == PROTO_ROBSTRIDE) {
			robstride_apply_cycle(&actuator_table[i],
			                      &actuator_desire_live[i],
			                      &actuator_state_live[i]);
			continue;
		}

		can_frame_t tx;
		if (plugin_pack_tx(&actuator_table[i], &actuator_desire_live[i], &tx) == PLUGIN_OK)
			(void)can_tx_enqueue(actuator_table[i].bus, &tx);
	}

	can_router_poll();

	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		if (actuator_table[i].protocol == PROTO_ROBSTRIDE)
			continue;

		can_frame_t rx;
		while (can_rx_pop(actuator_table[i].bus, &rx) == CAN_OK)
			plugin_parse_rx(&actuator_table[i], &rx, &actuator_state_live[i]);
	}
}

void actuator_capture_state(void)
{
	__disable_irq();
	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++)
		actuator_state_stage[i] = actuator_state_live[i];
	__enable_irq();
}

void actuator_feedback_snapshot(host_actuator_feedback_t *dst, uint8_t count)
{
	if (dst == NULL || count == 0)
		return;

	uint8_t n = (count < ACTUATOR_COUNT) ? count : ACTUATOR_COUNT;

	__disable_irq();
	for (uint8_t i = 0; i < n; i++)
		dst[i] = actuator_state_stage[i];
	__enable_irq();

	for (uint8_t i = n; i < count; i++)
		memset(&dst[i], 0, sizeof(dst[i]));
}
