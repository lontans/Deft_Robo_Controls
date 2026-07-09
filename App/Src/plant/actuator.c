#include "plant/actuator.h"
#include "plant/plugin_schema/plugin_table.h"
#include "plant/can/can_router.h"
#include "plant/plugins/robstride.h"
#include "plant/plugins/damiao.h"
#include "plant/plant_diag.h"
#include "plant/plant_command.h"
#include "host/host_link.h"
#include "plant/plant_crit.h"
#include <math.h>
#include <string.h>

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

	plant_crit_enter();
	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++)
		actuator_desire_stage[i] = cmd->actuator_commands[i];
	actuator_desire_pending = true;
	plant_crit_exit();
}

void actuator_desire_clear(void)
{
	plant_crit_enter();
	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++)
		memset(&actuator_desire_live[i], 0, sizeof(actuator_desire_t));
	memset(actuator_desire_stage, 0, sizeof(actuator_desire_stage));
	actuator_desire_pending = false;
	plant_crit_exit();
}

bool actuator_any_non_idle_live(void)
{
	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		const actuator_desire_t *d = &actuator_desire_live[i];

		if (!actuator_table[i].enabled)
			continue;
		if (d->kp > 0.01f || d->kd > 0.01f)
			return true;
		if (fabsf(d->velocity) > 0.01f || fabsf(d->torque) > 0.01f)
			return true;
	}

	return false;
}

void plant_recovery_all(void)
{
	can_frame_t frame;

	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		if (!actuator_table[i].enabled)
			continue;

		if (actuator_table[i].protocol == PROTO_ROBSTRIDE) {
			if (robstride_send_reset(&actuator_table[i], &frame) == PLUGIN_OK)
				(void)can_tx_enqueue(actuator_table[i].bus, &frame);
			continue;
		}

		if (actuator_table[i].protocol == PROTO_DAMIAO) {
			damiao_reset_enable_latch(i);
			if (damiao_send_disable(&actuator_table[i], &frame) == PLUGIN_OK)
				(void)can_tx_enqueue(actuator_table[i].bus, &frame);
		}
	}

	can_router_poll();
	plant_diag_release_actuator_can();
	actuator_desire_clear();
}

void actuator_apply_desire(void)
{
	plant_crit_enter();
	if (actuator_desire_pending) {
		for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
			actuator_desire_live[i] = actuator_desire_stage[i];
			if (actuator_table[i].enabled &&
			    actuator_table[i].protocol == PROTO_ROBSTRIDE)
				robstride_host_desire_updated(i, &actuator_desire_live[i]);
		}
		actuator_desire_pending = false;
	}
	plant_crit_exit();

	if (!plant_runtime_actuator_can_apply())
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

		if (actuator_table[i].protocol == PROTO_DAMIAO) {
			damiao_apply_cycle(&actuator_table[i],
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
		if (actuator_table[i].protocol == PROTO_DAMIAO)
			continue;

		can_frame_t rx;
		while (can_rx_pop(actuator_table[i].bus, &rx) == CAN_OK)
			plugin_parse_rx(&actuator_table[i], &rx, &actuator_state_live[i]);
	}
}

void actuator_capture_state(void)
{
	plant_crit_enter();
	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++)
		actuator_state_stage[i] = actuator_state_live[i];
	plant_crit_exit();
}

void actuator_feedback_snapshot(host_actuator_feedback_t *dst, uint8_t count)
{
	if (dst == NULL || count == 0)
		return;

	uint8_t n = (count < ACTUATOR_COUNT) ? count : ACTUATOR_COUNT;

	plant_crit_enter();
	for (uint8_t i = 0; i < n; i++)
		dst[i] = actuator_state_stage[i];
	plant_crit_exit();

	for (uint8_t i = n; i < count; i++)
		memset(&dst[i], 0, sizeof(dst[i]));
}
