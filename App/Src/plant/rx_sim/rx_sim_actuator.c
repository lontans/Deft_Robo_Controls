#include "plant/rx_sim/rx_sim_actuator.h"
#include "plant/rx_sim/rx_sim.h"
#include "plant/can/can_router.h"
#include "plant/plugins/robstride.h"

void rx_sim_actuator_on_apply(const actuator_config_t *cfg,
                              const actuator_desire_t *desire)
{
	can_frame_t fb;

	if (!rx_sim_actuator_enabled() || cfg == NULL || desire == NULL)
		return;
	/* Match prior matrix hold: non-idle MIT desires only. */
	if (desire->kp <= 0.01f && desire->kd <= 0.01f)
		return;
	if (cfg->protocol != PROTO_ROBSTRIDE)
		return;

	if (robstride_pack_feedback(cfg,
	                            desire->position,
	                            desire->velocity,
	                            desire->torque,
	                            25.0f, &fb))
		(void)can_rx_push(cfg->bus, &fb);
}
