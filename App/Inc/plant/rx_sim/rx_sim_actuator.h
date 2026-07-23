#pragma once

#include "plant/actuator.h"
#include "plant/plugins/robstride.h"

/* After a held RobStride MIT: inject synthetic FB into the bus RX ring. */
void rx_sim_actuator_on_apply(const actuator_config_t *cfg,
                              const actuator_desire_t *desire);
