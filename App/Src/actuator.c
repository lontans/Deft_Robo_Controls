#include "actuator.h"
#include <string.h>

// Create an array for actuator config per actuator. Likewise, create more arrays for desires and states per actuator
actuator_config_t actuator_table[ACTUATOR_COUNT];
desire_t          desire[ACTUATOR_COUNT];
state_t           state[ACTUATOR_COUNT];

void actuator_init(void) {
	memset(actuator_table, 0, sizeof(actuator_table)); // Set up empty desire, state arrays in RAM.
	memset(desire, 0, sizeof(desire));
	memset(state, 0, sizeof(state));
}
