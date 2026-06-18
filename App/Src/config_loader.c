#include "config_loader.h"
#include "actuator.h"

void config_loader_init(void) {

	// Initialising fake actuator for testing
	actuator_table[0].bus = CAN_BUS_CH1;
	actuator_table[0].protocol = PROTO_ROBSTRIDE;
	actuator_table[0].motor_id = 0x00000141;   //placeholder
	actuator_table[0].enabled = true;

}
