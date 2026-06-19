#include "config_loader.h"
#include "actuator.h"

void config_loader_init(void) {

	// Initialising fake actuator for testing
	actuator_table[0].bus = CAN_BUS_CH1;
	actuator_table[0].protocol = PROTO_ROBSTRIDE;
	actuator_table[0].motor_id = 0x7F;   //placeholder
	actuator_table[0].enabled = true;

	desire[0].position = 0.0f;
	desire[0].velocity = 0.0f;
	desire[0].kp       = 0.0f;
	desire[0].kd       = 1.0f;
	desire[0].torque   = 0.0f;

}
