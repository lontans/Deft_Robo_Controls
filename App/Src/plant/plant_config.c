#include "plant/plant_config.h"
#include "plant/actuator.h"

void plant_config_init(void)
{
	actuator_table[0].bus = CAN_BUS_CH1;
	actuator_table[0].protocol = PROTO_ROBSTRIDE;
	actuator_table[0].motor_id = 0x7F;
	actuator_table[0].enabled = true;

	actuator_desire_live[0].position = 0.0f;
	actuator_desire_live[0].velocity = 0.0f;
	actuator_desire_live[0].kp       = 0.0f;
	actuator_desire_live[0].kd       = 1.0f;
	actuator_desire_live[0].torque   = 0.0f;
}
