#include "plant/plant_config.h"
#include "plant/actuator.h"

void plant_config_init(void)
{
	actuator_table[0] = (actuator_config_t){
		.bus = CAN_BUS_CH1,
		.protocol = PROTO_ROBSTRIDE,
		.motor_id = 0x70,
		.enabled = true,
	};

	actuator_table[1] = (actuator_config_t){
		.bus = CAN_BUS_CH1,
		.protocol = PROTO_ROBSTRIDE,
		.motor_id = 0x74,
		.enabled = true,
	};

	actuator_table[2] = (actuator_config_t){
		.bus = CAN_BUS_CH2,
		.protocol = PROTO_ROBSTRIDE,
		.motor_id = 0x73,
		.enabled = true,
	};

	actuator_table[3] = (actuator_config_t){
		.bus = CAN_BUS_CH3,
		.protocol = PROTO_ROBSTRIDE,
		.motor_id = 0x75,
		.enabled = true,
	};

	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		actuator_desire_live[i].position = 0.0f;
		actuator_desire_live[i].velocity = 0.0f;
		actuator_desire_live[i].kp       = 0.0f;
		actuator_desire_live[i].kd       = 0.0f;
		actuator_desire_live[i].torque   = 0.0f;
	}
}
