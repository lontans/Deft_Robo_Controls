#include "plant/plant_config.h"
#include "plant/actuator.h"
#include "plant/plugins/dynamixel.h"

// Servo table initialization
servo_config_t servo_table[SERVO_COUNT];

void plant_config_init(void)
{
	// Actuator initialization
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

	// Servo initialization
	// Neck travel measured on hardware (~1265 bottom, ~2623 top); not centered at 2048.
	servo_table[0] = (servo_config_t){
		.id = 1,
		.enabled = true,
		.pos_min = 1024,
		.pos_max = 3072,
		.position_p_gain = DXL_DEFAULT_POSITION_P_GAIN,
		.position_d_gain = DXL_DEFAULT_POSITION_D_GAIN,
		.default_profile_vel = 180,
		.default_profile_accel = DXL_DEFAULT_PROFILE_ACCEL,
	};

	servo_table[1] = (servo_config_t){
		.id = 2,
		.enabled = true,
		.pos_min = 512,
		.pos_max = 3072,
		.position_p_gain = DXL_DEFAULT_POSITION_P_GAIN,
		.position_d_gain = DXL_DEFAULT_POSITION_D_GAIN,
		.default_profile_vel = 180,
		.default_profile_accel = DXL_DEFAULT_PROFILE_ACCEL,
	};

}
