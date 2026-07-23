#include "plant/plant_config.h"
#include "plant/plant_config_nvm.h"
#include "plant/actuator.h"
#include "plant/plugins/dynamixel.h"
#include "plant/plugins/sk9822.h"
#include "plant/plugins/damiao.h"

// Servo table initialization
servo_config_t servo_table[SERVO_COUNT];

/* LED strip config (same pattern as servo_table: defined here, extern in sk9822.h). */
sk9822_config_t led_table[LED_STRIP_COUNT];

void plant_config_init(void)
{
	plant_config_load_factory_defaults();
	(void)plant_config_nvm_load();

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

	/* Top neck (ID 2): usable ~575..2656 on bench; keep margin inside stops. */
	servo_table[1] = (servo_config_t){
		.id = 2,
		.enabled = true,
		.pos_min = 700,
		.pos_max = 2500,
		.position_p_gain = DXL_DEFAULT_POSITION_P_GAIN,
		.position_d_gain = DXL_DEFAULT_POSITION_D_GAIN,
		.default_profile_vel = 180,
		.default_profile_accel = DXL_DEFAULT_PROFILE_ACCEL,
	};

	// LED Initialisation
	led_table[0].default_count = LED_STRIP_MAX;

}
