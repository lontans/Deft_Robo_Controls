#include "plant/plant_config.h"
#include "plant/plant_config_nvm.h"
#include "plant/actuator.h"
#include "plant/plugins/dynamixel.h"
#include "plant/plugins/sk9822.h"
#include "plant/plugins/damiao.h"

servo_config_t servo_table[SERVO_COUNT];
sk9822_config_t led_table[LED_STRIP_COUNT];

void plant_config_init(void)
{
	/* Factory already installs a full NVM v2 RAM image (actuators + neck +
	 * LED + listen_pdu). Flash overlays when valid (v2), or absorbs legacy
	 * v1 actuator slots onto that factory base. */
	plant_config_load_factory_defaults();
	(void)plant_config_nvm_load();

	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		actuator_desire_live[i].position = 0.0f;
		actuator_desire_live[i].velocity = 0.0f;
		actuator_desire_live[i].kp       = 0.0f;
		actuator_desire_live[i].kd       = 0.0f;
		actuator_desire_live[i].torque   = 0.0f;
	}
}
