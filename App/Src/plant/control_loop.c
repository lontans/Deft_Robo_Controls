#include "plant/control_loop.h"
#include "plant/actuator.h"
#include "plant/plugin_schema/plugin_types.h"
#include "plant/can/can_router.h"
#include "plant/plugins/robstride.h"
#include "tim.h"

#define HEARTBEAT_PORT GPIOC
#define HEARTBEAT_PIN  GPIO_PIN_3
#define HEARTBEAT_TOGGLE_EVERY 250u

volatile uint32_t g_control_tick_count = 0;

void control_loop_init(void)
{
	can_frame_t en;
	if (robstride_send_enable(&actuator_table[0], &en) == PLUGIN_OK &&
	    can_tx_enqueue(actuator_table[0].bus, &en) == CAN_OK) {
		can_router_poll();
	}

	HAL_TIM_Base_Start_IT(&htim6);
}

void control_loop_tick(void)
{
	g_control_tick_count++;
	if ((g_control_tick_count % HEARTBEAT_TOGGLE_EVERY) == 0u) {
		HAL_GPIO_TogglePin(HEARTBEAT_PORT, HEARTBEAT_PIN);
	}

	actuator_apply_desire();
	actuator_capture_state();
}
