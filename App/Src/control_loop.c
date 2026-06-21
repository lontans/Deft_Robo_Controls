#include "control_loop.h"
#include "actuator.h"
#include "plugin_types.h"
#include "can_router.h"
#include "plugins/robstride.h"
#include "tim.h"

#define HEARTBEAT_PORT GPIOC
#define HEARTBEAT_PIN  GPIO_PIN_3
#define HEARTBEAT_TOGGLE_EVERY 250u // At 500Hz/250 = 2 Hz blink

static bool motor_enabled = false;
volatile uint32_t g_control_tick_count = 0; // Heartbeat/control tick counter


void control_loop_init(void) {
	can_frame_t en;
	if (robstride_send_enable(&actuator_table[0], &en) == PLUGIN_OK){
		can_tx_enqueue(actuator_table[0].bus, &en);
		can_router_poll(); // One-time poll + timer start

		// Could implement logic to check tx queue is drained, expose tx_queues in header file
		motor_enabled = true;
	}

	HAL_TIM_Base_Start_IT(&htim6); // Initialise 2ms timer
}

void control_loop_tick(void) {

	// Heartbeat implementation/toggle. On every 250th tick (2ms period* 250 = every 0.5s), the HB LED is inverted. E.g 0-250 LOW, 250-500 HIGH, 500-750 LOW
	g_control_tick_count ++;
	if ((g_control_tick_count % HEARTBEAT_TOGGLE_EVERY) == 0u) {
		HAL_GPIO_TogglePin(HEARTBEAT_PORT, HEARTBEAT_PIN);
	}

	actuator_apply();
	actuator_consume();
}
