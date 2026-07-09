#include "app.h"
#include "plant/plant_config.h"
#include "plant/actuator.h"
#include "plant/servo.h"
#include "plant/led.h"
#include "plant/plugins/dynamixel.h"
#include "plant/control_loop.h"
#include "plant/plant_diag.h"
#include "plant/can/can_router.h"
#include "host/host_link.h"
#include "host/host_uart_bridge.h"
#include "plant/plugin_schema/plugin_table.h"
#include "main.h"

/* PC1: slow toggle while superloop runs (after main.c drops it LOW post-app_init). */
#define APP_RUN_HEARTBEAT_PORT GPIOC
#define APP_RUN_HEARTBEAT_PIN  GPIO_PIN_1
#define APP_RUN_HEARTBEAT_MS   500u

static uint32_t s_app_run_heartbeat_ms;

void app_init(void)
{
	plugin_table_init();
	actuator_init();
	servo_init();
	led_init();
	plant_config_init();

	dynamixel_bus_init();

	can_router_init();
	host_link_init();
	host_uart_bridge_init();

	control_loop_init();
}

void app_run(void)
{
	uint32_t now = HAL_GetTick();

	host_link_begin_loop();
	host_link_poll_rx();
	plant_diag_service();
	plant_diag_can_router_poll();

#if !USE_FREERTOS_SCHEDULER
	/* Drain pending plant ticks — non-blocking MCP TX keeps this fast. */
	for (uint8_t i = 0; i < 4u; i++) {
		control_loop_service();
		host_link_poll_rx();
		plant_diag_can_router_poll();
	}
#endif

	led_service();

#if !USE_FREERTOS_SCHEDULER
	host_link_poll_tx();
#endif

	plant_diag_can_router_poll();

	if (now - s_app_run_heartbeat_ms >= APP_RUN_HEARTBEAT_MS) {
		s_app_run_heartbeat_ms = now;
		HAL_GPIO_TogglePin(APP_RUN_HEARTBEAT_PORT, APP_RUN_HEARTBEAT_PIN);
	}
}
