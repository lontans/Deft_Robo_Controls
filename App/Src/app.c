#include "app.h"
#include "plant/plant_config.h"
#include "plant/actuator.h"
#include "plant/servo.h"
#include "plant/led.h"
#include "plant/thermo.h"
#include "plant/spi3_role.h"
#include "plant/plugins/dynamixel.h"
#include "plant/control_loop.h"
#include "plant/plant_diag.h"
#include "plant/can/can_router.h"
#include "host/host_link.h"
#include "host/host_uart_bridge.h"
#include "host/pdb_link.h"
#include "plant/plant_timing.h"
#include "plant/plugin_schema/plugin_table.h"
#include "main.h"

/* PC1: slow toggle while host path runs (after main.c drops it LOW post-app_init). */
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
	thermo_init();
	plant_config_init();

	dynamixel_bus_init();

	can_router_init();
	host_link_init();
	host_uart_bridge_init();
	pdb_link_init();

	control_loop_init();
}

void app_host_service(void)
{
	uint32_t now = HAL_GetTick();

	host_link_begin_loop();
	host_link_poll_rx();
	plant_diag_service();
	pdb_link_service();

	if (now - s_app_run_heartbeat_ms >= APP_RUN_HEARTBEAT_MS) {
		s_app_run_heartbeat_ms = now;
		HAL_GPIO_TogglePin(APP_RUN_HEARTBEAT_PORT, APP_RUN_HEARTBEAT_PIN);
	}
}

void app_plant_service(void)
{
	/* Actuator autonomy lap — PlantTask only (apply + FB TX). */
	plant_timing_lap_begin();
	plant_timing_note_pending_at_lap(control_loop_pending_get());
	control_loop_service();
	host_link_poll_tx();
	plant_timing_lap_end();
}

void app_peripheral_service(void)
{
	/* Peripheral lap — may block on DXL; does not inflate actuator lap_ms. */
	plant_timing_periph_lap_begin();
	host_link_apply_pending_peripheral();
	servo_apply_desire();
	servo_bus_poll();
	if (spi3_role_get() == SPI3_ROLE_LED)
		led_service();
	else if (spi3_role_get() == SPI3_ROLE_THERMO)
		thermo_service();
	plant_diag_can_router_poll();
	plant_timing_periph_lap_end();
}

void app_run(void)
{
	app_host_service();
#if !USE_FREERTOS_SCHEDULER
	app_plant_service();
	app_peripheral_service();
#endif
}
