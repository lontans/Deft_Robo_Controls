#include "plant/control_loop.h"
#include "plant/actuator.h"
#include "plant/plugin_schema/plugin_types.h"
#include "main.h"
#include "tim.h"

#define HEARTBEAT_PORT GPIOC
#define HEARTBEAT_PIN  GPIO_PIN_3
#define HEARTBEAT_TOGGLE_EVERY 250u
#define CONTROL_TICK_BURST_MAX 8u

volatile uint32_t g_control_tick_count = 0;
static volatile uint8_t g_control_ticks_pending;

void control_loop_start(void)
{
	HAL_GPIO_TogglePin(HEARTBEAT_PORT, HEARTBEAT_PIN);
	HAL_TIM_Base_Start_IT(&htim6);
}

void control_loop_init(void)
{
	/* Motor enable/wake is host-driven (RS2 PDU or actuator commands). */
}

void control_loop_service(void)
{
	uint8_t n = g_control_ticks_pending;

	if (n == 0u)
		return;

	if (n > CONTROL_TICK_BURST_MAX)
		n = CONTROL_TICK_BURST_MAX;

	g_control_ticks_pending -= n;

	while (n-- > 0u) {
		actuator_apply_desire();
		actuator_capture_state();
	}
}

void control_loop_tick(void)
{
	g_control_tick_count++;
	if ((g_control_tick_count % HEARTBEAT_TOGGLE_EVERY) == 0u)
		HAL_GPIO_TogglePin(HEARTBEAT_PORT, HEARTBEAT_PIN);
	if (g_control_ticks_pending < 255u)
		g_control_ticks_pending++;
}
