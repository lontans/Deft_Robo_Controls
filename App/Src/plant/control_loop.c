#include "plant/control_loop.h"
#include "plant/actuator.h"
#include "plant/servo.h"
#include "plant/plant_timing.h"
#include "main.h"
#include "tim.h"

#define HEARTBEAT_PORT GPIOC
#define HEARTBEAT_PIN  GPIO_PIN_3
#define HEARTBEAT_TOGGLE_EVERY 250u
#define CONTROL_TICK_BURST_MAX 8u
#define CONTROL_TICK_PENDING_MAX 255u

volatile uint32_t g_control_tick_count = 0;
static volatile uint8_t g_control_ticks_pending;

void control_loop_start(void)
{
	HAL_GPIO_TogglePin(HEARTBEAT_PORT, HEARTBEAT_PIN);
	HAL_TIM_Base_Start_IT(&htim6);
}

void control_loop_init(void)
{
}

void control_loop_service(void)
{
	uint32_t primask;
	uint8_t n;

	/* g_control_ticks_pending is also written (increment only) from the
	 * priority-0 TIM6 ISR (control_loop_tick()). Reading it and clearing the
	 * serviced count with a plain "-=" is two separate accesses -- if the ISR
	 * fires between this function's load and store, its increment would be
	 * silently overwritten by our stale-based store. Short critical section,
	 * same pattern already used for other main<->TIM6 shared state. */
	primask = __get_PRIMASK();
	__disable_irq();
	n = g_control_ticks_pending;
	if (n > CONTROL_TICK_BURST_MAX)
		n = CONTROL_TICK_BURST_MAX;
	g_control_ticks_pending -= n;
	__set_PRIMASK(primask);

	if (n == 0u)
		return;

	plant_timing_note_service(n);

	while (n-- > 0u) {
		actuator_apply_desire();
		actuator_capture_state();
		if (n == 0u) {
			servo_apply_desire();
			servo_capture_state();
		}
	}
}

void control_loop_tick(void)
{
	g_control_tick_count++;
	if ((g_control_tick_count % HEARTBEAT_TOGGLE_EVERY) == 0u)
		HAL_GPIO_TogglePin(HEARTBEAT_PORT, HEARTBEAT_PIN);
	if (g_control_ticks_pending < CONTROL_TICK_PENDING_MAX)
		g_control_ticks_pending++;
}

uint8_t control_loop_pending_get(void)
{
	return g_control_ticks_pending;
}
