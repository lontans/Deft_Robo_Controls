#include "plant/control_loop.h"
#include "plant/actuator.h"
#include "plant/servo.h"
#include "plant/plant_timing.h"
#include "plant/rx_sim/rx_sim_pdu.h"
#include "host/host_link.h"
#include "app.h"
#include "main.h"
#include "tim.h"

#if USE_FREERTOS_SCHEDULER
#include "FreeRTOS.h"
#include "task.h"
#endif

#define HEARTBEAT_PORT GPIOC
#define HEARTBEAT_PIN  GPIO_PIN_3
#define HEARTBEAT_TOGGLE_EVERY 250u
#define CONTROL_TICK_BURST_MAX 1u
#define CONTROL_TICK_PENDING_MAX 255u

volatile uint32_t g_control_tick_count = 0;
static volatile uint8_t g_control_ticks_pending;

#if USE_FREERTOS_SCHEDULER
static TaskHandle_t s_plant_task;
#endif

void control_loop_start(void)
{
	HAL_GPIO_TogglePin(HEARTBEAT_PORT, HEARTBEAT_PIN);
	HAL_TIM_Base_Start_IT(&htim6);
}

void control_loop_init(void)
{
}

#if USE_FREERTOS_SCHEDULER
void control_loop_set_plant_task(void *task_handle)
{
	s_plant_task = (TaskHandle_t)task_handle;
}
#endif

void control_loop_service(void)
{
	uint32_t primask;
	uint8_t n;

	primask = __get_PRIMASK();
	__disable_irq();
	n = g_control_ticks_pending;
	if (n > CONTROL_TICK_BURST_MAX)
		n = CONTROL_TICK_BURST_MAX;
	g_control_ticks_pending -= n;
	__set_PRIMASK(primask);

	if (n == 0u)
		return;

	host_link_apply_pending_plant();

	plant_timing_note_service(n);

	while (n-- > 0u) {
		actuator_apply_desire();
		actuator_capture_state();
		if (n == 0u) {
			/* FB snapshot only — no UART; PeripheralTask owns DXL bus TXN. */
			servo_apply_desire();
			servo_capture_state();
			rx_sim_pdu_tick();
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

#if USE_FREERTOS_SCHEDULER
	/* Handle is NULL until PlantTask starts (after osKernelStart).
	 * TIM6 NVIC must be ≥6 so FromISR is legal (syscall ceiling 5). */
	if (s_plant_task != NULL) {
		BaseType_t hpw = pdFALSE;

		vTaskNotifyGiveFromISR(s_plant_task, &hpw);
		portYIELD_FROM_ISR(hpw);
	}
#endif
}

uint8_t control_loop_pending_get(void)
{
	return g_control_ticks_pending;
}
