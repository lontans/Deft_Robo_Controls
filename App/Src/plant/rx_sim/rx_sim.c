#include "plant/rx_sim/rx_sim.h"
#include "plant/rx_sim/rx_sim_led.h"
#include "plant/rx_sim/rx_sim_pdu.h"

static uint8_t s_mask;

void rx_sim_set_mask(uint8_t mask)
{
	s_mask = mask;
	rx_sim_led_set_active((mask & RX_SIM_LED) != 0u);
}

void rx_sim_clear(void)
{
	rx_sim_set_mask(0u);
}

uint8_t rx_sim_mask(void)
{
	return s_mask;
}

void rx_sim_apply_from_reserved(uint32_t reserved)
{
	rx_sim_set_mask((uint8_t)(reserved & 0x0Fu));
}

bool rx_sim_actuator_enabled(void)
{
	return (s_mask & RX_SIM_ACTUATOR) != 0u;
}

bool rx_sim_servo_enabled(void)
{
	return (s_mask & RX_SIM_SERVO) != 0u;
}

bool rx_sim_led_enabled(void)
{
	return (s_mask & RX_SIM_LED) != 0u;
}

bool rx_sim_pdu_enabled(void)
{
	return (s_mask & RX_SIM_PDU) != 0u;
}
