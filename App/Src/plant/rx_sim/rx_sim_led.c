#include "plant/rx_sim/rx_sim_led.h"
#include "plant/led.h"
#include "plant/spi3_role.h"

static bool        s_forced;
static spi3_role_t s_saved = SPI3_ROLE_THERMO;

void rx_sim_led_set_active(bool enable)
{
	if (enable) {
		if (!s_forced) {
			s_saved = spi3_role_get();
			spi3_role_set(SPI3_ROLE_LED);
			s_forced = true;
			led_sim_kick_test(8u);
		}
		return;
	}

	/* Falling edge only. Calling force-off on every plant apply when the
	 * LED sim bit is clear (normal host LED path) wiped leds[0] FLASH/TEST
	 * every TIM6 tick and blanked the strip. Host mode=OFF / leave_idle
	 * blanks via led_command_mount + led_service. */
	if (!s_forced)
		return;

	led_force_all_off();
	spi3_role_set(s_saved);
	s_forced = false;
}
