/* SUPERSEDED — do not merge. See thermo.h in this same folder for why;
 * the live App/Src/plant/thermo.c uses SPI3 (shared with SK9822 via
 * spi3_role) + PB7 CS, not the dedicated SPI4 this draft assumed. */
#include "plant/thermo.h"
#include "plant/plugins/max31855.h"
#include "spi.h"
#include "main.h"
#include <string.h>

#define THERMO_PERIOD_MS 200u   /* 5 Hz — thermal time constants are slow; tunable, not hardware-blocked */

static max31855_reading_t s_last_reading;
static uint32_t           s_last_read_ms;
static bool                s_probe_active;

void thermo_init(void)
{
	memset(&s_last_reading, 0, sizeof(s_last_reading));
	s_last_read_ms = 0u;
	s_probe_active = false;

	/* CS idle high — matches spi_can_port.c's convention. */
	HAL_GPIO_WritePin(THERMO_CS_GPIO_Port, THERMO_CS_Pin, GPIO_PIN_SET);
}

void thermo_service(void)
{
	uint32_t now = HAL_GetTick();

	if ((now - s_last_read_ms) < THERMO_PERIOD_MS)
		return;
	s_last_read_ms = now;

	/* TODO CONFIRM: assumes SPI4 (see Core/Src/spi.c TODOs) — swap to
	 * &hspi2 here if the breakout is actually wired to SPI2 instead. */
	(void)max31855_read(&hspi4, THERMO_CS_GPIO_Port, THERMO_CS_Pin, &s_last_reading);
}

bool thermo_is_command(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return false;
	return cmd->pdu.data[0] == (uint8_t)PLANT_THERMO_TAG0 &&
	       cmd->pdu.data[1] == (uint8_t)PLANT_THERMO_TAG1 &&
	       cmd->pdu.data[2] == (uint8_t)PLANT_THERMO_TAG2;
}

void thermo_on_command(const host_command_image_t *cmd)
{
	(void)cmd;
	/* No blocking read triggered here — thermo_service() is the only SPI
	 * access point. This just arms the next feedback frame to mirror the
	 * latest cached reading (same "probe tagged the last command" shape as
	 * the RS2/DM bench backdoors, without their session state machine). */
	s_probe_active = true;
}

void thermo_feedback_fill(host_pdu_feedback_t *pdu)
{
	uint32_t age_ms;

	if (pdu == NULL || !s_probe_active)
		return;
	s_probe_active = false;

	age_ms = HAL_GetTick() - s_last_read_ms;

	pdu->data[0] = (uint8_t)PLANT_THERMO_RESP_TAG;
	pdu->data[1] = s_last_reading.fault_bits;
	pdu->data[2] = s_last_reading.ok ? 1u : 0u;
	pdu->data[3] = 0u; /* reserved */
	memcpy(&pdu->data[4], &s_last_reading.thermocouple_c, sizeof(float));
	memcpy(&pdu->data[8], &s_last_reading.cold_junction_c, sizeof(float));
	memcpy(&pdu->data[12], &age_ms, sizeof(uint32_t));
}
