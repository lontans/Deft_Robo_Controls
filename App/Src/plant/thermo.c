#include "plant/thermo.h"
#include "plant/plugins/max31855.h"
#include "plant/spi3_role.h"
#include "spi.h"
#include "main.h"
#include <string.h>

#define THERMO_PERIOD_MS 200u /* ~5 Hz — thermal time constants are slow */

static max31855_reading_t s_last_reading;
static uint32_t           s_last_read_ms;

void thermo_init(void)
{
	memset(&s_last_reading, 0, sizeof(s_last_reading));
	s_last_read_ms = 0u;

	/* CS idle high (active low). Pin is already OUTPUT_PP from MX_GPIO_Init. */
	HAL_GPIO_WritePin(THERMO_CS_GPIO_Port, THERMO_CS_Pin, GPIO_PIN_SET);

	/* Prime cache so the first feedback frames already carry a reading. */
	if (spi3_role_get() == SPI3_ROLE_THERMO) {
		s_last_read_ms = HAL_GetTick();
		(void)max31855_read(&hspi3, THERMO_CS_GPIO_Port, THERMO_CS_Pin, &s_last_reading);
	}
}

void thermo_service(void)
{
	uint32_t now;

	if (spi3_role_get() != SPI3_ROLE_THERMO)
		return;

	now = HAL_GetTick();
	if ((now - s_last_read_ms) < THERMO_PERIOD_MS)
		return;
	s_last_read_ms = now;

	(void)max31855_read(&hspi3, THERMO_CS_GPIO_Port, THERMO_CS_Pin, &s_last_reading);
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
	/* Optional probe tag — readings are already streamed in feedback when
	 * role is THERMO; keep the handler so host TMP commands stay valid. */
}

void thermo_feedback_fill(host_pdu_feedback_t *pdu)
{
	uint32_t age_ms;

	if (pdu == NULL || spi3_role_get() != SPI3_ROLE_THERMO)
		return;

	/* This runs unconditionally every tick (no request/pending gate like CFG/RS2/DM
	 * diag), so without this check it silently clobbers any reply those modules staged
	 * into the same shared PDU slot this tick — e.g. CFG GET would build a valid
	 * response, thermo would immediately overwrite it, and the host would time out
	 * waiting for a CFG reply it never sees. Only claim the slot if nothing already has. */
	if (pdu->data[0] != 0u)
		return;

	age_ms = HAL_GetTick() - s_last_read_ms;

	pdu->data[0] = (uint8_t)PLANT_THERMO_RESP_TAG;
	pdu->data[1] = s_last_reading.fault_bits;
	pdu->data[2] = s_last_reading.ok ? 1u : 0u;
	pdu->data[3] = 0u;
	memcpy(&pdu->data[4], &s_last_reading.thermocouple_c, sizeof(float));
	memcpy(&pdu->data[8], &s_last_reading.cold_junction_c, sizeof(float));
	memcpy(&pdu->data[12], &age_ms, sizeof(uint32_t));
}
