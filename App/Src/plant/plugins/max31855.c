#include "plant/plugins/max31855.h"

static void cs_assert(GPIO_TypeDef *port, uint16_t pin)
{
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

static void cs_deassert(GPIO_TypeDef *port, uint16_t pin)
{
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

static int32_t sign_extend(uint32_t value, uint8_t bits)
{
	uint32_t sign_bit = 1u << (bits - 1u);
	return (int32_t)((value ^ sign_bit) - sign_bit);
}

bool max31855_read(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin,
                   max31855_reading_t *out)
{
	uint8_t raw[4];
	uint8_t dummy[4] = { 0xFFu, 0xFFu, 0xFFu, 0xFFu };
	HAL_StatusTypeDef st;

	if (hspi == NULL || cs_port == NULL || out == NULL)
		return false;

	cs_assert(cs_port, cs_pin);
	/* Full-duplex: clock 4 bytes; SO lands on MISO. Dummy MOSI bytes may
	 * glitch an attached SK9822 — exclusive spi3_role keeps LEDs off. */
	st = HAL_SPI_TransmitReceive(hspi, dummy, raw, 4u, 10u);
	cs_deassert(cs_port, cs_pin);

	if (st != HAL_OK) {
		out->ok = false;
		out->fault_bits = 0u;
		out->thermocouple_c = 0.0f;
		out->cold_junction_c = 0.0f;
		return false;
	}

	uint32_t word = ((uint32_t)raw[0] << 24) | ((uint32_t)raw[1] << 16) |
	                ((uint32_t)raw[2] << 8) | (uint32_t)raw[3];

	uint32_t tc_raw   = (word >> 18) & 0x3FFFu;
	bool     tc_fault = ((word >> 16) & 0x1u) != 0u;
	uint32_t cj_raw   = (word >> 4) & 0xFFFu;
	uint8_t  faults   = (uint8_t)(word & 0x7u);

	out->thermocouple_c  = (float)sign_extend(tc_raw, 14u) * 0.25f;
	out->cold_junction_c = (float)sign_extend(cj_raw, 12u) * 0.0625f;
	out->fault_bits       = faults;
	out->ok                = !tc_fault && faults == 0u;

	return true;
}
