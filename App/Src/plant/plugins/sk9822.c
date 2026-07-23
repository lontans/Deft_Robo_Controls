#include "plant/plugins/sk9822.h"
#include "plant/plant_config.h"
#include "spi.h"
#include <string.h>

extern SPI_HandleTypeDef hspi3;

/* TX buffer: start bytes, max pixels, end frames according to documentation */
#define SK9822_END_MAX (1u + 5u + (LED_STRIP_MAX / 16u))
#define SK9822_TX_MAX  (4u + (4u * LED_STRIP_MAX) + SK9822_END_MAX)

static uint8_t g_tx[SK9822_TX_MAX];
static volatile bool s_tx_busy;

uint32_t sk9822_pack_pixel(uint8_t brightness_0_31,
                           uint8_t r, uint8_t g, uint8_t b)
{
	uint32_t w = 0xE0000000u;
	w |= ((uint32_t)(brightness_0_31 & 0x1Fu) << 24);
	w |= ((uint32_t)b << 16);
	w |= ((uint32_t)g << 8);
	w |= (uint32_t)r;
	return w;
}

static void u32_to_bytes_be(uint32_t w, uint8_t out[4])
{
	out[0] = (uint8_t)(w >> 24);
	out[1] = (uint8_t)(w >> 16);
	out[2] = (uint8_t)(w >> 8);
	out[3] = (uint8_t)(w);
}

static uint16_t sk9822_end_len(uint16_t n)
{
	return (uint16_t)(1u + 5u + (n / 16u));
}

static bool sk9822_pack(const sk9822_pixel_t *pixels, uint16_t n,
                        uint8_t brightness_0_31, uint16_t *total_out)
{
	uint8_t *p;
	uint16_t total;
	uint16_t i;
	uint16_t end_len;

	if (pixels == NULL || n == 0u || n > LED_STRIP_MAX)
		return false;

	total = (uint16_t)(4u + 4u * n + sk9822_end_len(n));
	if (total > SK9822_TX_MAX)
		return false;

	p = g_tx;
	memset(p, 0, 4);
	p += 4;

	for (i = 0; i < n; i++) {
		uint32_t frame = sk9822_pack_pixel(brightness_0_31,
		                                   pixels[i].r,
		                                   pixels[i].g,
		                                   pixels[i].b);
		u32_to_bytes_be(frame, p);
		p += 4;
	}

	*p++ = 0xFFu;
	end_len = sk9822_end_len(n);
	memset(p, 0, end_len - 1u);

	*total_out = total;
	return true;
}

bool sk9822_tx_busy(void)
{
	return s_tx_busy;
}

bool sk9822_transmit(const sk9822_pixel_t *pixels, uint16_t n,
                     uint8_t brightness_0_31)
{
	uint16_t total;

	if (s_tx_busy)
		return false;
	if (!sk9822_pack(pixels, n, brightness_0_31, &total))
		return false;

	s_tx_busy = true;
	if (HAL_SPI_Transmit_IT(&hspi3, g_tx, total) != HAL_OK) {
		s_tx_busy = false;
		return false;
	}
	return true;
}

bool sk9822_transmit_blocking(const sk9822_pixel_t *pixels, uint16_t n,
                              uint8_t brightness_0_31)
{
	uint16_t total;
	uint32_t t0;

	/* Wait out an in-flight IT TX (force-off / role switch). */
	t0 = HAL_GetTick();
	while (s_tx_busy) {
		if ((HAL_GetTick() - t0) > 50u) {
			(void)HAL_SPI_Abort(&hspi3);
			s_tx_busy = false;
			break;
		}
	}

	if (!sk9822_pack(pixels, n, brightness_0_31, &total))
		return false;

	return (HAL_SPI_Transmit(&hspi3, g_tx, total, 100u) == HAL_OK);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == NULL || hspi->Instance != SPI3)
		return;
	/* SPI3 is LED/thermo exclusive via spi3_role; clear busy either way. */
	s_tx_busy = false;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == NULL || hspi->Instance != SPI3)
		return;
	s_tx_busy = false;
}
