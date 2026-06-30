#include "plant/plugins/sk9822.h"
#include "plant/plant_config.h"
#include "spi.h"
#include <string.h>

extern SPI_HandleTypeDef hspi3;

// TX buffer: start bytes, max pixels, end frames according to documentation
#define SK9822_END_MAX (1u + 5u + (LED_STRIP_MAX/16u))
#define SK9822_TX_MAX  (4u + (4u * LED_STRIP_MAX) + SK9822_END_MAX)

static uint8_t g_tx[SK9822_TX_MAX];

// LED frame pixel packing
uint32_t sk9822_pack_pixel(uint8_t brightness_0_31,
						   uint8_t r, uint8_t g, uint8_t b)
{
	uint32_t w = 0xE0000000u;
	w |= ((uint32_t)(brightness_0_31 & 0x1Fu) << 24);
	w |= ((uint32_t)b << 16);
	w |= ((uint32_t)g << 8 );
	w |= (uint32_t)r;
	return w;
}

static void u32_to_bytes_be(uint32_t w, uint8_t out[4])
{
	out[0] = (uint8_t)(w >> 24);
	out[1] = (uint8_t)(w >> 16);
	out[2] = (uint8_t)(w >>  8);
	out[3] = (uint8_t)(w);
}

// End frame
static uint16_t sk9822_end_len(uint16_t n)
{
	return (uint16_t)(1u + 5u + (n / 16u));
}


bool sk9822_transmit (const sk9822_pixel_t *pixels, uint16_t n,
					   uint8_t brightness_0_31)
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

	// Start frame: 4 zero bytes
	memset(p, 0, 4);  // Why are we committing memory for this?
	p += 4;

	// LED Frame: Byte 1 (init+brightness), Byte 2-4 (R,G,B values)
	for (i = 0; i < n; i++) {
		uint32_t frame = sk9822_pack_pixel(brightness_0_31,
										   pixels[i].r,
										   pixels[i].g,
										   pixels[i].b);
		u32_to_bytes_be(frame, p);
		p += 4;
	}

	// End Frame: 8 empty bits * (n-1)/16, rounded up
	*p++ = 0xFFu;
	end_len = sk9822_end_len(n);
	memset(p, 0, end_len -1u);

	return (HAL_SPI_Transmit(&hspi3, g_tx, total, 100u) == HAL_OK);
}
