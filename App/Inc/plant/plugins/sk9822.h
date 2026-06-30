#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "host/host_exchange_schema.h"

#define LED_STRIP_MAX 120u
#define LED_STRIP_COUNT 1u  // Only one LED strip on the board

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} sk9822_pixel_t;

typedef struct {
	uint8_t default_count;
} sk9822_config_t;

extern sk9822_config_t led_table[LED_STRIP_COUNT];


// LED frame pixel info
uint32_t sk9822_pack_pixel(uint8_t brightness_0_31,
							uint8_t r, uint8_t g, uint8_t b);

bool sk9822_transmit(const sk9822_pixel_t *pixels, uint16_t n, uint8_t brightness_0_31);

