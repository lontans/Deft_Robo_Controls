#pragma once

#include <stdint.h>

/*
 * Exclusive SPI3 accessory role (SK9822 LEDs vs MAX31855 thermocouple).
 * Both share SCK (PB3); LEDs use MOSI (PB5), thermo SO uses MISO (PB4) + soft CS.
 * Only one owner may touch hspi3 at a time — LED TX is continuous when active.
 *
 * Product default is LED. NVM board-config will persist this later; until then
 * SPI3_ROLE_DEFAULT selects the bring-up owner at compile time.
 */
typedef enum {
	SPI3_ROLE_LED    = 0u,
	SPI3_ROLE_THERMO = 1u,
	SPI3_ROLE_NONE   = 2u,
} spi3_role_t;

#ifndef SPI3_ROLE_DEFAULT
#define SPI3_ROLE_DEFAULT SPI3_ROLE_LED
#endif

spi3_role_t spi3_role_get(void);
void        spi3_role_set(spi3_role_t role);
