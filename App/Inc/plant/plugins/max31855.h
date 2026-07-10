#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/*
 * MAX31855 K-type thermocouple-to-digital converter (MW BA017 breakout).
 * Read-only SPI: SCK, CS-bar, SO. On this board SO is wired to SPI3 MISO (PB4);
 * SK9822 owns MOSI when in LED role. Callers must only use this while
 * spi3_role is THERMO (exclusive SPI3 owner).
 */

typedef struct {
	float   thermocouple_c;
	float   cold_junction_c;
	uint8_t fault_bits; /* bit2=SCV, bit1=SCG, bit0=OC */
	bool    ok;         /* false if SPI failed or chip fault bit (bit16) set */
} max31855_reading_t;

/* Blocking short-timeout SPI transaction — call from thermo_service() only,
 * never from the 500 Hz control path. */
bool max31855_read(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin,
                   max31855_reading_t *out);
