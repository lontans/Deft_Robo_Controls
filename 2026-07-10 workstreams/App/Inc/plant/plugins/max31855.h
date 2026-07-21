#pragma once
/* SUPERSEDED — do not merge. Live App/Inc/plant/plugins/max31855.h uses
 * full-duplex HAL_SPI_TransmitReceive on shared SPI3, not this half-duplex
 * SPI4 assumption. See ../../../plant/thermo.h in this workstream folder. */
#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/*
 * MAX31855 K-type thermocouple-to-digital converter (MW BA017 breakout).
 * Read-only SPI device: SCK, CS-bar, SO — no data-input pin on the real
 * chip. This breakout doesn't expose a separate MISO pin; SO is wired to
 * what the harness calls MOSI, so the caller's SPI_HandleTypeDef must be
 * configured for half-duplex 1-line mode (SPI_DIRECTION_1LINE) — see
 * MX_SPI4_Init() in the duplicated Core/Src/spi.c for the init side.
 *
 * This driver takes the SPI handle and CS pin as parameters (not a global)
 * so the actual peripheral choice (SPI2 vs SPI4 — unconfirmed, see
 * thermo.h) stays isolated to one call site.
 */

typedef struct {
	float   thermocouple_c;
	float   cold_junction_c;
	uint8_t fault_bits;   /* bit2=SCV (short to VCC), bit1=SCG (short to GND), bit0=OC (open circuit) */
	bool    ok;           /* false if SPI transfer failed or the chip-level fault bit (bit16) is set */
} max31855_reading_t;

/* Blocking, short-timeout SPI transaction — same convention as
 * spi_can_port.c (HAL_SPI_Receive with a ~10 ms timeout). Must only be
 * called from thermo_service()'s low-rate scheduling point, never from the
 * 500 Hz control path. */
bool max31855_read(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin,
                   max31855_reading_t *out);
