#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "host/host_exchange_schema.h"

/*
 * Bench thermocouple (MAX31855) on SPI3 — exclusive with SK9822 via spi3_role.
 * When role is THERMO, every feedback image carries a 't' PDU with the latest
 * cached reading (host_exchange pdu[], not a schema layout change).
 *
 * Soft CS: PB7 (CubeMX GPIO_Output, unused by App elsewhere — confirm harness).
 * SO: SPI3 MISO PB4. SCK: SPI3 PB3.
 */

#define PLANT_THERMO_TAG0     'T'
#define PLANT_THERMO_TAG1     'M'
#define PLANT_THERMO_TAG2     'P'
#define PLANT_THERMO_RESP_TAG 't'

#define THERMO_CS_GPIO_Port GPIOB
#define THERMO_CS_Pin       GPIO_PIN_7

void thermo_init(void);
void thermo_service(void);

bool thermo_is_command(const host_command_image_t *cmd);
void thermo_on_command(const host_command_image_t *cmd);

/* Fills pdu with latest reading when spi3_role is THERMO. Must stay on the
 * plant_feedback whitelist or servo_diag_feedback_fill() stomps tag 't'. */
void thermo_feedback_fill(host_pdu_feedback_t *pdu);
