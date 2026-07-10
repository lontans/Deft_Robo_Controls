#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "host/host_exchange_schema.h"

/*
 * Bench thermocouple (MAX31855) service — rides the existing `pdu` bench-
 * backdoor mechanism (same shape as RS2/DM/DXL/UART-bridge probes) rather
 * than touching the frozen 562 B host_exchange_schema.h v1 layout. Not
 * streamed on every 500 Hz frame — the host tags a command with the TMP
 * PDU, and the next feedback frame mirrors the latest cached reading.
 *
 * SPI instance (SPI2 vs SPI4) and CS pin are TODO — see the placeholders
 * below and the matching TODOs in Core/Src/spi.c (this duplicate assumes
 * SPI4; swap both places together if the breakout is actually on SPI2).
 */

#define PLANT_THERMO_TAG0     'T'
#define PLANT_THERMO_TAG1     'M'
#define PLANT_THERMO_TAG2     'P'
#define PLANT_THERMO_RESP_TAG 't'

/* TODO CONFIRM: placeholder CS pin, matches the SCK/MOSI placeholders in
 * the duplicated Core/Src/spi.c (THERMO_SPI4_*). */
#define THERMO_CS_GPIO_Port GPIOE
#define THERMO_CS_Pin       GPIO_PIN_4

void thermo_init(void);

/* Call unconditionally every app_run() lap — self-gates internally at a low
 * rate (thermal time constants are slow), same shape as led_service(). */
void thermo_service(void);

bool thermo_is_command(const host_command_image_t *cmd);
void thermo_on_command(const host_command_image_t *cmd);

/* Mirrors the latest cached reading into the feedback pdu iff the last
 * command tagged a thermo probe — called from plant_feedback_image_fetch().
 * Must be added to plant_feedback.c's tag-preservation whitelist (see the
 * duplicated plant_feedback.c) or servo_diag_feedback_fill() will stomp it. */
void thermo_feedback_fill(host_pdu_feedback_t *pdu);
