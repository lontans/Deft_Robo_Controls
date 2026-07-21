#pragma once
/*
 * SUPERSEDED — do not merge this file. Kept for reference only.
 *
 * The live tree already has a real, working App/Inc/plant/thermo.h that
 * resolved these placeholders differently: SPI3 shared with SK9822 via
 * spi3_role.h (exclusive role arbitration), CS on PB7, full-duplex
 * HAL_SPI_TransmitReceive with dummy TX bytes — not the dedicated
 * half-duplex SPI4 this draft assumed. Verified by diff 2026-07-20:
 * plant_command.c/plant_feedback.c/app.c dispatch already wires thermo_*
 * end to end live. Only the CubeMars-specific files in this workstream
 * (cubemars.h/.c, plugin_table.c's cubemars_ops registration) are still
 * unmerged and relevant — see ../../README.md.
 */
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
