#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "host/host_exchange_schema.h"

/*
 * Software entry into the STM32 ROM (system memory) USB DFU bootloader.
 *
 * PB8/BOOT0 is FDCAN1_RX on this board, so the pin cannot select boot mode.
 * Soft MEMRMP jumps into system memory often leave CDC dead without ever
 * enumerating 0483:DF11 on this hardware. The reliable path programs the
 * nBOOT0 option byte (nSWBOOT0 stays 0) and resets:
 *
 * Flow:
 *   1. Preferred: host sends plant/DEBUG with stm32_mode=SOFT_DFU (ADR-004).
 *      Legacy: DEBUG mailbox tag SOFT_DFU_TAG0..3 = "DFU!".
 *   2. soft_dfu_on_command() drops USB D+, programs nBOOT0=0, OBL_LAUNCH.
 *      MCU resets into system memory → host sees 0483:DF11.
 *   3. Host programs flash (CubeProg USB DFU or dfu-util).
 *   4. Host Leave (AN3156) jumps to SOFT_DFU_LEAVE_VT_ADDR; the trampoline
 *      programs nBOOT0=1 and OBL_LAUNCH so the next boot is the app (CDC).
 *
 * Legacy: if option-byte launch somehow returns, a RAM .dfu_sig + soft jump
 * path remains (soft_dfu_check_and_jump in main before HAL_Init).
 *
 * System memory base 0x1FFF0000 / USB DFU PA11/PA12: AN2606 STM32G47xxx.
 */

#define SOFT_DFU_TAG0 'D'
#define SOFT_DFU_TAG1 'F'
#define SOFT_DFU_TAG2 'U'
#define SOFT_DFU_TAG3 '!'

/* Host Leave DFU Set-Address target: mini vector table that restores
 * nBOOT0=1 then resets. See .soft_dfu_leave_vt in the linker script. */
#define SOFT_DFU_LEAVE_VT_ADDR 0x0803F800u

/* Legacy DEBUG mailbox tag "DFU!" — prefer stm32_mode=SOFT_DFU. */
bool soft_dfu_is_command(const host_command_image_t *cmd);

/* Triggers option-byte boot into ROM DFU. Does not return on success. */
void soft_dfu_on_command(const host_command_image_t *cmd);

/* Call once, first thing in main(), before HAL_Init(). Legacy soft-jump
 * fallback if a prior reset left .dfu_sig armed. */
void soft_dfu_check_and_jump(void);
