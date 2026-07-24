#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "host/host_exchange_schema.h"

/*
 * Software entry into the STM32 ROM (system memory) bootloader -- the
 * AN2606/AN3155 "jump to bootloader from application" technique.
 *
 * Needed because PB8/BOOT0 is permanently repurposed as FDCAN1_RX on this
 * board (DeftRoboticsControlsPCB.ioc) and can't be used to select boot mode
 * the normal way -- this is the software-only substitute.
 *
 * Flow:
 *   1. Host sends the DFU backdoor PDU (see SOFT_DFU_TAG0..3 below).
 *   2. soft_dfu_on_command() stashes a signature in the reserved .dfu_sig
 *      RAM region (see STM32G474RETX_FLASH.ld) and calls
 *      NVIC_SystemReset(). That region sits outside the _sbss.._ebss range
 *      Reset_Handler zeroes, so it survives the reset untouched.
 *   3. On the next boot, soft_dfu_check_and_jump() (called from main(),
 *      before HAL_Init()/SystemClock_Config()) sees the signature, CONSUMES
 *      it immediately, resets USB FS, remaps system memory, re-enables
 *      IRQs, then jumps into the ROM bootloader instead of continuing
 *      normal boot.
 *   4. At that point, reflash over USB via STM32CubeProgrammer / dfu-util.
 *      Host tool falls back to ST-Link SWD if DF11 never enumerates.
 *
 * System memory base 0x1FFF0000 and USB DFU (PA11/PA12) are from AN2606
 * for STM32G47xxx (bootloader ID 0xD5). Jump must not call HAL_RCC_DeInit()
 * before HAL_Init() -- that hangs with no tick and never enumerates DFU.
 *
 * Failure mode if the vector table looks invalid: check returns and app
 * boots normally (signature already consumed). A hard-fault after jump is
 * still recoverable with reset / power cycle.
 */

#define SOFT_DFU_TAG0 'D'
#define SOFT_DFU_TAG1 'F'
#define SOFT_DFU_TAG2 'U'
#define SOFT_DFU_TAG3 '!'

/* Host Leave DFU Set-Address target: mini vector table that only system-resets.
 * See .soft_dfu_leave_vt in STM32G474RETX_FLASH.ld / soft_dfu.c. */
#define SOFT_DFU_LEAVE_VT_ADDR 0x0803F800u

bool soft_dfu_is_command(const host_command_image_t *cmd);

/* Triggers the reset + bootloader jump. Does not return if cmd matches the
 * DFU tag (unreachable code stops here as a defensive fallback). */
void soft_dfu_on_command(const host_command_image_t *cmd);

/* Call once, first thing in main(), before HAL_Init(). Returns normally if
 * no DFU request is pending. Jumps into the ROM bootloader (never returns)
 * if one is. */
void soft_dfu_check_and_jump(void);
