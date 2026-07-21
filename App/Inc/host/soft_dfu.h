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
 *      it immediately, then jumps into the ROM bootloader instead of
 *      continuing normal boot.
 *   4. At that point, reflash over USB via STM32CubeProgrammer's DFU/UART
 *      interface -- no ST-Link needed.
 *
 * UNVERIFIED WITHOUT HARDWARE -- confirm before trusting this:
 *   - SOFT_DFU_SYSMEM_BASE (soft_dfu.c) is written from memory against the
 *     STM32G4 AN2606 table, not cross-checked against silicon or the PDF.
 *   - Never tested against real hardware (no ST-Link available this pass).
 *
 * Failure mode if the base address is wrong: the jump lands in invalid
 * memory and hard-faults. That's recoverable with a normal reset or power
 * cycle -- the signature is consumed *before* the jump is attempted, so the
 * next boot always continues normally rather than retrying a bad jump. Not
 * a brick either way.
 */

#define SOFT_DFU_TAG0 'D'
#define SOFT_DFU_TAG1 'F'
#define SOFT_DFU_TAG2 'U'
#define SOFT_DFU_TAG3 '!'

bool soft_dfu_is_command(const host_command_image_t *cmd);

/* Triggers the reset + bootloader jump. Does not return if cmd matches the
 * DFU tag (unreachable code stops here as a defensive fallback). */
void soft_dfu_on_command(const host_command_image_t *cmd);

/* Call once, first thing in main(), before HAL_Init(). Returns normally if
 * no DFU request is pending. Jumps into the ROM bootloader (never returns)
 * if one is. */
void soft_dfu_check_and_jump(void);
