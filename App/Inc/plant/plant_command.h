#pragma once
#include "host/host_exchange_schema.h"

/* system.mcu_state (3 bits) */
#define PLANT_MCU_STATE_NORMAL     0u
#define PLANT_MCU_STATE_RECOVERY   1u
#define PLANT_MCU_STATE_DIAG_ONLY  2u
#define PLANT_MCU_STATE_ESTOP      3u

/* Plant cyclic CMDH — desires / mcu_state only; ignores pdb DEBUG tags.
 * TIM6 path: actuators (+ system/ESTOP/rx_sim). Servo/LED via
 * peripheral_command_mount() after host FB TX. */
void plant_command_image_dispatch_plant(const host_command_image_t *cmd);

/* Mount servo + LED desires from a plant CMD image (not on TIM6). */
void peripheral_command_mount(const host_command_image_t *cmd);

/* DEBUG DBGC — CFG / RS2 / DM / DFU / thermo mailbox in pdb[0..31]. */
void plant_command_image_dispatch_debug(const host_command_image_t *cmd);

/* Last command system.mcu_state handled (echoed in feedback). */
uint8_t plant_command_mcu_state_readback(void);
