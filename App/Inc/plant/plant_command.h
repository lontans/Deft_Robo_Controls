#pragma once
#include "host/host_exchange_schema.h"
#include <stdbool.h>
#include <stdint.h>

/* system.mcu_state (3 bits) — safety/lifecycle only.
 * Observe vs control is system.plant_apply (wire bit 11), not mcu_state.
 * DIAG_ONLY (2) kept as a deprecated host alias: FW treats it as apply off. */
#define PLANT_MCU_STATE_NORMAL     0u
#define PLANT_MCU_STATE_RECOVERY   1u
#define PLANT_MCU_STATE_DIAG_ONLY  2u /* deprecated: forces plant_apply=0 */
#define PLANT_MCU_STATE_ESTOP      3u

/* Plant cyclic CMDH — desires / mcu_state only; ignores pdb DEBUG tags.
 * TIM6 path: actuators (+ system/ESTOP/rx_sim). Servo/LED via
 * peripheral_command_mount() after host FB TX. */
void plant_command_image_dispatch_plant(const host_command_image_t *cmd);

/* Mount servo + LED desires from a plant CMD image (not on TIM6). */
void peripheral_command_mount(const host_command_image_t *cmd);

/* DEBUG DBGC — debug lanes (preferred) or legacy mailbox in pdb[0..31]. */
void plant_command_image_dispatch_debug(const host_command_image_t *cmd);

/* Last command system.mcu_state handled (echoed in feedback). */
uint8_t plant_command_mcu_state_readback(void);

/* Last plant_apply arm (wire bit 11); false ⇒ observe / no actuator mount. */
bool plant_command_plant_apply_readback(void);

/* Last plant/debug link mode (stm32_mode); Soft-DFU may not return. */
uint8_t plant_command_stm32_mode_readback(void);

/* Update stm32_mode echo from a just-RX'd CMDH (HostTask). Soft-DFU enter
 * still happens only on plant apply (plant_command_image_dispatch_plant). */
void plant_command_observe_stm32_mode(const host_command_image_t *cmd);

/* True when the last DEBUG command used the DL\x01 debug_lanes map. */
bool plant_command_debug_lanes_pending(void);
void plant_command_debug_lanes_clear_pending(void);
uint16_t plant_command_debug_lanes_arm_mask(void);
