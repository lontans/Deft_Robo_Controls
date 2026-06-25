#pragma once
#include "host/host_exchange_schema.h"

/* system.mcu_state (3 bits at command image offset 12) — layout v1 */
#define PLANT_MCU_STATE_NORMAL     0u
#define PLANT_MCU_STATE_RECOVERY   1u
#define PLANT_MCU_STATE_DIAG_ONLY  2u
#define PLANT_MCU_STATE_ESTOP      3u

void plant_command_image_dispatch(const host_command_image_t *cmd);

/* Last command system.mcu_state handled (echoed in feedback byte 12). */
uint8_t plant_command_mcu_state_readback(void);
