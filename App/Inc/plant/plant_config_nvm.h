#pragma once

#include "host/host_exchange_schema.h"
#include <stdbool.h>
#include <stdint.h>

/* Bench PDU tag 'C','F','G' — plant actuator table (RAM + flash NVM). */
#define PLANT_CFG_PDU_TAG0           'C'
#define PLANT_CFG_PDU_TAG1           'F'
#define PLANT_CFG_PDU_TAG2           'G'
#define PLANT_CFG_PDU_RESP_TAG0      'c'
#define PLANT_CFG_PDU_RESP_TAG1      'f'
#define PLANT_CFG_PDU_RESP_TAG2      'g'

#define PLANT_CFG_OP_GET      1u
#define PLANT_CFG_OP_SET      2u
#define PLANT_CFG_OP_SAVE     3u
#define PLANT_CFG_OP_LOAD     4u
#define PLANT_CFG_OP_DEFAULTS 5u

#define PLANT_CFG_STATUS_OK        0u
#define PLANT_CFG_STATUS_BAD_ARG   1u
#define PLANT_CFG_STATUS_FLASH_ERR 2u
#define PLANT_CFG_STATUS_BAD_CRC   3u

bool plant_config_is_command(const host_command_image_t *cmd);
void plant_config_on_command(const host_command_image_t *cmd);
void plant_config_feedback_fill(host_pdu_feedback_t *pdu);

bool plant_config_nvm_load(void);
bool plant_config_nvm_save(void);
void plant_config_load_factory_defaults(void);
