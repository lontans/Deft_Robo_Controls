#pragma once

#include "host/host_exchange_schema.h"
#include <stdbool.h>
#include <stdint.h>

/* Plant config over DEBUG mailbox tag 'C','F','G' (not plant desire path). */
#define PLANT_CFG_PDU_TAG0           'C'
#define PLANT_CFG_PDU_TAG1           'F'
#define PLANT_CFG_PDU_TAG2           'G'
#define PLANT_CFG_PDU_RESP_TAG0      'c'
#define PLANT_CFG_PDU_RESP_TAG1      'f'
#define PLANT_CFG_PDU_RESP_TAG2      'g'

#define PLANT_CFG_OP_GET         1u
#define PLANT_CFG_OP_SET         2u
#define PLANT_CFG_OP_SAVE        3u
#define PLANT_CFG_OP_LOAD        4u
#define PLANT_CFG_OP_DEFAULTS    5u
#define PLANT_CFG_OP_GET_PERIPH  6u /* servos + LED + flags (one mailbox) */
#define PLANT_CFG_OP_SET_PERIPH  7u

#define PLANT_CFG_STATUS_OK        0u
#define PLANT_CFG_STATUS_BAD_ARG   1u
#define PLANT_CFG_STATUS_FLASH_ERR 2u
#define PLANT_CFG_STATUS_BAD_CRC   3u

/* NVM / GET_PERIPH flags byte (image + RAM).
 *   bit0     — honor PDU kill for LED / policy (LISTEN_PDU)
 *   bit1..2  — SPI3 accessory role (see spi3_role_t: 0=LED, 1=THERMO, 2=NONE)
 * Old images with only bit0 set keep LED (role 0) — correct product default. */
#define PLANT_CFG_FLAG_LISTEN_PDU     (1u << 0)
#define PLANT_CFG_SPI3_ROLE_SHIFT     1u
#define PLANT_CFG_SPI3_ROLE_MASK      (3u << PLANT_CFG_SPI3_ROLE_SHIFT)
/* Old name — same bit; kept so stray includes still compile during rename. */
#define PLANT_CFG_FLAG_LISTEN_PDB  PLANT_CFG_FLAG_LISTEN_PDU

/* Servo model nibble (NVM / servo_table.model). */
#define PLANT_SERVO_MODEL_XL430         0u
#define PLANT_SERVO_MODEL_XL330_M288    1u /* XL330-M288-T */

bool plant_config_is_command(const host_command_image_t *cmd);
void plant_config_on_command(const host_command_image_t *cmd);
void plant_config_feedback_fill(host_pdu_feedback_t *pdu);

bool plant_config_nvm_load(void);
bool plant_config_nvm_save(void);
void plant_config_load_factory_defaults(void);

/* RAM mirrors of NVM v2 peripheral block (valid after factory/load/set). */
bool plant_config_listen_pdu(void);
void plant_config_set_listen_pdu(bool enable);
/* SPI3 LED vs MAX31855 role — also mirrored in periph flags bits 1..2. */
uint8_t plant_config_spi3_role(void);
void plant_config_set_spi3_role(uint8_t role);
/* Aliases — prefer listen_pdu. */
static inline bool plant_config_listen_pdb(void) { return plant_config_listen_pdu(); }
static inline void plant_config_set_listen_pdb(bool enable)
{
	plant_config_set_listen_pdu(enable);
}
