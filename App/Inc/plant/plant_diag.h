#pragma once
#include "host/host_exchange_schema.h"
#include <stdint.h>
#include <stdbool.h>

/* Bench PDU scan (layout v1 pdu.data[32], offset 530 in 562 B image). */
#define PLANT_DIAG_PDU_TAG0          'R'
#define PLANT_DIAG_PDU_TAG1          'S'
#define PLANT_DIAG_PDU_TAG2          '2'
#define PLANT_DIAG_PDU_RESP_TAG      'r'

#define PLANT_DIAG_PROBE_FULL        0u
#define PLANT_DIAG_PROBE_ENABLE_CTRL 1u
#define PLANT_DIAG_PROBE_CTRL_ONLY   2u
#define PLANT_DIAG_PROBE_PROMISC     10u
#define PLANT_DIAG_PROBE_RESET       11u
#define PLANT_DIAG_PROBE_ENABLE_ONLY 12u
#define PLANT_DIAG_PROBE_CTRL_FAST   13u
#define PLANT_DIAG_PROBE_PARAREAD    14u
#define PLANT_DIAG_PROBE_PROACTIVE   15u
#define PLANT_DIAG_PROBE_CALI        16u
#define PLANT_DIAG_PROBE_ZERO        17u
#define PLANT_DIAG_PROBE_DATA_SAVE   18u
#define PLANT_DIAG_PROBE_PARAWRITE   19u
#define PLANT_DIAG_SESSION_BEGIN     254u
#define PLANT_DIAG_SESSION_END       255u

/* Host RS2 PDU: pdu.data[11] = FDCAN instance (1=CH1, 2=CH2, 3=CH3). */
#define PLANT_DIAG_PDU_CAN_BUS       11u

bool plant_diag_skip_actuator_can(void);
bool plant_diag_is_rs2_command(const host_command_image_t *cmd);
void plant_diag_on_command(const host_command_image_t *cmd);
void plant_diag_feedback_fill(host_pdu_feedback_t *pdu);
