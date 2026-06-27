#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "host/host_exchange_schema.h"
#include "plant/plugins/dynamixel.h"

typedef host_servo_command_t  servo_desire_t;
typedef host_servo_feedback_t servo_state_t;

#define SERVO_HOST_STALE_MS 500u

extern servo_config_t servo_table[SERVO_COUNT];
extern servo_desire_t   servo_desire_live[SERVO_COUNT];
extern servo_state_t    servo_state_live[SERVO_COUNT];

void servo_init(void);
void servo_command_mount(const host_command_image_t *cmd);

void servo_apply_desire(void);
void servo_capture_state(void);

void servo_feedback_snapshot(host_servo_feedback_t *dst, uint8_t count);

void servo_desire_clear(void);

void servo_diag_feedback_fill(host_pdu_feedback_t *pdu);
