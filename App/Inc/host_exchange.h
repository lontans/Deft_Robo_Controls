#pragma once
#include <stdint.h>
#include <stddef.h>

#define HOST_COMMAND_MAGIC  0x434D4448u
#define HOST_FEEDBACK_MAGIC 0x46424848u
#define HOST_LAYOUT_VERSION 1u            // Schema v1

#define HOST_EXCHANGE_ACTUATOR_SLOTS 25u
#define HOST_EXCHANGE_SERVO_SLOTS    2u
#define HOST_EXCHANGE_LED_SLOTS      1u
#define HOST_PDU_PAYLOAD_BYTES       32u

typedef struct __attribute__((packed)) {
	float position;
	float velocity;
	float kp;
	float kd;
	float torque;
} host_actuator_command_t;

typedef struct __attribute__((packed)) {
	float position;
	float velocity;
	float torque;
	float temperature;
	uint32_t fault;
} host_actuator_feedback_t;

typedef struct __attribute__((packed)) {
	uint32_t e_stop_ack : 1;
	uint32_t mcu_state  : 3;
	uint32_t heartbeat  : 1;
	uint32_t reserved   : 27;
} host_system_command_t;

typedef struct __attribute__((packed)) {
	uint32_t control_tick_count  : 12;
	uint32_t e_stop_ack_readback : 1;
	uint32_t mcu_state_readback  : 3;
	uint32_t heartbeat_readback  : 1;
	uint32_t last_command_seq    : 8;
	uint32_t reserved            : 7;
} host_system_feedback_t;

typedef struct __attribute__((packed)) {
	int16_t native_step_position;
	int16_t native_speed_unit;
	uint16_t torque_enable  : 1;
	uint16_t led_control    : 1;
	uint16_t operating_mode : 3;
	uint16_t servo_id       : 8;
	uint16_t reserved       : 3;
} host_servo_command_t;

typedef struct __attribute__((packed)) {
	int16_t present_position;
	int16_t present_speed;
	uint16_t moving          : 1;
	uint16_t target_reached  : 1;
	uint16_t err_input_volt  : 1;
	uint16_t err_overheating : 1;
	uint16_t err_overload    : 1;
	uint16_t motor_source_id : 8;
	uint16_t reserved        : 3;
} host_servo_feedback_t;

typedef struct __attribute__((packed)) {
	uint8_t mode             : 5;
	uint8_t reserved         : 3;
	uint8_t master_brightness;
} host_led_command_t;

typedef struct __attribute__((packed)) {
	uint8_t mode_readback  : 5;
	uint8_t driver_status  : 3;
	uint8_t brightness_readback;
} host_led_feedback_t;

typedef struct __attribute__((packed)) {
	uint8_t data[HOST_PDU_PAYLOAD_BYTES];
} host_pdu_command_t;

typedef struct __attribute__((packed)) {
	uint8_t data[HOST_PDU_PAYLOAD_BYTES];
} host_pdu_feedback_t;

typedef struct __attribute__((packed)) {
	uint32_t magic;
	uint16_t layout_version;
	uint16_t byte_size;
	uint32_t seq;
} host_image_header_t;

typedef struct __attribute__((packed)) {
	host_image_header_t header;
	host_system_command_t system;
	host_actuator_command_t actuator_commands[HOST_EXCHANGE_ACTUATOR_SLOTS];
	host_servo_command_t servos[HOST_EXCHANGE_SERVO_SLOTS];
	host_led_command_t leds[HOST_EXCHANGE_LED_SLOTS];
	host_pdu_command_t pdu;
} host_command_image_t;

typedef struct __attribute__((packed)) {
	host_image_header_t header;
	host_system_feedback_t system;
	host_actuator_feedback_t actuator_feedback[HOST_EXCHANGE_ACTUATOR_SLOTS];
	host_servo_feedback_t servos[HOST_EXCHANGE_SERVO_SLOTS];
	host_led_feedback_t leds[HOST_EXCHANGE_LED_SLOTS];
	host_pdu_feedback_t pdu;
} host_feedback_image_t;

// Definitions for verifying image bytes, important actuator bytes
#define HOST_COMMAND_IMAGE_BYTES  ((uint16_t)sizeof(host_command_image_t))
#define HOST_FEEDBACK_IMAGE_BYTES ((uint16_t)sizeof(host_feedback_image_t))
#define HOST_ACTUATOR_CMD_BYTES \
	(HOST_EXCHANGE_ACTUATOR_SLOTS * (uint16_t)sizeof(host_actuator_command_t))
#define HOST_ACTUATOR_FB_BYTES \
	(HOST_EXCHANGE_ACTUATOR_SLOTS * (uint16_t)sizeof(host_actuator_feedback_t))
