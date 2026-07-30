#pragma once
#include <stdint.h>
#include <stddef.h>

#define HOST_COMMAND_MAGIC  0x434D4448u /* "CMDH" plant */
#define HOST_FEEDBACK_MAGIC 0x46424848u /* "HBHF" plant */
#define HOST_DEBUG_COMMAND_MAGIC  0x44424743u /* "DBGC" DEBUG */
#define HOST_DEBUG_FEEDBACK_MAGIC 0x46424744u /* "DBGF" DEBUG */
#define HOST_LAYOUT_VERSION 3u /* Schema v3 — 694 B; see docs/host-exchange-v3.md */

#define HOST_EXCHANGE_ACTUATOR_SLOTS 26u
#define HOST_EXCHANGE_SERVO_SLOTS    2u
#define HOST_EXCHANGE_LED_SLOTS      1u
#define HOST_PDU_PAYLOAD_BYTES       32u /* DEBUG mailbox = pdb[0..31] (legacy) */
#define HOST_PDB_PAYLOAD_BYTES       64u

/* Link role (ADR-004) — distinct from mcu_state (apply/safety).
 * Packed in system.reserved field bits 4..5 ⇒ wire bits 9..10. */
#define HOST_STM32_MODE_BANDWIDTH 0u /* plant CMDH/HBHF only; no debug lanes */
#define HOST_STM32_MODE_DEBUG     1u
#define HOST_STM32_MODE_SOFT_DFU  2u
#define HOST_STM32_MODE_SHIFT     4u /* within system.reserved bitfield */
#define HOST_STM32_MODE_MASK      0x3u

/* Plant apply arm — system.reserved bit 6 ⇒ wire bit 11.
 * 1 = mount/apply actuator desires (subject to lease/probe/stale gates).
 * 0 = observe: stream OK, do not mount or tear down bench lease.
 * Replaces mcu_state=DIAG_ONLY as the observe/control toggle. */
#define HOST_PLANT_APPLY_SHIFT    6u /* within system.reserved bitfield */
#define HOST_PLANT_APPLY_MASK     0x1u

/* Debug lanes (DBGC/DBGF) — ADR-004 / docs/host-contract.md */
#define HOST_DEBUG_LANES_TAG0      'D'
#define HOST_DEBUG_LANES_TAG1      'L'
#define HOST_DEBUG_LANES_VER       1u
#define HOST_DEBUG_LANES_HDR_OFF   12u
#define HOST_DEBUG_LANES_HDR_BYTES 6u
#define HOST_DEBUG_LANE0_OFF 18u
#define HOST_DEBUG_LANE_BYTES 32u
#define HOST_DEBUG_LANE_COUNT 10u
#define HOST_DEBUG_LANE_RS    0u
#define HOST_DEBUG_LANE_CM    1u
#define HOST_DEBUG_LANE_ZE    2u
#define HOST_DEBUG_LANE_DM    3u
#define HOST_DEBUG_LANE_LED   4u
#define HOST_DEBUG_LANE_SERVO 5u
#define HOST_DEBUG_LANE_PDU   6u
#define HOST_DEBUG_LANE_CFG   7u /* reserved→CFG during migration */
#define HOST_DEBUG_LANE_R8    8u
#define HOST_DEBUG_LANE_R9    9u

/* Plant debug bytes (using DEBUG mailbox / pdb[0..31]) */
#define PLANT_DXL_PROBE_SCAN         1u
#define PLANT_DXL_PROBE_PING         2u
#define PLANT_DXL_PROBE_FIND_BAUD    3u

typedef struct __attribute__((packed)) {
	float position;
	float velocity;
	float kp;
	float kd;
	float torque;
	uint16_t meta; /* cmd: reserved 0; host writes 0 */
} host_actuator_command_t;

typedef struct __attribute__((packed)) {
	float position;
	float velocity;
	float torque;
	float temperature;
	uint32_t fault;
	/* protocol:3 bus:3 motor_id:8 flags:2 — see host-exchange-v2.md */
	uint16_t meta;
} host_actuator_feedback_t;

/*
 * system command word0:
 *   bit0      e_stop_ack
 *   bits1..3  mcu_state (NORMAL/RECOVERY/ESTOP; 2=legacy DIAG_ONLY → apply off)
 *   bit4      heartbeat
 *   bits5..8  rx_sim mask  → reserved field bits 0..3
 *   bits9..10 stm32_mode   → reserved field bits 4..5
 *   bit11     plant_apply  → reserved field bit 6
 */
typedef struct __attribute__((packed)) {
	uint32_t e_stop_ack : 1;
	uint32_t mcu_state  : 3;
	uint32_t heartbeat  : 1;
	uint32_t reserved   : 27;
	uint8_t  pad[28];
} host_system_command_t;

typedef struct __attribute__((packed)) {
	uint8_t tag0;     /* 'D' */
	uint8_t tag1;     /* 'L' */
	uint8_t ver;      /* 1 */
	uint8_t flags;
	uint16_t arm_mask; /* bit i = lane i armed */
} host_debug_lanes_header_t;

typedef struct __attribute__((packed)) {
	/* Word0 — same bit meaning as layout v1 system u32 */
	uint32_t control_tick_count  : 12;
	uint32_t e_stop_ack_readback : 1;
	uint32_t mcu_state_readback  : 3;
	uint32_t heartbeat_readback  : 1;
	uint32_t last_command_seq    : 8;
	uint32_t plant_block         : 7;
	/* Plant timing (was SVD/thermo PDU overlay) */
	uint16_t act_lap_ms;       /* PlantTask: last apply+FB TX duration */
	uint16_t act_lap_peak_ms;  /* sticky peak of act_lap_ms (reset via plant_timing_reset_peaks) */
	uint8_t  ticks_svc;
	uint8_t  ticks_pending;
	uint16_t usb_rx_drop;
	uint16_t can_rx_drop;
	uint8_t  kill_state;
	uint8_t  kill_reason;
	uint8_t  estop_sense;
	uint8_t  reserved0;
	/* Offsets 18..31 — CMD header.seq readbacks + peripheral lap.
	 * cmd_rx_seq:      header.seq when HostTask USB-RX staged the image.
	 * cmd_applied_seq: header.seq when PlantTask mounted actuators.
	 * last_command_seq (word0): low 8 bits of cmd_rx_seq (legacy ack).
	 * periph_lap_*: PeripheralTask last/peak service (DXL/LED/SPI3). */
	uint32_t cmd_rx_seq;
	uint32_t cmd_applied_seq;
	uint16_t periph_lap_ms;
	uint16_t periph_lap_peak_ms;
	uint8_t  reserved[2]; /* pad struct to 32 B */
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
	uint16_t mode             : 5;
	uint16_t master_brightness: 5;
	uint16_t led_count        : 6;
} host_led_command_t;

typedef struct __attribute__((packed)) {
	uint16_t mode_readback       : 5;
	uint16_t brightness_readback : 5;
	uint16_t driver_status       : 6;
} host_led_feedback_t;

/* DEBUG mailbox (CFG / RS2 / DM / DFU / thermo) — first 32 of pdb region. */
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

/*
 * Tail: 64 B pdb[]. Anonymous union keeps cmd->pdu / fb->pdu for DEBUG
 * (pdb[0..31]). Full pdb[64] is the power-board mirror (ADR-001).
 */
typedef struct __attribute__((packed)) {
	host_image_header_t header;
	host_system_command_t system;
	host_actuator_command_t actuator_commands[HOST_EXCHANGE_ACTUATOR_SLOTS];
	host_servo_command_t servos[HOST_EXCHANGE_SERVO_SLOTS];
	host_led_command_t leds[HOST_EXCHANGE_LED_SLOTS];
	union {
		uint8_t pdb[HOST_PDB_PAYLOAD_BYTES];
		struct {
			host_pdu_command_t pdu;
			uint8_t pdb_hi[HOST_PDB_PAYLOAD_BYTES - HOST_PDU_PAYLOAD_BYTES];
		};
	};
} host_command_image_t;

typedef struct __attribute__((packed)) {
	host_image_header_t header;
	host_system_feedback_t system;
	host_actuator_feedback_t actuator_feedback[HOST_EXCHANGE_ACTUATOR_SLOTS];
	host_servo_feedback_t servos[HOST_EXCHANGE_SERVO_SLOTS];
	host_led_feedback_t leds[HOST_EXCHANGE_LED_SLOTS];
	union {
		uint8_t pdb[HOST_PDB_PAYLOAD_BYTES];
		struct {
			host_pdu_feedback_t pdu;
			uint8_t pdb_hi[HOST_PDB_PAYLOAD_BYTES - HOST_PDU_PAYLOAD_BYTES];
		};
	};
} host_feedback_image_t;

#define HOST_COMMAND_IMAGE_BYTES  ((uint16_t)sizeof(host_command_image_t))
#define HOST_FEEDBACK_IMAGE_BYTES ((uint16_t)sizeof(host_feedback_image_t))
#define HOST_ACTUATOR_CMD_BYTES \
	(HOST_EXCHANGE_ACTUATOR_SLOTS * (uint16_t)sizeof(host_actuator_command_t))
#define HOST_ACTUATOR_FB_BYTES \
	(HOST_EXCHANGE_ACTUATOR_SLOTS * (uint16_t)sizeof(host_actuator_feedback_t))

/* Actuator meta helpers (feedback). */
#define HOST_ACT_META_PROTOCOL(m) ((uint8_t)((m) & 7u))
#define HOST_ACT_META_BUS(m)      ((uint8_t)(((m) >> 3) & 7u))
#define HOST_ACT_META_MOTOR_ID(m) ((uint8_t)(((m) >> 6) & 0xFFu))
#define HOST_ACT_META_ENABLED(m)  (((m) >> 14) & 1u)
#define HOST_ACT_META_FB_VALID(m) (((m) >> 15) & 1u)
#define HOST_ACT_META_PACK(protocol, bus, motor_id, enabled, fb_valid) \
	((uint16_t)( \
		((uint16_t)(protocol) & 7u) | \
		(((uint16_t)(bus) & 7u) << 3) | \
		(((uint16_t)(motor_id) & 0xFFu) << 6) | \
		((uint16_t)(!!(enabled)) << 14) | \
		((uint16_t)(!!(fb_valid)) << 15)))
