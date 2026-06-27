#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "host/host_exchange_schema.h"

#define DXL_PROTO_VERSION 2.0f

#define DXL_ADDR_OPERATING_MODE    11u
#define DXL_ADDR_TORQUE_ENABLE     64u
#define DXL_ADDR_LED               65u
#define DXL_ADDR_PROFILE_ACCEL     108u
#define DXL_ADDR_PROFILE_VELOCITY  112u
#define DXL_ADDR_POSITION_D_GAIN   80u
#define DXL_ADDR_POSITION_P_GAIN   84u

/* Softer defaults for loaded neck (factory P/D = 400; profile vel 0 = max speed). */
#define DXL_DEFAULT_POSITION_P_GAIN   200u
#define DXL_DEFAULT_POSITION_D_GAIN   700u
#define DXL_DEFAULT_PROFILE_ACCEL     40u
#define DXL_ADDR_GOAL_POSITION     116u
#define DXL_ADDR_MOVING            122u
#define DXL_ADDR_HW_ERROR_STATUS   70u
#define DXL_ADDR_PRESENT_VELOCITY  128u
#define DXL_ADDR_PRESENT_POSITION  132u

#define DXL_INST_SYNC_READ  0x82u
#define DXL_INST_SYNC_WRITE 0x83u

#define DXL_BAUD_NUM_2M     4u
#define DXL_BAUD_TOGGLE_2M  2000000u

#define DXL_MODE_POSITION          3u
#define DXL_TORQUE_ON              1u
#define DXL_TORQUE_OFF             0u

#define DXL_POS_TICKS_PER_REV      4096
#define DXL_POS_MAX_TICK           4095

#define DXL_BAUD_RATE              2000000u
#define DXL_TX_TIMEOUT_MS          10u
#define DXL_RX_TIMEOUT_MS          50u
#define DXL_BAUD_SETTLE_MS         2u

#define DXL_MODEL_XL330_M077       1190u
#define DXL_MODEL_XL330_M288       1200u
#define DXL_ADDR_BAUD_RATE         8u
#define DXL_BAUD_NUM_57600         1u
#define DXL_BAUD_NUM_1M            3u
#define DXL_BAUD_TOGGLE_1M         1000000u
#define DXL_BAUD_TOGGLE_57600      57600u

#ifndef DXL_BAUD_PROBE_ID_START
#define DXL_BAUD_PROBE_ID_START    1u
#endif
#ifndef DXL_BAUD_PROBE_ID_END
#define DXL_BAUD_PROBE_ID_END      253u
#endif
#ifndef DXL_BAUD_SEQ_PROBE_MAX
#define DXL_BAUD_SEQ_PROBE_MAX     64u
#endif

/* Set >0 to hold EN_SERV_TX high for N ms at probe start (DMM sanity check). */
#ifndef DXL_PROBE_EN_CANARY_MS
#define DXL_PROBE_EN_CANARY_MS   0u  // Set to 500u for sanity check
#endif

/*
 * Dynamixel TTL bus via SN74LVC2G241 on UART5 (board net names).
 *
 *   PC12 serv_tx  -> 2A (channel 2 input)
 *   2Y            -> serv_data (DATA line to servo connector)
 *   serv_data     -> 1A (channel 1 input, looped back)
 *   1Y            -> PD2 serv_rx (with 3V3 pull-up)
 *
 *   PA1 EN_SERV_TX -> pin 1 (1OE, active-LOW) + pin 7 (2OE, active-HIGH)
 *   PA1 has 10k pulldown -> defaults LOW at power-on.
 *
 * 74LVC2G241 OE polarity: 1OE is active-LOW, 2OE is active-HIGH.
 * Tying both to PA1 makes exactly one channel active at a time:
 *
 *   PA1 = LOW  -> 1OE=0 (RX gate ON:  serv_data -> serv_rx -> PD2)
 *                 2OE=0 (TX gate OFF: PC12 not driving bus)       = RECEIVE
 *
 *   PA1 = HIGH -> 1OE=1 (RX gate OFF: PD2 not driven, no echo)
 *                 2OE=1 (TX gate ON:  PC12 -> serv_data -> servo) = TRANSMIT
 *
 * Sequence per packet:
 *   1. Idle in RX:  PA1=LOW.
 *   2. Flush RX FIFO, then set PA1=HIGH (TX gate on, RX gate off).
 *   3. HAL_UART_Transmit - wait for TC (not just TXE).
 *   4. PA1=HIGH still; hold one-bit-period so stop bit clears the wire.
 *   5. Set PA1=LOW (RX mode). Turnaround delay; drain RX FIFO.
 *   6. Read servo status reply on PD2.
 */
#define DXL_GPIO_EN_PORT           GPIOA
#define DXL_GPIO_EN_PIN            GPIO_PIN_1

/* PA1=HIGH enables TX gate (2OE active-HIGH); PA1=LOW enables RX gate (1OE active-LOW). */
#define DXL_BUS_TX_ACTIVE          GPIO_PIN_SET
#define DXL_BUS_RX_ACTIVE          GPIO_PIN_RESET

#define DXL_PRE_TX_SETTLE_US       5u
#define DXL_TURNAROUND_MIN_US      50u
#define DXL_TURNAROUND_MARGIN_US   20u
#define DXL_BUS_RELEASE_MIN_US     30u

#define SERVO_COUNT                HOST_EXCHANGE_SERVO_SLOTS

typedef struct {
	uint8_t  id;
	bool     enabled;
	uint16_t pos_min;
	uint16_t pos_max;
	uint16_t position_p_gain;
	uint16_t position_d_gain;
	uint16_t default_profile_vel;
	uint16_t default_profile_accel;
} servo_config_t;

#define DXL_ID_BROADCAST     0xFEu
#define DXL_SCAN_RESULT_MAX  8u

#define DXL_HDR0             0
#define DXL_HDR1             1
#define DXL_HDR2             2
#define DXL_RESERVED         3
#define DXL_ID               4
#define DXL_LEN_L            5
#define DXL_LEN_H            6
#define DXL_INST             7
#define DXL_ERR              8
#define DXL_PARAM0           8
#define DXL_STATUS           7

#define DXL_INST_PING        0x01u
#define DXL_INST_READ        0x02u
#define DXL_INST_WRITE       0x03u
#define DXL_STATUS_PKT       0x55u

#define DXL_LOBYTE(w)        ((uint8_t)((w) & 0xFFu))
#define DXL_HIBYTE(w)        ((uint8_t)(((w) >> 8) & 0xFFu))
#define DXL_LOWORD(d)        ((uint16_t)((d) & 0xFFFFu))
#define DXL_HIWORD(d)        ((uint16_t)(((d) >> 16) & 0xFFFFu))

typedef struct {
	uint8_t  id;
	uint16_t model_number;
} dxl_scan_hit_t;

typedef struct {
	uint8_t        status;
	uint8_t        probe_kind;
	uint32_t       baud_rate;
	uint8_t        count;
	dxl_scan_hit_t hits[DXL_SCAN_RESULT_MAX];
} dxl_probe_result_t;

void     dxl_port_init(void);
void     dxl_port_flush_rx(void);
void     dxl_port_delay_ms(uint32_t ms);
bool     dxl_port_set_baud(uint32_t baud);
int      dxl_port_write(const uint8_t *buf, int len);
int      dxl_port_read_byte(uint8_t *byte, uint32_t timeout_ms);
uint32_t dxl_port_millis(void);

/* Debug: last raw HAL status values captured during probe. */
uint8_t  dxl_port_debug_init_hal_st(void);
uint8_t  dxl_port_debug_tx_hal_st(void);
uint8_t  dxl_port_debug_gstate(void);
uint8_t  dxl_port_debug_lock(void);

bool     dxl_ping(uint8_t id);
uint16_t dxl_ping_model_number(uint8_t id);
bool     dxl_find_baud(uint32_t *baud_out, uint8_t id_start, uint8_t id_end);
uint8_t  dxl_scan_ids(uint8_t id_start, uint8_t id_end,
                      dxl_scan_hit_t *hits, uint8_t max_hits);
bool     dxl_write_u8(uint8_t id, uint16_t addr, uint8_t value);
bool     dxl_write_u16(uint8_t id, uint16_t addr, uint16_t value);
bool     dxl_write_u32(uint8_t id, uint16_t addr, uint32_t value);
bool     dxl_read_u32(uint8_t id, uint16_t addr, uint32_t *value_out);
bool     dxl_toggle_ids_baud(uint8_t id_start, uint8_t id_end, uint32_t *new_baud_out);
void     dynamixel_bus_init(void);

bool dxl_sync_write_tx(uint16_t start_addr, uint16_t data_len,
				       const uint8_t *param, uint16_t param_len);

bool dxl_sync_read_tx(uint16_t start_addr, uint16_t data_len,
	                  const uint8_t *ids, uint8_t id_count);

bool dxl_sync_read_rx_one(uint8_t *id_out, uint8_t *data, uint16_t data_len,
				          uint32_t timeout_ms);

void dynamixel_probe_run(uint8_t kind, uint8_t target_id,
                         uint8_t id_start, uint8_t id_end);
void dynamixel_probe_feedback_fill(host_pdu_feedback_t *pdu);
bool dynamixel_probe_feedback_valid(void);
