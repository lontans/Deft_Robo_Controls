#pragma once

#include <stdint.h>
#include <stdbool.h>

/*
 * Plant load / RX simulation — bench only.
 * Host system.reserved bits (wire word bits 5+):
 *   bit0 ACTUATOR  synthetic RS02 FB into CAN RX rings
 *   bit1 SERVO     soft Dynamixel state machine (no UART; does not block CAN)
 *   bit2 LED       force SPI3 LED role + real SK9822 refresh
 *   bit3 PDU       reserved (stub)
 * hub.set_rx_sim(True) enables ACTUATOR only (bit0). Use set_rx_sim_mask
 * for SERVO/LED children. LED bit force-owns SPI3; clearing it does not
 * blank host-driven LEDs (falling-edge only).
 */

#define RX_SIM_ACTUATOR (1u << 0)
#define RX_SIM_SERVO    (1u << 1)
#define RX_SIM_LED      (1u << 2)
#define RX_SIM_PDU      (1u << 3)

/* Default bench: actuator+LED only. Soft servo is opt-in (bit1) — does not
 * touch UART, but keep it off by default so DXL hardware stays quiet. */
#define RX_SIM_BENCH_MASK \
	(RX_SIM_ACTUATOR | RX_SIM_LED)

void rx_sim_set_mask(uint8_t mask);
void rx_sim_clear(void);
uint8_t rx_sim_mask(void);

/* reserved = host_system_command_t.reserved (low bits). */
void rx_sim_apply_from_reserved(uint32_t reserved);

bool rx_sim_actuator_enabled(void);
bool rx_sim_servo_enabled(void);
bool rx_sim_led_enabled(void);
bool rx_sim_pdu_enabled(void);
