#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "plant/can/mcp2518fd.h"

/*
 * SPI1 @ PCLK2=85 MHz, prescaler /8 → SCK ≈ 10.625 MHz (MCP2518 max 20 MHz).
 * nCS idle high: rail0=PB11 (CH4), rail1=PB1 (CH5), rail2=PA4 (CH6).
 *
 * MCP2518 INT (open-drain, active-low): CH4/rail0=PB10, CH5/rail1=PB0, CH6/rail2=PA3.
 * Board has no external pull-up — firmware enables GPIO_PULLUP in gpio.c + irq_init().
 */
#define MCP_INT_RAIL0_PORT  GPIOB
#define MCP_INT_RAIL0_PIN   GPIO_PIN_10
#define MCP_INT_RAIL1_PORT  GPIOB
#define MCP_INT_RAIL1_PIN   GPIO_PIN_0
#define MCP_INT_RAIL2_PORT  GPIOA
#define MCP_INT_RAIL2_PIN   GPIO_PIN_3

void spi_can_port_init(void);
void spi_can_port_irq_init(void);
bool spi_can_port_xfer(uint8_t rail, const uint8_t *tx, uint8_t *rx, uint16_t len);
bool spi_can_port_int_active(uint8_t rail);
