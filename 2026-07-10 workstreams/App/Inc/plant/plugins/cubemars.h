#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "plant/plugin_schema/plugin_types.h"
#include "plant/can/can_frame.h"
#include "plant/actuator.h"

/*
 * CubeMars AK-series "Servo Mode" protocol (extended 29-bit CAN ID).
 * Source: External_Documentation/CubeMars/CubeMars_AK_Driver_Doc_Generalised.pdf §5.1-5.2.
 *
 * CAN ID bits [28:8] = control mode, bits [7:0] = source node ID.
 * MVP implements Position-Speed Loop Mode (6) only — matches the existing
 * teleop usage pattern (position + bounded speed). Duty/Current/CurrentBrake/
 * Speed/SetOrigin are intentionally NOT implemented here; they belong on a
 * future bench/diagnostic path (same shape as the RS2/DM PDU probes), not
 * the hot plant path, until there's a concrete need for them.
 */

typedef enum {
	CUBEMARS_MODE_DUTY          = 0u,
	CUBEMARS_MODE_CURRENT       = 1u,
	CUBEMARS_MODE_CURRENT_BRAKE = 2u,
	CUBEMARS_MODE_SPEED         = 3u,
	CUBEMARS_MODE_POSITION      = 4u,
	CUBEMARS_MODE_SET_ORIGIN    = 5u,
	CUBEMARS_MODE_POS_SPEED     = 6u,
} cubemars_control_mode_t;

/* TODO(hardware, P0 — do not power a motor against these unverified):
 * The vendor doc's Position/Speed sections mix "electrical degrees"/
 * "electrical RPM" language inconsistently. Converting ActuatorDesire's
 * mechanical rad / rad*s^-1 to whatever this protocol actually expects may
 * require the motor's pole-pair count, which is unknown until hardware is
 * in hand. cubemars_pack_tx() currently passes desire->position/velocity
 * through as if they were already in the wire's native units — this is a
 * placeholder, not a verified scaling. Confirm against real hardware before
 * ever commanding a loaded joint. */
#define CUBEMARS_POS_SCALE   10000.0f   /* documented: deg * 10000 -> int32 LSB */
#define CUBEMARS_SPEED_SCALE (1.0f / 10.0f)  /* documented: eRPM / 10 -> int16 LSB */
#define CUBEMARS_ACCEL_SCALE (1.0f / 10.0f)  /* documented: (eRPM/s^2) / 10 -> int16 LSB */

uint32_t cubemars_build_ext_id(cubemars_control_mode_t mode, uint8_t node_id);
bool     cubemars_parse_ext_id(uint32_t ext_id, cubemars_control_mode_t *mode, uint8_t *node_id);

extern const plugin_ops_t cubemars_ops;
