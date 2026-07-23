#include "host/host_exchange_schema.h"

_Static_assert(sizeof(host_image_header_t)      == 12u, "header");
_Static_assert(sizeof(host_system_command_t)    == 32u, "system cmd");
_Static_assert(sizeof(host_system_feedback_t)   == 32u, "system fb");
_Static_assert(sizeof(host_actuator_command_t)  == 22u, "actuator cmd");
_Static_assert(sizeof(host_actuator_feedback_t) == 22u, "actuator fb");
_Static_assert(sizeof(host_servo_command_t)     ==  6u, "servo cmd");
_Static_assert(sizeof(host_servo_feedback_t)    ==  6u, "servo fb");
_Static_assert(sizeof(host_led_command_t)       ==  2u, "led cmd");
_Static_assert(sizeof(host_led_feedback_t)      ==  2u, "led fb");
_Static_assert(sizeof(host_pdu_command_t)       == 32u, "pdu cmd");
_Static_assert(sizeof(host_pdu_feedback_t)      == 32u, "pdu fb");

_Static_assert(HOST_EXCHANGE_ACTUATOR_SLOTS == 25u, "slot count");
_Static_assert(HOST_ACTUATOR_CMD_BYTES == 550u, "actuator cmd block");
_Static_assert(HOST_ACTUATOR_FB_BYTES  == 550u, "actuator fb block");
_Static_assert(HOST_ACTUATOR_CMD_BYTES == HOST_ACTUATOR_FB_BYTES,
               "actuator cmd/fb bytes must match");

_Static_assert(sizeof(host_command_image_t)  == 672u, "command image");
_Static_assert(sizeof(host_feedback_image_t) == 672u, "feedback image");
_Static_assert(sizeof(host_command_image_t) == sizeof(host_feedback_image_t),
               "cmd/fb image size match");
_Static_assert(HOST_COMMAND_IMAGE_BYTES == HOST_FEEDBACK_IMAGE_BYTES, "");

_Static_assert(offsetof(host_command_image_t, header)            == 0u,  "");
_Static_assert(offsetof(host_command_image_t, system)            == 12u, "");
_Static_assert(offsetof(host_command_image_t, actuator_commands) == 44u, "");
_Static_assert(offsetof(host_feedback_image_t, actuator_feedback) == 44u, "");
_Static_assert(offsetof(host_command_image_t, servos)            == 594u, "");
_Static_assert(offsetof(host_command_image_t, leds)              == 606u, "");
_Static_assert(offsetof(host_command_image_t, pdb)               == 608u, "");
_Static_assert(offsetof(host_command_image_t, pdu)               == 608u, "");

_Static_assert(offsetof(host_actuator_command_t, position) == 0u,  "");
_Static_assert(offsetof(host_actuator_command_t, kp)       == 8u,  "");
_Static_assert(offsetof(host_actuator_command_t, torque)   == 16u, "");
_Static_assert(offsetof(host_actuator_command_t, meta)     == 20u, "");

_Static_assert(offsetof(host_actuator_feedback_t, temperature) == 12u, "");
_Static_assert(offsetof(host_actuator_feedback_t, fault)       == 16u, "");
_Static_assert(offsetof(host_actuator_feedback_t, meta)        == 20u, "");

_Static_assert(offsetof(host_system_feedback_t, act_lap_ms) == 4u, "act lap");
_Static_assert(offsetof(host_system_feedback_t, act_lap_peak_ms) == 6u, "act peak");
_Static_assert(offsetof(host_system_feedback_t, cmd_rx_seq) == 18u, "cmd rx seq");
_Static_assert(offsetof(host_system_feedback_t, cmd_applied_seq) == 22u,
	"cmd applied seq");
_Static_assert(offsetof(host_system_feedback_t, periph_lap_ms) == 26u, "periph lap");
_Static_assert(offsetof(host_system_feedback_t, periph_lap_peak_ms) == 28u,
	"periph peak");

/*
 * HOST_LAYOUT_VERSION 2
 *   command/feedback image: 672 bytes
 *   system: 32 B (health + timing; not PDU)
 *   actuator_commands/feedback[25]: 22 B/slot (20 B MIT + 2 B meta)
 *   pdb[64]: power mirror; DEBUG mailbox = pdb[0..31] (pdu alias)
 */
_Static_assert(HOST_LAYOUT_VERSION == 2u, "update asserts when bumping layout");
