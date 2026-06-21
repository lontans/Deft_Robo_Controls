#include "host_exchange.h"

// Static assert functions to ensure data in and out expectations are correct

// Struct sizes
_Static_assert(sizeof(host_image_header_t)      == 12u, "header");
_Static_assert(sizeof(host_system_command_t)    ==  4u, "system cmd");
_Static_assert(sizeof(host_system_feedback_t)   ==  4u, "system fb");
_Static_assert(sizeof(host_actuator_command_t)  == 20u, "actuator cmd");
_Static_assert(sizeof(host_actuator_feedback_t) == 20u, "actuator fb");
_Static_assert(sizeof(host_servo_command_t)     ==  6u, "servo cmd");
_Static_assert(sizeof(host_servo_feedback_t)    ==  6u, "servo fb");
_Static_assert(sizeof(host_led_command_t)       ==  2u, "led cmd");
_Static_assert(sizeof(host_led_feedback_t)      ==  2u, "led fb");
_Static_assert(sizeof(host_pdu_command_t)       == 32u, "pdu cmd");
_Static_assert(sizeof(host_pdu_feedback_t)      == 32u, "pdu fb");

// Actuator macro payload
_Static_assert(HOST_EXCHANGE_ACTUATOR_SLOTS == 25u, "slot count");
_Static_assert(HOST_ACTUATOR_CMD_BYTES == 500u, "actuator cmd block");
_Static_assert(HOST_ACTUATOR_FB_BYTES  == 500u, "actuator fb block");
_Static_assert(HOST_ACTUATOR_CMD_BYTES == HOST_ACTUATOR_FB_BYTES,
               "actuator cmd/fb bytes must match");

// Full image size+symmetry test
_Static_assert(sizeof(host_command_image_t)  == 562u, "command image");
_Static_assert(sizeof(host_feedback_image_t) == 562u, "feedback image");
_Static_assert(sizeof(host_command_image_t) == sizeof(host_feedback_image_t),
               "cmd/fb image size match");
_Static_assert(HOST_COMMAND_IMAGE_BYTES == HOST_FEEDBACK_IMAGE_BYTES, "");

// Offset checks
_Static_assert(offsetof(host_command_image_t, header)            == 0u,  "");
_Static_assert(offsetof(host_command_image_t, system)            == 12u, "");
_Static_assert(offsetof(host_command_image_t, actuator_commands) == 16u, "");
_Static_assert(offsetof(host_feedback_image_t, actuator_feedback) == 16u, "");

_Static_assert(offsetof(host_actuator_command_t, position) == 0u,  "");
_Static_assert(offsetof(host_actuator_command_t, kp)       == 8u,  "");
_Static_assert(offsetof(host_actuator_command_t, torque)     == 16u, "");

_Static_assert(offsetof(host_actuator_feedback_t, temperature) == 12u, "");
_Static_assert(offsetof(host_actuator_feedback_t, fault)       == 16u, "");

/*
 * HOST_LAYOUT_VERSION 1
 *   command/feedback image: 562 bytes
 *   actuator_commands[25]: 20 B/slot (p,v,kp,kd,tau)
 *   actuator_feedback[25]: 20 B/slot (p,v,tau,temp,fault)
 *   pdu: 32 B opaque (v2: structured battery + 6 rails + fan bits)
 */
_Static_assert(HOST_LAYOUT_VERSION == 1u, "update asserts when bumping layout");
