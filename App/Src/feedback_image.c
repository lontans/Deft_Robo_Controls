#include "feedback_image.h"
#include "actuator.h"
#include "host_link.h"
#include "control_loop.h"
#include <string.h>

void feedback_image_build(host_feedback_image_t *out){
	memset(out, 0, sizeof(*out));
	out -> header.magic              = HOST_FEEDBACK_MAGIC;
	out -> header.layout_version     = HOST_LAYOUT_VERSION;
	out -> header.byte_size          = HOST_FEEDBACK_IMAGE_BYTES;

	out->system.control_tick_count = (uint32_t)(g_control_tick_count & 0xFFFu);
	out->system.last_command_seq   = (uint32_t)(host_link_last_command_seq() & 0xFFu);

	actuator_state_snapshot(out->actuator_feedback, HOST_EXCHANGE_ACTUATOR_SLOTS);
}
