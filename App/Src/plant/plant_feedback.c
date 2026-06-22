#include "plant/plant_feedback.h"
#include "plant/actuator.h"

void plant_feedback_image_fetch(host_feedback_image_t *out)
{
	if (out == NULL)
		return;

	actuator_feedback_snapshot(out->actuator_feedback, HOST_EXCHANGE_ACTUATOR_SLOTS);
}
