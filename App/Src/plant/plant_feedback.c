#include "plant/plant_feedback.h"
#include "plant/actuator.h"
#include "plant/servo.h"
#include "plant/plant_diag.h"

void plant_feedback_image_fetch(host_feedback_image_t *out)
{
	if (out == NULL)
		return;

	actuator_feedback_snapshot(out->actuator_feedback, HOST_EXCHANGE_ACTUATOR_SLOTS);
	servo_feedback_snapshot(out->servos, HOST_EXCHANGE_SERVO_SLOTS);
	plant_diag_feedback_fill(&out->pdu);
	if (out->pdu.data[0] != (uint8_t)'d')
		servo_diag_feedback_fill(&out->pdu);
}
