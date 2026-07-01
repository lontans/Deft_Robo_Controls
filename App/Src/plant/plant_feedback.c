#include "plant/plant_feedback.h"
#include "plant/actuator.h"
#include "plant/servo.h"
#include "plant/led.h"
#include "plant/plant_diag.h"

void plant_feedback_image_fetch(host_feedback_image_t *out)
{
	if (out == NULL)
		return;

	actuator_feedback_snapshot(out->actuator_feedback, HOST_EXCHANGE_ACTUATOR_SLOTS);
	servo_feedback_snapshot(out->servos, HOST_EXCHANGE_SERVO_SLOTS);
	led_feedback_snapshot(&out->leds[0]);
	plant_diag_feedback_fill(&out->pdu);
	/* Keep RS2 ('r') and Dynamixel ('d') probe PDUs; default to servo SVD otherwise. */
	if (out->pdu.data[0] != (uint8_t)'d' &&
	    out->pdu.data[0] != (uint8_t)PLANT_DIAG_PDU_RESP_TAG)
		servo_diag_feedback_fill(&out->pdu);
}
