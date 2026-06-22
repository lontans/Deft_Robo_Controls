#include "plant/plant_command.h"
#include "plant/actuator.h"

void plant_command_image_dispatch(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return;

	actuator_command_mount(cmd);
}
