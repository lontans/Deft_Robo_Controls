#include "plant/plant_command.h"
#include "plant/actuator.h"
#include "plant/plant_diag.h"

void plant_command_image_dispatch(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return;

	plant_diag_on_command(cmd);
	actuator_command_mount(cmd);
}
