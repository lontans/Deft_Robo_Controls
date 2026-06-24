#include "app.h"
#include "plant/plant_config.h"
#include "plant/actuator.h"
#include "plant/control_loop.h"
#include "host/host_link.h"
#include "plant/can/can_router.h"
#include "plant/plugin_schema/plugin_table.h"

void app_init(void)
{
	plugin_table_init();
	actuator_init();
	plant_config_init();

	can_router_init();
	host_link_init();

	control_loop_init();
}

void app_run(void)
{
	control_loop_service();
	host_link_poll_rx();
	host_link_poll_tx();
	can_router_poll();
}
