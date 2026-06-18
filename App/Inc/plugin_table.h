// plugin_table.h — config protocol enum → protocol_handlers[] dispatch
// Declare what control_loop needs to function at a high level

#pragma once
#include "plugin.h"

void plugin_table_init(void);
plugin_status_t plugin_pack_tx(const actuator_config_t *cfg, const desire_t *desire, can_frame_t *frame_out);
plugin_status_t plugin_parse_rx(const actuator_config_t *cfg, const can_frame_t *frame_in, state_t *state_out);
