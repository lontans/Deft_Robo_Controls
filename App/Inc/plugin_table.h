#pragma once
#include "plugin.h"

plugin_status_t plugin_pack_tx(const actuator_config_t *cfg,
                               const actuator_desire_t *desire,
                               can_frame_t *frame_out);
plugin_status_t plugin_parse_rx(const actuator_config_t *cfg,
                                  const can_frame_t *frame_in,
                                  actuator_state_t *state_out);
void plugin_table_init(void);
