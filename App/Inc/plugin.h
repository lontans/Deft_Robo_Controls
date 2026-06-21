// plugin.h — shared protocol handler interface (pack TX / parse RX)
#pragma once
#include "plugin_types.h"
#include "actuator.h"
#include "can_frame.h"

typedef plugin_status_t (*plugin_pack_tx_fn)(
	const actuator_config_t *cfg,
	const actuator_desire_t *desire,
	can_frame_t *frame_out
);

typedef plugin_status_t (*plugin_parse_rx_fn)(
	const actuator_config_t *cfg,
	const can_frame_t *frame_in,
	actuator_state_t *state_out
);

typedef struct {
	plugin_pack_tx_fn pack_tx;
	plugin_parse_rx_fn parse_rx;
} plugin_ops_t;
