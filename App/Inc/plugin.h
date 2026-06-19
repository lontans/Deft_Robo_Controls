// plugin.h — shared protocol handler interface (pack TX / parse RX)
#pragma once
#include "actuator.h"
#include "can_frame.h"

typedef enum {
	PLUGIN_OK = 0,
	PLUGIN_ERR_PARAM,
	PLUGIN_ERR_UNSUPPORTED,
} plugin_status_t;

// Function pointer type definition for plugin tx and rx

// With a tx packet from a plugin, encode the actuator config and desire into an output frame, fed to TX in HAL
typedef plugin_status_t (*plugin_pack_tx_fn)(
	const actuator_config_t *cfg,
	const desire_t *desire,
	can_frame_t *frame_out
	);

// With a received CAN frame, decode the state which will be fed back into RX and HAL
typedef plugin_status_t (*plugin_parse_rx_fn)(
	const actuator_config_t *cfg,
	const can_frame_t *frame_in,
	state_t *state_out
	);

// Interface for packing and parsing frames, functions defines in pointer functions above
typedef struct {
	plugin_pack_tx_fn pack_tx;
	plugin_parse_rx_fn parse_rx;
} plugin_ops_t;
