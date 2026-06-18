#include "plugin_table.h"
#include "plugin.h"
#include "actuator.h"
#include <stddef.h>

// Declare externally defined plugin functions (TODO add more besides robstride)
extern const plugin_ops_t robstride_ops;

// Array of plugins, operations nested
static const plugin_ops_t *const handlers[PROTO_COUNT] = {
		[PROTO_NONE]      = NULL,
		[PROTO_ROBSTRIDE] = &robstride_ops,
		[PROTO_CUBEMARS]  = NULL, // TODO add this after Robstride plugins implemented
};

// Use this function to compute desired output CAN frame based on actuator config and specific desire, returns status
// NOTE: acutuator config already includes motor id
plugin_status_t plugin_pack_tx(const actuator_config_t *cfg,
							   const desire_t *desire,
							   can_frame_t *frame_out) {
	if (cfg == NULL || desire == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;
	// Find operations for protocol by looking in handlers
	const plugin_ops_t *ops = handlers[cfg->protocol];

	// Error check if no operations or no pack_tx function found
	if (ops == NULL || ops -> pack_tx == NULL)
		return PLUGIN_ERR_UNSUPPORTED;

	return ops->pack_tx(cfg, desire, frame_out);
}

// Use this function to parse state information from incoming CAN frame sent by specific actuator. State parses into state struct
plugin_status_t plugin_parse_rx(const actuator_config_t *cfg,
							    const can_frame_t *frame_in,
							    state_t *state_out) {
	if (cfg == NULL || frame_in == NULL || state_out == NULL)
		return PLUGIN_ERR_PARAM;

	// Find operations for protocol by looking in handlers
	const plugin_ops_t *ops = handlers[cfg->protocol];

	// Error check if no operations or no parse_rx function found
	if (ops == NULL || ops -> parse_rx == NULL)
		return PLUGIN_ERR_UNSUPPORTED;

	return ops->parse_rx(cfg, frame_in, state_out);
}

void plugin_table_init(void) {
}
