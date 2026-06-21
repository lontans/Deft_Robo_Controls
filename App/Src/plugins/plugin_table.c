#include "plugin_table.h"
#include "plugin.h"
#include "actuator.h"
#include <stddef.h>

extern const plugin_ops_t robstride_ops;

static const plugin_ops_t *const handlers[PROTO_COUNT] = {
	[PROTO_NONE]      = NULL,
	[PROTO_ROBSTRIDE] = &robstride_ops,
	[PROTO_CUBEMARS]  = NULL,
};

plugin_status_t plugin_pack_tx(const actuator_config_t *cfg,
                               const actuator_desire_t *desire,
                               can_frame_t *frame_out)
{
	if (cfg == NULL || desire == NULL || frame_out == NULL)
		return PLUGIN_ERR_PARAM;

	const plugin_ops_t *ops = handlers[cfg->protocol];
	if (ops == NULL || ops->pack_tx == NULL)
		return PLUGIN_ERR_UNSUPPORTED;

	return ops->pack_tx(cfg, desire, frame_out);
}

plugin_status_t plugin_parse_rx(const actuator_config_t *cfg,
                                const can_frame_t *frame_in,
                                actuator_state_t *state_out)
{
	if (cfg == NULL || frame_in == NULL || state_out == NULL)
		return PLUGIN_ERR_PARAM;

	const plugin_ops_t *ops = handlers[cfg->protocol];
	if (ops == NULL || ops->parse_rx == NULL)
		return PLUGIN_ERR_UNSUPPORTED;

	return ops->parse_rx(cfg, frame_in, state_out);
}

void plugin_table_init(void)
{
}
