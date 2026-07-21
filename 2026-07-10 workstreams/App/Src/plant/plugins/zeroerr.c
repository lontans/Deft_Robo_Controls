#include "plant/plugins/zeroerr.h"
#include "plant/plugin_schema/plugin.h"

/*
 * Deliberately unimplemented — see zeroerr.h for exactly what's unknown
 * (no vendor doc, protocol family unconfirmed). Returning
 * PLUGIN_ERR_UNSUPPORTED unconditionally is the correct, safe behavior for
 * a reserved-but-not-yet-implemented protocol slot: plugin_pack_tx/
 * plugin_parse_rx (plugin_table.c) already treat that as "do nothing to
 * this slot," the same way an empty PROTO_NONE slot behaves today. This is
 * not a bug to fix quietly — it should stay exactly this loud until a real
 * implementation replaces it.
 */

static plugin_status_t zeroerr_pack_tx(const actuator_config_t *cfg,
                                       const actuator_desire_t *desire,
                                       can_frame_t *frame_out)
{
	(void)cfg;
	(void)desire;
	(void)frame_out;
	return PLUGIN_ERR_UNSUPPORTED;
}

static plugin_status_t zeroerr_parse_rx(const actuator_config_t *cfg,
                                        const can_frame_t *frame_in,
                                        actuator_state_t *state_out)
{
	(void)cfg;
	(void)frame_in;
	(void)state_out;
	return PLUGIN_ERR_UNSUPPORTED;
}

const plugin_ops_t zeroerr_ops = {
	.pack_tx  = zeroerr_pack_tx,
	.parse_rx = zeroerr_parse_rx,
};
