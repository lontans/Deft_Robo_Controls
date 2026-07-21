#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "plant/plugin_schema/plugin_types.h"
#include "plant/can/can_frame.h"
#include "plant/actuator.h"

/*
 * ZeroErr actuator support — RESERVED SLOT ONLY, not a working plugin.
 *
 * Update 2026-07-20: External_Documentation/zeroerr.md now exists — public
 * GitHub examples + an EDS file (not the gated 219-page official manual)
 * confirm ZeroErr's CANopen drive ("eDriver") really is CiA 402 (SDO/PDO,
 * 0x6040 controlword etc.), NOT a single-frame MIT-style protocol. That
 * resolves TODO #1/#2 below in the "yes, it's CANopen" direction — read
 * that file (and its own "what's still unknown" section — it's assembled
 * from example code, not a verified datasheet) before starting #3. This
 * remains an intentional stub until that real integration work happens.
 *
 * There is no ZeroErr vendor CAN protocol document in this repo's
 * External_Documentation/ *hardware datasheet* sense (CubeMars/Damiao/
 * Dynamixel/RobStride/SK9822/MCP2518FD are real vendor PDFs; zeroerr.md is
 * secondhand, not one). Everything below is a stub whose only job is to
 * make PROTO_ZEROERR safe to select (plugin_pack_tx/plugin_parse_rx
 * cleanly return PLUGIN_ERR_UNSUPPORTED via the generic dispatch in
 * plugin_table.c) without silently doing something wrong.
 *
 * TODO(vendor-doc / design pass, blocks everything below):
 *   1. Confirm which ZeroErr product line + firmware is actually on hand
 *      (eRob spans several rotary actuator models — zeroerr.md flags this
 *      as unconfirmed). Their catalog may not be one uniform interface.
 *   2. Confirmed CANopen (CiA 402) per zeroerr.md — so this single-frame
 *      pack_tx/parse_rx shape is the WRONG integration point. CANopen
 *      needs SDO/PDO mapping and an NMT state machine, a materially bigger
 *      addition than "fill in two functions" — likely touching
 *      can_router.c's RX dispatch, not just a plugin_ops_t pair. Scope
 *      that as its own design pass rather than forcing it through this
 *      shape.
 *   3. Do not fill in zeroerr_pack_tx/parse_rx as a single CAN frame at
 *      all, per #2 — they exist only to make the reserved slot inert.
 *
 * Do not power a motor against this file — it does nothing (by design)
 * until the above is resolved. See External_Documentation/zeroerr.md.
 */

extern const plugin_ops_t zeroerr_ops;
