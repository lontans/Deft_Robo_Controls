"""Compact feedback log records — accurate wire capture without fat SessionState.

Live UI (`SessionState` / `state.json` / `/api/state`) is a latest-wins snapshot
for operators. Accurate logging is opt-in: one NDJSON line per feedback image
(optional raw frame), enqueued to ``SessionDiskWriter`` so control stays lean.
"""
from __future__ import annotations

import base64
import time
from typing import Any, Dict, List, Optional

from deft_controls_sdk.link.exchange import (
    ACTUATOR_COUNT,
    PDU_OFF,
    PLANT_BLOCK_NAMES,
    parse_actuator_feedback,
    parse_feedback_header,
)

FB_LOG_SCHEMA = "deft_fb_v1"


def build_fb_log_record(
    raw: bytes,
    *,
    host: Optional[Dict[str, Any]] = None,
    include_raw: bool = True,
    include_actuators: bool = True,
) -> Dict[str, Any]:
    """Build one compact log dict from a 672 B feedback image.

    This is cheap relative to ``SessionState.to_dict()`` + grading: header parse,
    optional 14-slot strip, optional base64 of the wire image.
    """
    hdr = parse_feedback_header(raw) or {}
    plant_block = int(hdr.get("plant_block") or 0)
    rec: Dict[str, Any] = {
        "schema": FB_LOG_SCHEMA,
        "t": time.time(),
        "tick": hdr.get("tick"),
        "ack_seq": hdr.get("last_cmd_seq"),
        "mcu_state": hdr.get("mcu_state"),
        "plant_block": plant_block,
        "plant_block_name": hdr.get("plant_block_name")
        or PLANT_BLOCK_NAMES.get(plant_block, str(plant_block)),
        "pdu_tag": hdr.get("pdu_tag"),
        "lap_ms": hdr.get("lap_ms"),
        "lap_max_ms": hdr.get("lap_max_ms"),
        "ticks_pending": hdr.get("ticks_pending"),
    }
    if host:
        rec["host"] = dict(host)
    if include_actuators:
        acts: List[Optional[Dict[str, Any]]] = []
        for slot in range(ACTUATOR_COUNT):
            act = parse_actuator_feedback(raw, slot)
            if act is None:
                acts.append(None)
            else:
                acts.append(
                    {
                        "slot": slot,
                        "position": act["position"],
                        "velocity": act["velocity"],
                        "torque": act["torque"],
                        "temperature": act["temperature"],
                        "fault": act["fault"],
                    }
                )
        rec["actuators"] = acts
    if include_raw:
        rec["raw_b64"] = base64.b64encode(raw).decode("ascii")
        rec["raw_len"] = len(raw)
    pdu = raw[PDU_OFF : PDU_OFF + 32] if len(raw) >= PDU_OFF + 32 else b""
    rec["svd_present"] = len(pdu) >= 3 and pdu[:3] == b"SVD"
    return rec
