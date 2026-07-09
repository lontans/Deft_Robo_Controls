"""Bench probe line formatting for RS2 PDU replies."""
from __future__ import annotations

from typing import Optional

from ..protocol.rs02 import comm_label, decode_ext_id, mms_label


def format_probe_line(label: str, motor_id: int, resp: Optional[dict]) -> str:
    if resp is None:
        return f"  ----  {label:<32s}  (no USB feedback — MCU busy or not running plant_diag?)"

    raw_n = resp.get("raw_frames", 0)
    if not resp["found"] and raw_n == 0:
        return (
            f"  ....  {label:<32s}  MCU replied: no ext-CAN RX  "
            f"(probe_kind={resp['probe_kind']} raw={raw_n} — activity LED may still blink on TX)"
        )

    ext = decode_ext_id(resp["ext_id"])
    disc = resp.get("discovered_id") or ext.motor_id or (motor_id & 0xFF)
    valid = disc == (motor_id & 0xFF) and resp["comm_mode"] in (0x02, 0x11, 0x12, 0x18)
    if resp["found"] and resp.get("ext_id"):
        ext = decode_ext_id(resp["ext_id"])
        if ext.mode != resp["comm_mode"]:
            valid = False
    fault_s = ",".join(ext.faults) if ext.faults else "none"
    mms = mms_label(ext.mode_status)
    tag = "HIT" if resp["found"] else "SNIFF"
    if resp["found"] and not valid:
        tag = "NOISE"
    data_hex = resp["can_data"].hex()

    return (
        f"  {tag}  {label:<32s}  ext=0x{resp['ext_id']:08X}  "
        f"motor=0x{disc:02X}  comm={resp['comm_mode']}  mms={mms}  "
        f"faults=[{fault_s}]  T={resp['temperature']:.1f}C  pos={resp['position']:+.4f}  "
        f"data={data_hex}"
    )
