"""RS02 ext_id decode — matches control_hub.protocol.rs02."""
from __future__ import annotations

from control_hub.protocol.rs02 import decode_ext_id, mms_label


def test_mms_rest() -> None:
    # comm=0x02 style: mode_status in data16 bits 14-15
    ext_id = 0x020070FD
    ext = decode_ext_id(ext_id)
    assert ext.motor_id == 0x70
    assert mms_label(ext.mode_status) in ("rest", "cali", "running")


def test_under_voltage_fault() -> None:
    # status byte bit0 = under_voltage in fault map
    data16 = (0x01 << 8) | 0x70
    ext_id = (0x02 << 24) | (data16 << 8) | 0xFD
    ext = decode_ext_id(ext_id)
    assert "under_voltage" in ext.faults
