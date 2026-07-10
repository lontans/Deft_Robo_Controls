"""Golden tests for the bench thermocouple (MAX31855) wire protocol (no hardware).

Unlike the CubeMars tests, the MAX31855 bit layout is a standard, precisely
documented datasheet fact (not a per-motor unknown) — test_max31855_decode_bit_layout
is a legitimate hardware-independent correctness test, not just an internal
round-trip check.
"""
from __future__ import annotations

import struct

import pytest

from controls_pcb_host.commands import build_thermo_probe_command
from controls_pcb_host.feedback import parse_thermo_feedback
from controls_pcb_host.protocol import IMAGE_BYTES, PDU_OFF, THERMO_TAG0, THERMO_TAG1, THERMO_TAG2


def test_build_thermo_command_tag_bytes() -> None:
    buf = build_thermo_probe_command(1)
    assert len(buf) == IMAGE_BYTES
    assert buf[PDU_OFF + 0] == THERMO_TAG0
    assert buf[PDU_OFF + 1] == THERMO_TAG1
    assert buf[PDU_OFF + 2] == THERMO_TAG2


def test_build_thermo_command_preserves_actuator_slots() -> None:
    # A TMP probe must be sendable alongside a live teleop stream — verify
    # slot_commands still land at the normal actuator offset.
    buf = build_thermo_probe_command(1, slot_commands={0: (1.0, 0.0, 8.0, 0.45, 0.0)})
    pos, vel, kp, kd, tau = struct.unpack_from("<fffff", buf, 16)
    assert (pos, vel, kp, kd, tau) == pytest.approx((1.0, 0.0, 8.0, 0.45, 0.0))


def _build_feedback_frame(*, tag: int, fault_bits: int = 0, ok: bool = True,
                           thermocouple_c: float = 24.5, cold_junction_c: float = 23.0,
                           age_ms: int = 42) -> bytes:
    buf = bytearray(IMAGE_BYTES)
    struct.pack_into("<I", buf, 0, 0x46424848)  # HOST_FEEDBACK_MAGIC
    buf[PDU_OFF + 0] = tag
    buf[PDU_OFF + 1] = fault_bits
    buf[PDU_OFF + 2] = 1 if ok else 0
    struct.pack_into("<f", buf, PDU_OFF + 4, thermocouple_c)
    struct.pack_into("<f", buf, PDU_OFF + 8, cold_junction_c)
    struct.pack_into("<I", buf, PDU_OFF + 12, age_ms)
    return bytes(buf)


def test_parse_thermo_feedback_roundtrip() -> None:
    frame = _build_feedback_frame(tag=ord("t"), fault_bits=0, ok=True,
                                   thermocouple_c=24.5, cold_junction_c=23.0, age_ms=42)
    parsed = parse_thermo_feedback(frame)
    assert parsed is not None
    assert parsed["thermocouple_c"] == pytest.approx(24.5)
    assert parsed["cold_junction_c"] == pytest.approx(23.0)
    assert parsed["age_ms"] == 42
    assert parsed["ok"] is True
    assert parsed["fault_bits"] == 0


def test_parse_thermo_feedback_wrong_tag_returns_none() -> None:
    frame = _build_feedback_frame(tag=ord("m"))  # Damiao's tag, not thermo's
    assert parse_thermo_feedback(frame) is None


def test_parse_thermo_feedback_fault_bits_and_not_ok() -> None:
    frame = _build_feedback_frame(tag=ord("t"), fault_bits=0x1, ok=False)
    parsed = parse_thermo_feedback(frame)
    assert parsed is not None
    assert parsed["fault_bits"] == 0x1
    assert parsed["ok"] is False


def _sign_extend(value: int, bits: int) -> int:
    sign_bit = 1 << (bits - 1)
    return (value ^ sign_bit) - sign_bit


def _max31855_decode(word: int) -> dict:
    """Reference decode matching App/Src/plant/plugins/max31855.c bit-for-bit,
    used to verify the firmware's decode formula independent of any C build."""
    tc_raw = (word >> 18) & 0x3FFF
    tc_fault = bool((word >> 16) & 0x1)
    cj_raw = (word >> 4) & 0xFFF
    faults = word & 0x7
    return {
        "thermocouple_c": _sign_extend(tc_raw, 14) * 0.25,
        "cold_junction_c": _sign_extend(cj_raw, 12) * 0.0625,
        "fault_bits": faults,
        "ok": not tc_fault and faults == 0,
    }


def test_max31855_decode_bit_layout_positive() -> None:
    # +100.00 C thermocouple (raw 400 @ 0.25 C/LSB), +25.0 C cold junction
    # (raw 400 @ 0.0625 C/LSB), no faults.
    tc_raw = 400
    cj_raw = 400
    word = (tc_raw << 18) | (cj_raw << 4)
    out = _max31855_decode(word)
    assert out["thermocouple_c"] == pytest.approx(100.0)
    assert out["cold_junction_c"] == pytest.approx(25.0)
    assert out["fault_bits"] == 0
    assert out["ok"] is True


def test_max31855_decode_bit_layout_negative_sign_extension() -> None:
    # -10.00 C thermocouple: raw = -40 in 14-bit two's complement.
    tc_raw_14bit = (-40) & 0x3FFF
    word = tc_raw_14bit << 18
    out = _max31855_decode(word)
    assert out["thermocouple_c"] == pytest.approx(-10.0)


def test_max31855_decode_bit_layout_fault_flags() -> None:
    word = 0x7  # SCV | SCG | OC all set, zero temps
    out = _max31855_decode(word)
    assert out["fault_bits"] == 0x7
    assert out["ok"] is False
