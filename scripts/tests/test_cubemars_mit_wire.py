"""Golden vectors for the CubeMars AK MIT Power Mode wire helpers (PDF §5.3).

No hardware, no COM5 — pure byte-level pack/unpack against the vendor PDF's
bit-field tables (not its buggy sample code; see docs/legacy/rfc/rfc-cubemars-mit-plant.md
§"PDF sample bugs (do not copy)"). Mirrors the intended
App/Src/plant/plugins/cubemars.c MIT path.
"""
from __future__ import annotations

import os
import sys

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import pytest

from deft_controls_sdk.protocol.cubemars_mit import (
    CMD_DISABLE,
    CMD_ENABLE,
    CMD_SET_ZERO,
    CubemarsAkModel,
    KD_MAX,
    KD_MIN,
    KP_MAX,
    KP_MIN,
    P_MAX,
    P_MIN,
    float_to_uint,
    limits_for_model,
    pack_disable,
    pack_enable,
    pack_mit,
    pack_set_zero,
    resolve_rx_can_id,
    uint_to_float,
    unpack_mit_rx,
    unpack_mit_tx,
)


def test_enable_disable_zero_frames_match_pdf_special_codes():
    """PDF §5.3 'Special Can code': FF*7 + opcode, std ID = motor ID."""
    can_id, data = pack_enable(0x05)
    assert can_id == 0x05
    assert data == bytes([0xFF] * 7 + [CMD_ENABLE])
    assert CMD_ENABLE == 0xFC

    can_id, data = pack_disable(0x05)
    assert data == bytes([0xFF] * 7 + [CMD_DISABLE])
    assert CMD_DISABLE == 0xFD

    can_id, data = pack_set_zero(0x05)
    assert data == bytes([0xFF] * 7 + [CMD_SET_ZERO])
    assert CMD_SET_ZERO == 0xFE


def test_special_codes_are_8_bytes():
    for fn in (pack_enable, pack_disable, pack_set_zero):
        _, data = fn(1)
        assert len(data) == 8


@pytest.mark.parametrize("model", list(CubemarsAkModel))
def test_mit_command_round_trip_every_model(model):
    """Round-trip position/velocity/kp/kd/torque through pack_mit/unpack_mit_tx
    for every AK module — proves the shared 16/12/12/12/12-bit layout holds
    regardless of which per-model span is in effect."""
    lim = limits_for_model(model)
    position = lim.p_max * 0.25
    velocity = lim.v_max * 0.5
    kp = 40.0
    kd = 1.0
    torque = lim.t_min * 0.5

    can_id, data = pack_mit(0x07, position, velocity, kp, kd, torque, model=model)
    assert can_id == 0x07
    assert len(data) == 8

    out = unpack_mit_tx(data, model=model)
    assert out["position"] == pytest.approx(position, abs=lim.p_max / 32000.0)
    assert out["velocity"] == pytest.approx(velocity, abs=lim.v_max / 2000.0)
    assert out["kp"] == pytest.approx(kp, abs=KP_MAX / 2000.0)
    assert out["kd"] == pytest.approx(kd, abs=KD_MAX / 2000.0)
    assert out["torque"] == pytest.approx(torque, abs=abs(lim.t_min) / 2000.0)


def test_mit_command_clamps_out_of_range_inputs():
    lim = limits_for_model(CubemarsAkModel.AK80_9)
    _, data = pack_mit(
        1,
        position=lim.p_max * 10,
        velocity=lim.v_max * 10,
        kp=KP_MAX * 10,
        kd=KD_MAX * 10,
        torque=lim.t_max * 10,
        model=CubemarsAkModel.AK80_9,
    )
    out = unpack_mit_tx(data, model=CubemarsAkModel.AK80_9)
    assert out["position"] == pytest.approx(lim.p_max, rel=1e-3)
    assert out["velocity"] == pytest.approx(lim.v_max, rel=1e-3)
    assert out["kp"] == pytest.approx(KP_MAX, rel=1e-3)
    assert out["kd"] == pytest.approx(KD_MAX, rel=1e-3)
    assert out["torque"] == pytest.approx(lim.t_max, rel=1e-3)


def test_mit_feedback_round_trip():
    lim = limits_for_model()
    p_u = float_to_uint(5.0, lim.p_min, lim.p_max, 16)
    v_u = float_to_uint(-2.0, lim.v_min, lim.v_max, 12)
    t_u = float_to_uint(3.0, lim.t_min, lim.t_max, 12)
    fb_payload = bytes(
        [
            3,
            (p_u >> 8) & 0xFF,
            p_u & 0xFF,
            (v_u >> 4) & 0xFF,
            ((v_u & 0x0F) << 4) | ((t_u >> 8) & 0x0F),
            t_u & 0xFF,
            25,  # temperature
            0,  # error code
        ]
    )
    out = unpack_mit_rx(fb_payload)
    assert out is not None
    assert out["motor_id"] == 3.0
    assert out["position"] == pytest.approx(5.0, abs=lim.p_max / 32000.0)
    assert out["velocity"] == pytest.approx(-2.0, abs=lim.v_max / 2000.0)
    assert out["torque"] == pytest.approx(3.0, abs=abs(lim.t_max) / 2000.0)
    assert out["temperature"] == 25.0
    assert out["fault"] == 0.0


def test_mit_feedback_accepts_dlc_6_no_temp_or_error():
    """PDF text says DLC=6 but the field table lists 8 bytes (temp+error) —
    a documented discrepancy (see RFC 'PDF sample bugs'). Accept the shorter
    6-byte form for motion data only; temp/error default to 0 when absent."""
    lim = limits_for_model()
    p_u = float_to_uint(1.0, lim.p_min, lim.p_max, 16)
    payload6 = bytes([1, (p_u >> 8) & 0xFF, p_u & 0xFF, 0, 0, 0])
    out = unpack_mit_rx(payload6)
    assert out is not None
    assert out["temperature"] == 0.0
    assert out["fault"] == 0.0


def test_mit_feedback_rejects_short_payload():
    assert unpack_mit_rx(bytes(5)) is None


def test_rx_can_id_defaults_to_motor_id_per_pdf():
    """PDF: feedback Identifier = '0x00 + Drive ID' — same numeric ID space
    as the command's target, unlike Damiao's separate ESC/Master ID split."""
    assert resolve_rx_can_id(0x05) == 0x05
    assert resolve_rx_can_id(0x05, master_id=0) == 0x05
    assert resolve_rx_can_id(0x05, master_id=0xFFFFFFFF) == 0x05  # DM_MASTER_ID_AUTO sentinel


def test_rx_can_id_honors_explicit_override():
    assert resolve_rx_can_id(0x05, master_id=0x15) == 0x15


def test_per_module_position_range_is_not_the_pdf_sample_bug_value():
    """Regression pin for the RFC's documented vendor-doc contradiction: the
    per-module table (§5.3 p.44) gives +/-12.5 rad for every AK model; the
    'pack_cmd' sample code on the very next page hardcodes +/-95.5 rad
    instead. If this ever flips to 95.5, someone silently reintroduced the
    PDF's own bug — the per-module table, not the sample, is ground truth."""
    assert P_MIN == -12.5
    assert P_MAX == 12.5
    assert P_MIN != -95.5
    assert P_MAX != 95.5
    for model in CubemarsAkModel:
        lim = limits_for_model(model)
        assert lim.p_min == -12.5
        assert lim.p_max == 12.5


def test_per_module_velocity_and_torque_differ():
    """Speed/torque are genuinely per-module (unlike position) — pin a few
    known-distinct rows from the PDF table so a future refactor can't
    collapse them all to one shared constant by accident."""
    ak10 = limits_for_model(CubemarsAkModel.AK10_9)
    ak80_6 = limits_for_model(CubemarsAkModel.AK80_6)
    ak80_80 = limits_for_model(CubemarsAkModel.AK80_80)

    assert (ak10.v_min, ak10.v_max) == (-50.0, 50.0)
    assert (ak10.t_min, ak10.t_max) == (-65.0, 65.0)
    assert (ak80_6.v_min, ak80_6.v_max) == (-76.0, 76.0)
    assert (ak80_6.t_min, ak80_6.t_max) == (-12.0, 12.0)
    assert (ak80_80.v_min, ak80_80.v_max) == (-8.0, 8.0)
    assert (ak80_80.t_min, ak80_80.t_max) == (-144.0, 144.0)


def test_kp_kd_ranges_shared_across_models():
    for model in CubemarsAkModel:
        lim = limits_for_model(model)
        assert (lim.kp_min, lim.kp_max) == (KP_MIN, KP_MAX)
        assert (lim.kd_min, lim.kd_max) == (KD_MIN, KD_MAX)


def test_quantization_map_is_symmetric_unlike_pdf_sample():
    """PDF sample bug #2: float_to_uint divides by (1<<bits) while
    uint_to_float divides by (1<<bits)-1 — asymmetric. Damiao's own helpers
    use (1<<bits)-1 both ways; this module must match Damiao, not the PDF
    sample. A full-scale round trip should land exactly on x_max."""
    raw = float_to_uint(12.5, -12.5, 12.5, 16)
    assert raw == (1 << 16) - 1
    back = uint_to_float(raw, -12.5, 12.5, 16)
    assert back == pytest.approx(12.5, abs=1e-6)


def test_layout_matches_damiao_byte_shape():
    """Byte-for-byte parity check against damiao_pack_tx's own nibble
    interleave (App/Src/plant/plugins/damiao.c) using an independent
    reference packer, not cubemars_mit's own code under test."""

    def reference_damiao_style_pack(p_u, v_u, kp_u, kd_u, t_u):
        return bytes(
            [
                (p_u >> 8) & 0xFF,
                p_u & 0xFF,
                (v_u >> 4) & 0xFF,
                ((v_u & 0x0F) << 4) | ((kp_u >> 8) & 0x0F),
                kp_u & 0xFF,
                (kd_u >> 4) & 0xFF,
                ((kd_u & 0x0F) << 4) | ((t_u >> 8) & 0x0F),
                t_u & 0xFF,
            ]
        )

    lim = limits_for_model(CubemarsAkModel.AK80_9)
    p_u = float_to_uint(1.0, lim.p_min, lim.p_max, 16)
    v_u = float_to_uint(2.0, lim.v_min, lim.v_max, 12)
    kp_u = float_to_uint(40.0, lim.kp_min, lim.kp_max, 12)
    kd_u = float_to_uint(1.0, lim.kd_min, lim.kd_max, 12)
    t_u = float_to_uint(3.0, lim.t_min, lim.t_max, 12)

    _, got = pack_mit(1, 1.0, 2.0, 40.0, 1.0, 3.0, model=CubemarsAkModel.AK80_9)
    expect = reference_damiao_style_pack(p_u, v_u, kp_u, kd_u, t_u)
    assert got == expect


def test_no_invented_clear_fault_opcode():
    """Damiao has 0xFB clear-fault; CubeMars PDF does not — do not invent it."""
    assert CMD_ENABLE == 0xFC
    assert CMD_DISABLE == 0xFD
    assert CMD_SET_ZERO == 0xFE
    assert 0xFB not in (CMD_ENABLE, CMD_DISABLE, CMD_SET_ZERO)


def test_lifecycle_enable_then_idle_mit_golden():
    """Formal TX sequence: ENTER (0xFC) then STREAM MIT including idle kp=kd=0."""
    mid = 0x03
    lim = limits_for_model(CubemarsAkModel.AK80_9)
    can_en, en = pack_enable(mid)
    can_mit, mit = pack_mit(mid, 0.0, 0.0, 0.0, 0.0, 0.0, model=CubemarsAkModel.AK80_9)
    assert can_en == mid and can_mit == mid
    assert en == bytes([0xFF] * 7 + [0xFC])
    # Idle MIT is a real packed frame (not disable). Quantization may sit on
    # mid-code for bipolar spans — allow one LSB of V/T.
    out = unpack_mit_tx(mit, model=CubemarsAkModel.AK80_9)
    assert out["position"] == pytest.approx(0.0, abs=lim.p_max / 32000.0)
    assert out["velocity"] == pytest.approx(0.0, abs=lim.v_max / 2000.0)
    assert out["kp"] == pytest.approx(0.0, abs=KP_MAX / 2000.0)
    assert out["kd"] == pytest.approx(0.0, abs=KD_MAX / 2000.0)
    assert out["torque"] == pytest.approx(0.0, abs=abs(lim.t_max) / 2000.0)
    can_dis, dis = pack_disable(mid)
    assert can_dis == mid and dis == bytes([0xFF] * 7 + [0xFD])


def test_mit_feedback_maps_err_byte_to_fault():
    lim = limits_for_model()
    p_u = float_to_uint(0.0, lim.p_min, lim.p_max, 16)
    payload = bytes(
        [
            7,
            (p_u >> 8) & 0xFF,
            p_u & 0xFF,
            0,
            0,
            0,
            40,
            0xA5,  # err → fault
        ]
    )
    out = unpack_mit_rx(payload, expected_motor_id=7)
    assert out is not None
    assert out["fault"] == 0xA5
    assert out["temperature"] == 40.0


def test_mit_feedback_rejects_wrong_drive_id_in_d0():
    lim = limits_for_model()
    p_u = float_to_uint(0.0, lim.p_min, lim.p_max, 16)
    payload = bytes([9, (p_u >> 8) & 0xFF, p_u & 0xFF, 0, 0, 0, 0, 0])
    assert unpack_mit_rx(payload, expected_motor_id=7) is None
    assert unpack_mit_rx(payload, expected_motor_id=9) is not None


def test_pdf_sample_torque_nibble_bug_not_present():
    """If data[6] reused kp>>8 (PDF sample bug), high torque would corrupt.
    Pack nonzero torque with kp=0 and ensure high nibble of data[6] is t>>8."""
    _, data = pack_mit(
        1,
        0.0,
        0.0,
        0.0,
        0.0,
        9.0,  # within AK80-9 ±18
        model=CubemarsAkModel.AK80_9,
    )
    lim = limits_for_model(CubemarsAkModel.AK80_9)
    t_u = float_to_uint(9.0, lim.t_min, lim.t_max, 12)
    kp_u = float_to_uint(0.0, lim.kp_min, lim.kp_max, 12)
    # Correct layout: data[6] high nibble from kd (0), low from t>>8
    assert (data[6] & 0x0F) == ((t_u >> 8) & 0x0F)
    assert (data[6] & 0x0F) != ((kp_u >> 8) & 0x0F) or (t_u >> 8) == (kp_u >> 8)
