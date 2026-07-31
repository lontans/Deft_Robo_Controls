"""Offline tests for deft_controls_sdk.config.yam_limits (no COM)."""
from __future__ import annotations

import os
import sys

import numpy as np
import pytest

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from deft_controls_sdk.config import (  # noqa: E402
    ARM_JOINT_COUNT,
    SOFT_MARGIN,
    apply_clear_inset,
    clamp_q7,
    limits_for_side,
    load_yam_limits,
    plan_hold_q7,
    plan_jog_q7,
    soft_limits_q7,
    yam_left_arm_rows,
    yam_product_rows,
)
from deft_controls_sdk.config.yam_limits import J7_MOTOR_HI, J7_MOTOR_LO  # noqa: E402
import deft_controls_sdk.config.yam_limits as yam_limits_mod  # noqa: E402


def test_load_table_has_14_joints_and_mirror():
    table = load_yam_limits()
    assert set(table.keys()) == set(range(1, 15))
    for j in range(1, 8):
        assert table[j].lo == table[j + 7].lo
        assert table[j].hi == table[j + 7].hi
    assert table[1].lo == pytest.approx(-2.61799, abs=1e-4)
    assert table[1].hi == pytest.approx(3.13, abs=1e-4)
    assert table[7].lo == J7_MOTOR_LO
    assert table[7].hi == J7_MOTOR_HI


def test_left_right_soft_limits_match_without_bench():
    lo_l, hi_l = soft_limits_q7("left", use_bench_clear=False)
    lo_r, hi_r = soft_limits_q7("right", use_bench_clear=False)
    assert lo_l.shape == (ARM_JOINT_COUNT,)
    np.testing.assert_allclose(lo_l, lo_r)
    np.testing.assert_allclose(hi_l, hi_r)
    lims = limits_for_side("left")
    assert lo_l[0] == pytest.approx(lims[0].lo + SOFT_MARGIN, abs=1e-6)


def test_clamp_q7_pulls_into_soft_window():
    q = np.array([-1.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    out = clamp_q7(q, "left", use_bench_clear=False)
    lo, hi = soft_limits_q7("left", use_bench_clear=False)
    assert float(out[0]) >= float(lo[0]) - 1e-6
    assert float(out[1]) == pytest.approx(float(lo[1]), abs=1e-5)
    assert float(out[1]) <= float(hi[1])


def test_left_right_clamp_identical_for_same_local_q():
    q = np.array([0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.5], dtype=np.float32)
    np.testing.assert_allclose(
        clamp_q7(q, "left", use_bench_clear=False),
        clamp_q7(q, "right", use_bench_clear=False),
    )


def test_plan_hold_and_jog():
    q = np.array([0.2, 1.0, 1.0, 0.0, 0.0, 0.0, 1.5], dtype=np.float32)
    hold = plan_hold_q7(q, "left")
    np.testing.assert_allclose(hold, clamp_q7(q, "left"))
    jog, note = plan_jog_q7(q, "left", joint=0, delta=0.05)
    assert float(jog[0]) == pytest.approx(float(hold[0]) + 0.05, abs=1e-3)
    assert isinstance(note, str)


def test_apply_clear_inset_conservative():
    lo, hi = apply_clear_inset(-1.0, 1.0, inset=0.08, home=0.0)
    assert lo == pytest.approx(-0.92, abs=1e-6)
    assert hi == pytest.approx(0.92, abs=1e-6)
    lo2, hi2 = apply_clear_inset(0.0, 0.2, inset=0.08, home=0.1)
    assert hi2 - lo2 < 0.2
    assert lo2 < 0.1 < hi2


def test_bench_clear_tightens_left(monkeypatch: pytest.MonkeyPatch):
    blo = np.array([-0.2, 0.9, 0.9, -0.1, -0.1, -0.1, 1.4], dtype=np.float64)
    bhi = np.array([0.2, 1.1, 1.1, 0.1, 0.1, 0.1, 1.6], dtype=np.float64)
    monkeypatch.setattr(yam_limits_mod, "load_bench_clear_left", lambda **_k: (blo, bhi))
    lo, hi = soft_limits_q7("left", use_bench_clear=True)
    np.testing.assert_allclose(lo, blo)
    np.testing.assert_allclose(hi, bhi)
    q = np.array([1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.5], dtype=np.float32)
    out = clamp_q7(q, "left", use_bench_clear=True)
    assert float(out[0]) == pytest.approx(0.2, abs=1e-5)


def test_yam_left_arm_rows_only_ch1_enabled():
    from deft_controls_sdk.link.exchange import ACTUATOR_COUNT as N

    rows = yam_left_arm_rows()
    assert len(rows) == N
    assert len(yam_product_rows()) == N
    for i, (bus, en, proto, mid, _m) in enumerate(rows):
        if i < 7:
            assert en and bus == 1 and proto == 3 and mid == 0x01 + i
        else:
            assert not en
