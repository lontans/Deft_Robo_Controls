"""joint goto/status — gating + dry-run logic (no hardware; see plan-yam-joint-commands.md P1)."""
from __future__ import annotations

from control_hub.hello_world import run_hello_world
from control_hub.joint_cmd import run_joint_goto
from control_hub.yam_limits import clamp_delta, limit_for_slot, load_yam_limits


def test_absolute_goto_refused_without_absolute_flag(capsys) -> None:
    rc = run_hello_world("COM5", joint=5, to=0.5, dry_run=True)
    assert rc == 2
    assert "requires --absolute" in capsys.readouterr().out


def test_absolute_goto_refused_without_i_know_zeros(capsys) -> None:
    rc = run_hello_world("COM5", joint=5, to=0.5, absolute_limits=True, dry_run=True)
    assert rc == 2
    assert "i-know-zeros" in capsys.readouterr().out


def test_absolute_goto_dry_run_ok_with_gates(capsys) -> None:
    rc = run_hello_world(
        "COM5", joint=5, to=0.5, absolute_limits=True, i_know_zeros=True, dry_run=True
    )
    assert rc == 0
    assert "dry-run: no plant session / no motion" in capsys.readouterr().out


def test_joint_goto_requires_delta_or_to(capsys) -> None:
    rc = run_joint_goto("COM5", joint=1, dry_run=True)
    assert rc == 2
    assert "requires --delta" in capsys.readouterr().out


def test_joint_goto_rejects_both_delta_and_to(capsys) -> None:
    rc = run_joint_goto("COM5", joint=1, delta=0.1, to=0.1, dry_run=True)
    assert rc == 2
    assert "not both" in capsys.readouterr().out


def test_joint_goto_relative_delta_dry_run_matches_hello_world(capsys) -> None:
    rc = run_joint_goto("COM5", joint=7, delta=-0.3, dry_run=True)
    assert rc == 0
    out = capsys.readouterr().out
    assert "flipped delta to +" in out  # J7 hard-stop flip, same as hello-world


def test_j7_hard_stop_flip_is_positive() -> None:
    limits = load_yam_limits()
    lim = limit_for_slot(6, limits)
    d, target, note = clamp_delta(1.20, -0.3, lim)
    assert d > 0
    assert "hard-stop" in note
