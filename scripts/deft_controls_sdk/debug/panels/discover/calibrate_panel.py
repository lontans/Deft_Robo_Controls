"""``calibrate_robstride_panel`` — thin dashboard wrapper around
``DebugAPI.calibrate_robstride`` (RS02 encoder calibration).

Routes through ``proxy.hub.debug.calibrate_robstride`` (see
``deft_controls_sdk/debug/__init__.py``), which is a clean 1:1 forward to
``deft_controls_sdk/debug/robstride_calibrate.py:calibrate`` (same keyword
args, same ``bool`` return) plus the ``require_debug_mode`` gate — so this
wrapper has nothing to reimplement, it only shapes the result into a dict
and surfaces the preconditions/warnings the caller needs to see.

RobStride-only: there is no damiao / cubemars / zeroerr calibrate
equivalent anywhere in this codebase, so this panel does not attempt to
support other protocols.

**Critical — do not add another lease/pause-stream wrapper here.**
``calibrate()`` (the real primitive) already wraps itself in
``pause_plant_stream(connection)`` and ``lease(connection, telemetry,
bus=bus)`` internally (see ``robstride_calibrate.py``). This panel function
calls ``proxy.hub.debug.calibrate_robstride(...)`` directly and must NOT
wrap that call in another ``pause_plant_stream``/``lease`` context manager
of its own — doing so would double-acquire the session lease / stream pause
and likely deadlock or raise. (See
``test_debug_panels_calibrate.py::test_calibrate_panel_does_not_double_wrap_lease_or_pause_stream``.)

Real preconditions (from ``robstride_calibrate.py`` / RS02 firmware docs),
surfaced here rather than silently swallowed:

- Supply must be 24-60 V. The underlying ``calibrate()`` reads VBUS and
  *prints* a warning when it is out of range but does not hard-enforce it
  (motor cal may simply fail/verify-false if supply is wrong) — this panel
  captures any such printed ``WARNING:`` lines into the returned dict's
  ``warnings`` list so a caller doesn't have to scrape stdout itself.
- **The shaft must be free to spin** during the cali listen window (no load
  on the output) — the encoder cal sequence (reset -> iq_test -> 0x05 cali
  -> 0x06 zero -> 0x16 save) actively spins the motor to characterize the
  encoder. Calibrating under load will produce a bad calibration or a
  verify-false result. There is no software interlock for this — it is on
  the caller to ensure it physically.
- Motor must be at rest before starting; the underlying ``calibrate()``
  already retries reset internally when it observes a running state, so
  this panel does not duplicate that handling.

This call is blocking for roughly 15-35+ seconds (dominated by
``cal_listen_s``, default 28s) — that is expected and this panel does not
attempt to background it; any backgrounding is the dashboard HTTP layer's
concern.
"""
from __future__ import annotations

import io
import sys
from contextlib import contextmanager
from typing import TYPE_CHECKING, Iterator, List

if TYPE_CHECKING:
    from deft_controls_sdk.host_proxy import HostProxy

__all__ = ["calibrate_robstride_panel"]

PRECONDITION_NOTE = (
    "RS02 encoder calibration precondition: supply 24-60 V, shaft free to "
    "spin (no load on the output) for the full cal_listen_s window, motor "
    "at rest before starting. These are not software-enforced beyond the "
    "VBUS warning captured in `warnings` — verify the mechanical/electrical "
    "state before calling."
)


class _StdoutTee(io.TextIOBase):
    """Mirrors writes to the real stdout while also buffering them.

    ``calibrate()`` reports progress / WARNING lines via ``print()`` — this
    lets the panel recover those WARNING lines into the returned dict
    without losing the live console output a notebook/CLI caller expects.
    """

    def __init__(self, original: "io.TextIOBase") -> None:
        self._original = original
        self._buffer = io.StringIO()

    def write(self, s: str) -> int:  # type: ignore[override]
        self._original.write(s)
        self._buffer.write(s)
        return len(s)

    def flush(self) -> None:  # type: ignore[override]
        self._original.flush()

    @property
    def text(self) -> str:
        return self._buffer.getvalue()


@contextmanager
def _capture_stdout() -> Iterator[_StdoutTee]:
    original = sys.stdout
    tee = _StdoutTee(original)
    sys.stdout = tee
    try:
        yield tee
    finally:
        sys.stdout = original


def _extract_warnings(text: str) -> List[str]:
    return [line.strip() for line in text.splitlines() if "WARNING" in line]


def calibrate_robstride_panel(
    proxy: "HostProxy",
    *,
    bus: int,
    motor_id: int,
    cal_listen_s: float = 28.0,
    skip_iq_test: bool = False,
    strict_cali: bool = False,
) -> dict:
    """RS02 encoder calibration, wrapped for the dashboard.

    Blocking for ~15-35+ seconds. See module docstring for the real
    mechanical/electrical preconditions (shaft free, 24-60 V supply, motor
    at rest) — none of those are enforced here beyond passing through any
    VBUS warning ``calibrate()`` prints.

    Calls straight through to ``proxy.hub.debug.calibrate_robstride(...)``
    — that call already manages its own ``pause_plant_stream`` / ``lease``
    brackets internally; this function must not add another one.

    Returns
    -------
    dict with ``ok`` (the underlying bool result), ``bus``, ``motor_id``,
    ``warnings`` (any ``WARNING:`` lines the underlying call printed, e.g.
    VBUS out of range), and ``precondition_note`` (static reminder string).
    """
    with _capture_stdout() as tee:
        ok = proxy.hub.debug.calibrate_robstride(
            bus=bus,
            motor_id=motor_id,
            cal_listen_s=cal_listen_s,
            skip_iq_test=skip_iq_test,
            strict_cali=strict_cali,
        )
    return {
        "ok": bool(ok),
        "bus": int(bus),
        "motor_id": int(motor_id),
        "warnings": _extract_warnings(tee.text),
        "precondition_note": PRECONDITION_NOTE,
    }
