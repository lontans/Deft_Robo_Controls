"""Unit tests for ``deft_controls_sdk.debug.panels.discover.calibrate_panel``.

No hardware / no real COM port. Two layers:

1. Mocked ``proxy.hub.debug`` — proves the panel calls through with the
   right kwargs and shapes the return dict (``ok``/``bus``/``motor_id``/
   ``warnings``/``precondition_note``), including recovering printed
   ``WARNING:`` lines.
2. A real ``DebugAPI`` backed by a scripted fake ``Connection`` (same
   pattern as ``test_deft_controls_sdk_robstride_calibrate.py``) driving the
   *actual* ``robstride_calibrate.calibrate()`` body end-to-end through the
   panel, with ``lease``/``pause_plant_stream`` wrapped in call-counters —
   this is the test that proves the panel does NOT double-wrap either
   context manager around the already-self-wrapping ``calibrate()`` call.
"""
from __future__ import annotations

import ast
import contextlib
import inspect
import os
import struct
import sys
from typing import Optional

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import pytest

from deft_controls_sdk.debug import DebugAPI
from deft_controls_sdk.debug.panels.discover import calibrate_panel
from deft_controls_sdk.debug.panels.discover.calibrate_panel import (
    calibrate_robstride_panel,
)
from deft_controls_sdk.link import Connection
from deft_controls_sdk.link.exchange import (
    HOST_DEBUG_COMMAND_MAGIC,
    HOST_DEBUG_FEEDBACK_MAGIC,
    HOST_LAYOUT_VERSION,
    IMAGE_BYTES,
    PDU_OFF,
    RS2_RESP_TAG,
    SESSION_BEGIN,
    SESSION_END,
    extract_rs2_mailbox,
)
from deft_controls_sdk.link.exchange.bench import (
    PROBE_CALI,
    PROBE_DATA_SAVE,
    PROBE_PARAREAD,
    PROBE_PARAWRITE,
    PROBE_RESET,
    PROBE_ZERO,
)
from deft_controls_sdk.link.exchange.wire_layout import STM32_MODE_DEBUG


# -- Layer 1: mocked DebugAPI ------------------------------------------------------


class _FakeDebugAPI:
    def __init__(self, *, result=True, printed: str = "", raise_exc: Exception | None = None):
        self._result = result
        self._printed = printed
        self._raise_exc = raise_exc
        self.calls: list[dict] = []

    def calibrate_robstride(
        self, *, bus, motor_id, cal_listen_s=28.0, skip_iq_test=False, strict_cali=False
    ):
        self.calls.append(
            {
                "bus": bus,
                "motor_id": motor_id,
                "cal_listen_s": cal_listen_s,
                "skip_iq_test": skip_iq_test,
                "strict_cali": strict_cali,
            }
        )
        if self._printed:
            print(self._printed)
        if self._raise_exc is not None:
            raise self._raise_exc
        return self._result


class _FakeHub:
    def __init__(self, debug_api):
        self.debug = debug_api


class _FakeProxy:
    def __init__(self, debug_api):
        self.hub = _FakeHub(debug_api)


def test_calibrate_panel_forwards_kwargs():
    api = _FakeDebugAPI(result=True)
    proxy = _FakeProxy(api)

    calibrate_robstride_panel(
        proxy,
        bus=4,
        motor_id=0x70,
        cal_listen_s=12.0,
        skip_iq_test=True,
        strict_cali=True,
    )

    assert api.calls == [
        {
            "bus": 4,
            "motor_id": 0x70,
            "cal_listen_s": 12.0,
            "skip_iq_test": True,
            "strict_cali": True,
        }
    ]


def test_calibrate_panel_default_kwargs():
    api = _FakeDebugAPI(result=True)
    proxy = _FakeProxy(api)
    calibrate_robstride_panel(proxy, bus=1, motor_id=0x40)
    call = api.calls[0]
    assert call["cal_listen_s"] == 28.0
    assert call["skip_iq_test"] is False
    assert call["strict_cali"] is False


def test_calibrate_panel_return_shape_on_success():
    api = _FakeDebugAPI(result=True)
    proxy = _FakeProxy(api)
    out = calibrate_robstride_panel(proxy, bus=4, motor_id=0x70)
    assert out == {
        "ok": True,
        "bus": 4,
        "motor_id": 0x70,
        "warnings": [],
        "precondition_note": calibrate_panel.PRECONDITION_NOTE,
    }


def test_calibrate_panel_return_shape_on_failure():
    api = _FakeDebugAPI(result=False)
    proxy = _FakeProxy(api)
    out = calibrate_robstride_panel(proxy, bus=4, motor_id=0x70)
    assert out["ok"] is False
    assert out["bus"] == 4
    assert out["motor_id"] == 0x70


def test_calibrate_panel_precondition_note_mentions_shaft_and_supply():
    """Precondition note must actually say something actionable, not just exist."""
    note = calibrate_panel.PRECONDITION_NOTE.lower()
    assert "shaft" in note
    assert "24" in note and "60" in note  # VBUS range


def test_calibrate_panel_docstring_mentions_preconditions():
    """Spec requires the precondition reminder to reach anyone reading the
    panel's source, i.e. live in the function's own docstring too."""
    doc = (calibrate_robstride_panel.__doc__ or "").lower()
    assert "shaft" in doc or "precondition" in doc


def test_calibrate_panel_captures_printed_warning_lines():
    api = _FakeDebugAPI(
        result=False,
        printed="  WARNING: VBUS=18.0 V outside 24-60 V",
    )
    proxy = _FakeProxy(api)
    out = calibrate_robstride_panel(proxy, bus=4, motor_id=0x70)
    assert any("VBUS" in w for w in out["warnings"])
    assert any("WARNING" in w for w in out["warnings"])


def test_calibrate_panel_no_warnings_key_noise_when_nothing_printed():
    api = _FakeDebugAPI(result=True, printed="  mms=rest\n  VBUS=48.0 V (in range)")
    proxy = _FakeProxy(api)
    out = calibrate_robstride_panel(proxy, bus=4, motor_id=0x70)
    assert out["warnings"] == []


def test_calibrate_panel_still_prints_to_real_stdout(capsys):
    """Capturing for `warnings` must not swallow the live console output a
    notebook/CLI caller expects to see during a 15-35s blocking call."""
    api = _FakeDebugAPI(result=True, printed="  WARNING: VBUS=70.0 V outside 24-60 V")
    proxy = _FakeProxy(api)
    calibrate_robstride_panel(proxy, bus=4, motor_id=0x70)
    captured = capsys.readouterr()
    assert "WARNING" in captured.out


def test_calibrate_panel_propagates_underlying_errors():
    api = _FakeDebugAPI(raise_exc=RuntimeError("hub.debug.calibrate_robstride needs mode='debug'"))
    proxy = _FakeProxy(api)
    with pytest.raises(RuntimeError, match="mode='debug'"):
        calibrate_robstride_panel(proxy, bus=1, motor_id=0x40)


def test_calibrate_panel_is_robstride_only_no_protocol_kwarg():
    """The shared signature has no protocol switch — confirms this panel
    doesn't pretend to support damiao/cubemars/zeroerr calibrate."""
    sig = inspect.signature(calibrate_robstride_panel)
    assert "protocol" not in sig.parameters
    assert list(sig.parameters) == [
        "proxy",
        "bus",
        "motor_id",
        "cal_listen_s",
        "skip_iq_test",
        "strict_cali",
    ]


# -- static guard: no lease/pause_plant_stream wrapper in the panel's own source --


def test_calibrate_panel_source_has_no_extra_lease_or_pause_wrapper():
    """Structural guard alongside the call-count integration test below:
    the panel module's source must not itself open a `with lease(...)` or
    `with pause_plant_stream(...)` block — that responsibility belongs
    entirely to the underlying `calibrate()`.

    AST-based (not substring) so the module docstring is free to *mention*
    `lease(...)`/`pause_plant_stream(...)` in prose (as it does, explaining
    why the panel must not add another one) without tripping this guard.
    """
    src = inspect.getsource(calibrate_panel)
    tree = ast.parse(src)

    def _called_name(expr: ast.expr) -> Optional[str]:
        if isinstance(expr, ast.Call) and isinstance(expr.func, ast.Name):
            return expr.func.id
        return None

    with_context_names = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.With, ast.AsyncWith)):
            for item in node.items:
                name = _called_name(item.context_expr)
                if name:
                    with_context_names.add(name)

    assert "lease" not in with_context_names
    assert "pause_plant_stream" not in with_context_names
    # And confirm the module doesn't even import those names — it has no
    # business touching them at all, only `calibrate()` does.
    imported_names = {
        alias.asname or alias.name
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom)
        for alias in node.names
    }
    assert "lease" not in imported_names
    assert "pause_plant_stream" not in imported_names


# -- Layer 2: real DebugAPI + fake Connection, call-count proof --------------------


def _fake_connection(responder) -> Connection:
    """Stubbed Connection: write_raw -> responder -> FrameReader (no real COM)."""
    conn = Connection("TESTPORT", stm32_mode=STM32_MODE_DEBUG)
    conn._ser = object()

    def _write_raw(frame: bytes, *, drain: bool = False) -> None:
        reply = responder(frame)
        if reply is not None:
            conn._reader.feed(reply)

    conn.write_raw = _write_raw  # type: ignore[method-assign]
    return conn


def _rs2_feedback(
    *,
    motor_id: int,
    probe_kind: int,
    comm_mode: int = 0x02,
    found: bool = True,
    mms: int = 0,
    position: float = 0.0,
    can_data: bytes = b"\x00" * 8,
    raw_frames: int = 1,
    discovered_id: Optional[int] = None,
) -> bytes:
    buf = bytearray(IMAGE_BYTES)
    struct.pack_into(
        "<IHHI", buf, 0, HOST_DEBUG_FEEDBACK_MAGIC, HOST_LAYOUT_VERSION, IMAGE_BYTES, 0
    )
    pdu = bytearray(32)
    pdu[0] = RS2_RESP_TAG
    pdu[1] = motor_id & 0xFF
    pdu[2] = 1 if found else 0
    pdu[3] = comm_mode & 0xFF
    data16 = (motor_id & 0xFF) | ((mms & 0x3) << 14)
    ext_id = ((comm_mode & 0x1F) << 24) | (data16 << 8)
    struct.pack_into("<I", pdu, 4, ext_id)
    pad = bytes(can_data[:8]).ljust(8, b"\x00")
    pdu[8:16] = pad
    struct.pack_into("<ff", pdu, 16, 25.0, float(position))
    pdu[24] = (discovered_id if discovered_id is not None else motor_id) & 0xFF
    pdu[25] = probe_kind & 0xFF
    pdu[26] = raw_frames & 0xFF
    buf[PDU_OFF : PDU_OFF + 32] = pdu
    return bytes(buf)


def _pararead_can_data_zero_echo(value: float) -> bytes:
    can = bytearray(8)
    struct.pack_into("<f", can, 4, float(value))
    return bytes(can)


def _counting_cm(original):
    """Wrap a ``@contextmanager`` factory so entries are counted."""
    calls = {"n": 0}

    @contextlib.contextmanager
    def wrapper(*args, **kwargs):
        calls["n"] += 1
        with original(*args, **kwargs):
            yield

    wrapper.calls = calls  # type: ignore[attr-defined]
    return wrapper


def _make_successful_responder(motor_id: int):
    from deft_controls_sdk.debug.robstride_calibrate import (
        PARAM_BUS_VOLT,
        PARAM_MECH_POS,
        PARAM_RUN_MODE,
    )

    def _ext_feedback(kind: int, *, mms: int = 0, comm: int = 0x02) -> bytes:
        return _rs2_feedback(
            motor_id=motor_id, probe_kind=kind, comm_mode=comm, mms=mms,
            position=0.0, raw_frames=1,
        )

    def _zero_echo_pararead(param_index: int) -> bytes:
        values = {PARAM_MECH_POS: 0.01, PARAM_BUS_VOLT: 48.0, PARAM_RUN_MODE: 0.0}
        value = float(values.get(param_index & 0xFFFF, 0.0))
        return _rs2_feedback(
            motor_id=motor_id, probe_kind=PROBE_PARAREAD, comm_mode=0x11,
            position=value, can_data=_pararead_can_data_zero_echo(value), raw_frames=1,
        )

    def responder(frame: bytes):
        pdu = extract_rs2_mailbox(frame)
        kind = pdu[4]
        param_index = pdu[5] | (pdu[6] << 8)

        if kind == SESSION_BEGIN:
            return _rs2_feedback(motor_id=0, probe_kind=SESSION_BEGIN, found=False, raw_frames=0)
        if kind == SESSION_END:
            return _rs2_feedback(motor_id=0, probe_kind=SESSION_END, found=False, raw_frames=0)
        if kind == PROBE_RESET:
            return _ext_feedback(PROBE_RESET, mms=0)
        if kind == PROBE_PARAWRITE:
            return _ext_feedback(PROBE_PARAWRITE, mms=0, comm=0x12)
        if kind == PROBE_PARAREAD:
            return _zero_echo_pararead(param_index)
        if kind == PROBE_CALI:
            return _ext_feedback(PROBE_CALI, mms=1) + _ext_feedback(PROBE_CALI, mms=0)
        if kind == PROBE_ZERO:
            return _ext_feedback(PROBE_ZERO, mms=0)
        if kind == PROBE_DATA_SAVE:
            return _ext_feedback(PROBE_DATA_SAVE, mms=0)
        return None

    return responder


def test_calibrate_panel_does_not_double_wrap_lease_or_pause_stream(monkeypatch):
    """End-to-end through the panel with the REAL ``calibrate()`` body (fake
    wire only) — proves ``lease`` and ``pause_plant_stream`` are each
    entered exactly once, i.e. the panel adds no extra wrapper of its own
    around the primitive, which already brackets itself with both."""
    monkeypatch.setattr("deft_controls_sdk.debug.robstride_calibrate.time.sleep", lambda *_a, **_k: None)
    monkeypatch.setattr("deft_controls_sdk.link.connection.time.sleep", lambda *_a, **_k: None)

    from deft_controls_sdk.debug.lease import lease as real_lease
    from deft_controls_sdk.debug.stream_pause import pause_plant_stream as real_pause

    wrapped_lease = _counting_cm(real_lease)
    wrapped_pause = _counting_cm(real_pause)

    # `lease` is bound into robstride_calibrate's module namespace at import
    # time (`from .lease import lease`) — patch it there.
    monkeypatch.setattr("deft_controls_sdk.debug.robstride_calibrate.lease", wrapped_lease)
    # `pause_plant_stream` is imported locally inside calibrate() from the
    # stream_pause module at call time — patch the module attribute.
    monkeypatch.setattr("deft_controls_sdk.debug.stream_pause.pause_plant_stream", wrapped_pause)

    motor_id = 0x70
    bus = 4
    conn = _fake_connection(_make_successful_responder(motor_id))
    debug_api = DebugAPI(conn, None)
    proxy = _FakeProxy(debug_api)

    out = calibrate_robstride_panel(
        proxy, bus=bus, motor_id=motor_id, cal_listen_s=10.0,
        skip_iq_test=False, strict_cali=False,
    )

    assert out["ok"] is True
    assert wrapped_lease.calls["n"] == 1, "lease must be entered exactly once"
    assert wrapped_pause.calls["n"] == 1, "pause_plant_stream must be entered exactly once"
