"""RS02 encoder calibration over DEBUG frames (DBGC/DBGF).

Sequence (RS02_Firmware_Documentation.pdf):
  reset (0x04) → iq_test parawrite 0x702D=1 → cali (0x05) + passive listen →
  zero (0x06) → data_save (0x16) → pararead verify.

Ported from scripts/legacy/control_hub/rs02/calibrate.py — no legacy imports.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, List, Optional, Tuple

from deft_controls_sdk.link.exchange.bench import (
    PROBE_CALI,
    PROBE_DATA_SAVE,
    PROBE_PARAREAD,
    PROBE_PARAWRITE,
    PROBE_RESET,
    PROBE_ZERO,
    build_rs2_probe_command,
    can_bus_label,
    parse_probe_pdu,
)

from .lease import lease
from .robstride import format_probe_line

if TYPE_CHECKING:
    from deft_controls_sdk.link import Connection
    from deft_controls_sdk.telemetry import TelemetryCache

PARAM_RUN_MODE = 0x7005
PARAM_MECH_POS = 0x7019
PARAM_BUS_VOLT = 0x701C
PARAM_IQ_TEST = 0x702D

VBUS_MIN_V = 24.0
VBUS_MAX_V = 60.0
DEFAULT_CAL_LISTEN_S = 28.0
CAL_SETTLE_S = 1.0
_CAL_PROGRESS_INTERVAL_S = 2.0
_V_MIN = -44.0
_V_MAX = 44.0

_VALID_COMM = (0x02, 0x11, 0x12, 0x18)
_MMS_LABELS = ("rest", "cali", "running")
_COMM_NAMES = {
    0x02: "motor_feedback",
    0x04: "motor_reset",
    0x05: "motor_cali",
    0x06: "motor_zero",
    0x11: "pararead",
    0x12: "parawrite",
    0x16: "data_save",
}

VERIFY_READS = (
    ("mechPos (0x7019)", PARAM_MECH_POS),
    ("VBUS (0x701C)", PARAM_BUS_VOLT),
    ("run_mode (0x7005)", PARAM_RUN_MODE),
)


@dataclass(frozen=True)
class ExtIdInfo:
    mode: int
    motor_id: int
    mode_status: int


def decode_ext_id(ext_id: int) -> ExtIdInfo:
    data16 = (ext_id >> 8) & 0xFFFF
    return ExtIdInfo(
        mode=(ext_id >> 24) & 0x1F,
        motor_id=data16 & 0xFF,
        mode_status=(data16 >> 14) & 0x3,
    )


def mms_label(mode_status: int) -> str:
    if mode_status < len(_MMS_LABELS):
        return _MMS_LABELS[mode_status]
    return f"mode{mode_status}"


def comm_label(comm_mode: int) -> str:
    name = _COMM_NAMES.get(comm_mode)
    if name is None:
        return f"0x{comm_mode:02X}"
    return f"0x{comm_mode:02X} {name}"


def probe_response_valid(resp: dict, motor_id: int) -> bool:
    if not resp.get("ext_id"):
        return False
    ext = decode_ext_id(int(resp["ext_id"]))
    disc = resp.get("discovered_id") or ext.motor_id
    if disc != (motor_id & 0xFF):
        return False
    comm = resp["comm_mode"]
    if comm != ext.mode or comm not in _VALID_COMM:
        return False
    return True


def cali_finished(resp: dict, motor_id: int, *, saw_cali: bool) -> bool:
    if not saw_cali or not probe_response_valid(resp, motor_id):
        return False
    return decode_ext_id(int(resp["ext_id"])).mode_status in (0, 2)


def _comm02_velocity_rad_s(can_data: bytes) -> float:
    if len(can_data) < 4:
        return 0.0
    v_raw = (can_data[2] << 8) | can_data[3]
    return _V_MIN + (v_raw / 65535.0) * (_V_MAX - _V_MIN)


def _send(
    connection: "Connection",
    motor_id: int,
    probe_kind: int,
    timeout_s: float,
    *,
    bus: int,
    param_index: int = 0,
    param_raw_value: int = 0,
) -> Optional[dict]:
    connection.reader.drain()
    frame = build_rs2_probe_command(
        motor_id,
        probe_kind,
        connection.next_seq(),
        param_index,
        param_raw_value,
        bus=bus,
    )
    return connection.exchange_raw(
        frame,
        parse_probe_pdu,
        timeout_s=timeout_s,
        predicate=lambda p: p["probe_id"] == (motor_id & 0xFF)
        and p["probe_kind"] == probe_kind,
    )


def _wait_cali(
    connection: "Connection",
    motor_id: int,
    timeout_s: float,
    *,
    on_progress: Optional[Callable[[dict, bool], None]] = None,
) -> Tuple[Optional[dict], bool]:
    """Wait for PROBE_CALI progress/final PDUs.

    Datasheet done signal is mms cali→rest/running in ext_id (no separate ACK).
    Firmware also enable-probes after saw_cali and returns early — we exit as soon
    as progress shows that transition (or a finished probe reply).
    """
    deadline = time.monotonic() + timeout_s
    latest_valid: Optional[dict] = None
    saw_cali = False
    while time.monotonic() < deadline:
        frame = connection.reader.pop()
        while frame is not None:
            parsed = parse_probe_pdu(frame)
            if (
                parsed is not None
                and parsed["probe_id"] == (motor_id & 0xFF)
                and parsed["probe_kind"] == PROBE_CALI
            ):
                if parsed.get("ext_id"):
                    ext = decode_ext_id(int(parsed["ext_id"]))
                    if ext.mode == 0x02 and ext.motor_id == (motor_id & 0xFF):
                        if ext.mode_status == 1:
                            saw_cali = True
                        if probe_response_valid(parsed, motor_id):
                            latest_valid = parsed
                        if cali_finished(parsed, motor_id, saw_cali=saw_cali):
                            return parsed, True
                if on_progress is not None:
                    on_progress(parsed, saw_cali)
            frame = connection.reader.pop()
        time.sleep(0.005)
    return latest_valid, saw_cali


def _probe_cali(
    connection: "Connection",
    motor_id: int,
    listen_s: float,
    *,
    bus: int,
    usb_wait_s: float,
    on_progress: Optional[Callable[[dict, bool], None]] = None,
) -> Tuple[Optional[dict], bool]:
    listen_s = max(8.0, listen_s)
    param = int(listen_s) & 0xFF
    connection.reader.drain()
    frame = build_rs2_probe_command(
        motor_id, PROBE_CALI, connection.next_seq(), param, bus=bus
    )
    connection.write_raw(frame, drain=True)
    return _wait_cali(connection, motor_id, usb_wait_s, on_progress=on_progress)


def pararead_index_echo(resp: dict) -> int:
    data = resp.get("can_data") or b""
    if len(data) < 2:
        return -1
    return int(data[0]) | (int(data[1]) << 8)


def pararead_is_hit(resp: dict, param_index: int) -> bool:
    """True when PDU is a usable pararead/parawrite reply for ``param_index``.

    RS02 does not echo the param index in data[0:1] (always 0x0000); the float
    lives at data[4:7]. Accept echo==0 or echo==index — same rule as
    ``scripts/legacy/rs02_can_scan.py``.
    """
    if resp.get("comm_mode") not in (0x11, 0x12):
        return False
    if not resp.get("found") and int(resp.get("raw_frames", 0) or 0) == 0:
        return False
    echo = pararead_index_echo(resp)
    return echo == 0 or echo == (param_index & 0xFFFF)


def _pararead(
    connection: "Connection",
    motor_id: int,
    index: int,
    timeout_s: float,
    *,
    bus: int,
) -> Tuple[Optional[dict], Optional[dict]]:
    connection.reader.drain()
    frame = build_rs2_probe_command(
        motor_id, PROBE_PARAREAD, connection.next_seq(), index, bus=bus
    )
    connection.write_raw(frame, drain=True)
    deadline = time.monotonic() + timeout_s
    hit: Optional[dict] = None
    sniff: Optional[dict] = None
    while time.monotonic() < deadline:
        raw = connection.reader.pop()
        while raw is not None:
            parsed = parse_probe_pdu(raw)
            if parsed is not None and parsed["probe_id"] == (motor_id & 0xFF):
                if pararead_is_hit(parsed, index):
                    return parsed, sniff
                if sniff is None:
                    sniff = parsed
            raw = connection.reader.pop()
        time.sleep(0.005)
    return hit, sniff


def _parawrite(
    connection: "Connection",
    motor_id: int,
    index: int,
    raw_value: int,
    timeout_s: float,
    *,
    bus: int,
) -> Optional[dict]:
    return _send(
        connection,
        motor_id,
        PROBE_PARAWRITE,
        timeout_s,
        bus=bus,
        param_index=index,
        param_raw_value=raw_value,
    )


def _cali_progress_line(motor_id: int, parsed: dict, saw_cali: bool) -> None:
    if not parsed.get("ext_id"):
        return
    ext = decode_ext_id(int(parsed["ext_id"]))
    if ext.mode != 0x02 or ext.motor_id != (motor_id & 0xFF):
        return
    flag = " *" if saw_cali else ""
    can_data = bytes(parsed.get("can_data") or b"")
    vel = _comm02_velocity_rad_s(can_data)
    print(
        f"  ... cali listen  mms={mms_label(ext.mode_status)}{flag}  "
        f"vel={vel:+.2f} rad/s  raw={int(parsed.get('raw_frames', 0))}  "
        f"data={can_data.hex()}"
    )


def _ensure_rest_before_cali(
    connection: "Connection",
    bus: int,
    motor_id: int,
    after_resp: Optional[dict],
) -> None:
    if after_resp and after_resp.get("ext_id") and probe_response_valid(after_resp, motor_id):
        mms = decode_ext_id(int(after_resp["ext_id"])).mode_status
        if mms == 0:
            return
        if mms == 2:
            print("  note: mms=running after iq_test — one reset before 0x05...")
    resp = _send(connection, motor_id, PROBE_RESET, 0.55, bus=bus)
    if resp and resp.get("ext_id") and probe_response_valid(resp, motor_id):
        mms = decode_ext_id(int(resp["ext_id"])).mode_status
        if mms == 0:
            print("  mms=rest")
        else:
            print(f"  WARNING: mms={mms_label(mms)} after reset — 0x05 may not arm.")


def calibrate(
    connection: "Connection",
    telemetry: Optional["TelemetryCache"],
    *,
    bus: int,
    motor_id: int,
    cal_listen_s: float = DEFAULT_CAL_LISTEN_S,
    skip_iq_test: bool = False,
    strict_cali: bool = False,
) -> bool:
    """Run encoder cal. Returns True on verify OK (mechPos near zero)."""
    motor_id &= 0xFF
    cal_listen_s = max(10.0, float(cal_listen_s))
    # Firmware packs listen seconds in one PDU byte (see robstride.c PROBE_CALI).
    if cal_listen_s > 255.0:
        print(
            f"  WARNING: --cal-listen-s={cal_listen_s:.0f} > 255; "
            f"MCU will use {int(cal_listen_s) & 0xFF}s (host still waits full pad)."
        )
    # MCU exits early (mms leave cali / enable ACK); keep a small USB pad only.
    usb_wait_s = cal_listen_s + 5.0

    print(f"RS2 encoder cal on {can_bus_label(bus)}  id=0x{motor_id:02X}")
    print("Sequence: reset → iq_test → 0x05 cali → 0x06 zero → 0x16 save.")
    print(
        f"Supply: {VBUS_MIN_V:.0f}–{VBUS_MAX_V:.0f} V. Shaft free, no load on output."
    )
    print()

    with lease(connection, telemetry, bus=bus):
        if telemetry is not None:
            telemetry.set_connected(True, mode="calibrate")
        return _cal_body(
            connection,
            bus,
            motor_id,
            cal_listen_s,
            usb_wait_s,
            skip_iq_test,
            strict_cali,
        )


def _cal_body(
    connection: "Connection",
    bus: int,
    motor_id: int,
    cal_listen_s: float,
    usb_wait_s: float,
    skip_iq_test: bool,
    strict_cali: bool,
) -> bool:
    print("--- prep: reset comm 0x04 ---")
    resp = _send(connection, motor_id, PROBE_RESET, 0.55, bus=bus)
    print(f"  reset  {format_probe_line(resp) if resp else 'no reply'}")
    if resp and resp.get("ext_id"):
        if decode_ext_id(int(resp["ext_id"])).mode_status == 2:
            print("  note: mms=running after reset — re-reset until rest...")
            for _ in range(8):
                time.sleep(0.15)
                resp = _send(connection, motor_id, PROBE_RESET, 0.45, bus=bus)
                if resp and resp.get("ext_id"):
                    if decode_ext_id(int(resp["ext_id"])).mode_status == 0:
                        print("  mms=rest")
                        break
    print()

    if not skip_iq_test:
        print("--- iq_test parawrite 0x702D=1 ---")
        resp = _parawrite(connection, motor_id, PARAM_IQ_TEST, 1, 1.5, bus=bus)
        print(f"  iq_test  {format_probe_line(resp) if resp else 'no reply'}")
        _ensure_rest_before_cali(connection, bus, motor_id, resp)
        print()

    hit, _ = _pararead(connection, motor_id, PARAM_BUS_VOLT, 0.45, bus=bus)
    if hit is not None:
        vbus = float(hit["position"])
        if vbus < VBUS_MIN_V or vbus > VBUS_MAX_V:
            print(f"  WARNING: VBUS={vbus:.1f} V outside {VBUS_MIN_V:.0f}–{VBUS_MAX_V:.0f} V")
        else:
            print(f"  VBUS={vbus:.1f} V (in range)")
        print()

    print(f"--- motor_cali 0x05 ({cal_listen_s:.0f}s passive listen — shaft must spin) ---")
    time.sleep(0.25)
    last_progress = 0.0

    def on_progress(parsed: dict, saw: bool) -> None:
        nonlocal last_progress
        now = time.monotonic()
        if now - last_progress >= _CAL_PROGRESS_INTERVAL_S:
            last_progress = now
            _cali_progress_line(motor_id, parsed, saw)

    try:
        resp, saw_cali = _probe_cali(
            connection,
            motor_id,
            cal_listen_s,
            bus=bus,
            usb_wait_s=usb_wait_s,
            on_progress=on_progress,
        )
    except KeyboardInterrupt:
        print("\n  cali aborted (Ctrl+C)")
        return False

    print(f"  cali  {format_probe_line(resp) if resp else 'no reply'}")
    cal_ok = resp is not None and cali_finished(resp, motor_id, saw_cali=saw_cali)
    print()

    if not saw_cali:
        print("  motor never entered mms=cali — 0x05 did not start encoder cal.")
        print("  Check: shaft free, bus/ID, motor at rest, firmware PROBE_CALI.")
        return False

    if not cal_ok:
        if strict_cali:
            print("  SKIP zero/save — strict mode requires mms→rest/running.")
            return False
        print(f"  proceeding to zero/save ({CAL_SETTLE_S:.0f}s settle) — mms=cali was seen.")
        time.sleep(CAL_SETTLE_S)

    print("--- motor_zero 0x06 ---")
    resp = _send(connection, motor_id, PROBE_ZERO, 4.0, bus=bus)
    print(f"  zero  {format_probe_line(resp) if resp else 'no reply'}")
    if resp is not None and not probe_response_valid(resp, motor_id):
        print("  SKIP data_save — motor_zero reply was bus noise.")
        return False
    print()

    print("--- data_save 0x16 ---")
    resp = _send(connection, motor_id, PROBE_DATA_SAVE, 5.0, bus=bus)
    print(f"  save  {format_probe_line(resp) if resp else 'no reply'}")
    print()

    print("--- pararead verify ---")
    got_pararead = False
    mech_pos: Optional[float] = None
    for label, index in VERIFY_READS:
        hit, sniff = _pararead(connection, motor_id, index, 0.55, bus=bus)
        time.sleep(0.12)
        if hit is None:
            line = f"  {label}: no pararead reply"
            if sniff is not None:
                line += f" (saw {comm_label(sniff['comm_mode'])})"
            print(line)
            continue
        got_pararead = True
        if index == PARAM_MECH_POS:
            mech_pos = float(hit["position"])
        idx_echo = pararead_index_echo(hit)
        echo_note = "(no echo — RS02 normal)" if idx_echo == 0 else f"echo=0x{idx_echo:04X}"
        print(
            f"  {label}: {comm_label(hit['comm_mode'])}  "
            f"{echo_note}  float={hit['position']:+.4f}"
        )
    print()

    if got_pararead and mech_pos is not None and abs(mech_pos) < 0.1:
        print("Result: cal sequence OK — mechPos near zero.")
        return True
    if got_pararead:
        print(f"Result: pararead OK but mechPos={mech_pos:+.4f} rad — retry zero/save or cal.")
        return False
    print("Result: zero/save ran but pararead verify failed.")
    return False
