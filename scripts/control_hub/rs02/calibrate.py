"""
RS02 encoder calibration — datasheet sequence (RS02_Firmware_Documentation.pdf).

Prep:  comm 0x04 reset (fault clear) → parawrite 0x702D=1 (iq_test, §4.2.7)
Cal:    comm 0x05 motor_cali — one passive MCU listen (shaft must spin freely)
Post:   comm 0x06 zero → comm 0x16 data_save → pararead verify

Motor must stay at rest before 0x05 (no enable/ctrl). Valid supply: 24–60 V.

Re-probing the bus during cal (follow-up listen chunks, enable polls) can
disturb the motor and yield stale mms readbacks even when cal finished on-device.
Default: one 0x05 + passive listen, then zero/save if mms=cali was seen.
Use strict_cali=True to require mms→rest/running in the final readback.
"""
from __future__ import annotations

import time
from typing import Optional

from controls_pcb_host.actuator_config import format_table, host_table
from controls_pcb_host.protocol import (
    PROBE_CALI,
    PROBE_DATA_SAVE,
    PROBE_RESET,
    PROBE_ZERO,
)
from controls_pcb_host.protocol.can_bus import can_bus_label, is_mcp_bus, print_can_bus_note, probe_target_label
from controls_pcb_host.session import PcbSession

from ..link import heal_usb, rs2_bench
from ..protocol.rs02 import (
    DEFAULT_CAL_LISTEN_S,
    PARAM_BUS_VOLT,
    PARAM_IQ_TEST,
    PARAM_MECH_POS,
    PARAM_RUN_MODE,
    VBUS_MAX_V,
    VBUS_MIN_V,
    comm_label,
    decode_ext_id,
    mms_label,
)
from .display import format_probe_line
from .probe import (
    cali_finished,
    pararead,
    parawrite,
    probe,
    probe_cali,
    probe_response_valid,
)

VERIFY_READS = (
    ("mechPos (0x7019)", PARAM_MECH_POS),
    ("VBUS (0x701C)", PARAM_BUS_VOLT),
    ("run_mode (0x7005)", PARAM_RUN_MODE),
)

CAL_SETTLE_S = 1.0
_CAL_PROGRESS_INTERVAL_S = 2.0

# RS02 comm 0x02 velocity field (rad/s) — matches RS02_V_MIN/MAX in firmware.
_V_MIN = -44.0
_V_MAX = 44.0


def _comm02_velocity_rad_s(can_data: bytes) -> float:
    if len(can_data) < 4:
        return 0.0
    v_raw = (can_data[2] << 8) | can_data[3]
    return _V_MIN + (v_raw / 65535.0) * (_V_MAX - _V_MIN)


def _cali_progress_line(motor_id: int, parsed: dict, saw_cali: bool) -> None:
    if not parsed.get("ext_id"):
        return
    ext = decode_ext_id(parsed["ext_id"])
    if ext.mode != 0x02 or ext.motor_id != (motor_id & 0xFF):
        return
    flag = " *" if saw_cali else ""
    can_data = bytes(parsed.get("can_data") or b"")
    vel = _comm02_velocity_rad_s(can_data)
    raw_n = int(parsed.get("raw_frames", 0))
    print(
        f"  ... cali listen  mms={mms_label(ext.mode_status)}{flag}  "
        f"vel={vel:+.2f} rad/s  raw={raw_n}  data={can_data.hex()}"
    )


def run_encoder_cal(
    port: str,
    bus: int,
    motor_id: int,
    *,
    cal_listen_s: float = DEFAULT_CAL_LISTEN_S,
    skip_iq_test: bool = False,
    strict_cali: bool = False,
) -> int:
    motor_id &= 0xFF
    cal_listen_s = max(10.0, cal_listen_s)
    usb_wait_s = cal_listen_s + 15.0

    print(f"RS2 encoder cal on {port}  {probe_target_label(motor_id, bus)}")
    print_can_bus_note(bus)
    print(format_table())
    print("RS2 --bus selects plant_diag probe target only; other slots stay in plant_config.")
    print("Sequence per RS02 manual: reset → iq_test → 0x05 cali → 0x06 zero → 0x16 save.")
    print(
        "Cal: one comm 0x05 + passive CAN listen — no follow-up probes during cal "
        "(extra bus traffic can fault the drive or lie about mms)."
    )
    print(f"Supply: {VBUS_MIN_V:.0f}–{VBUS_MAX_V:.0f} V. Shaft free, no load on output.")
    print()

    with PcbSession(port) as session:
        with session.rx_pump():
            print("Healing USB link before calibrate...")
            if not heal_usb(session):
                print("ERROR: no USB feedback — power-cycle board and retry.")
                return 1

            with rs2_bench(session, bus):
                return _cal_body(
                    session,
                    bus,
                    motor_id,
                    cal_listen_s,
                    usb_wait_s,
                    skip_iq_test,
                    strict_cali,
                )


def _ensure_rest_before_cali(
    session: PcbSession,
    bus: int,
    motor_id: int,
    after_resp: Optional[dict],
) -> bool:
    """One reset if iq_test left mms=running; avoid probe spam before 0x05."""
    if after_resp and after_resp.get("ext_id") and probe_response_valid(after_resp, motor_id):
        if decode_ext_id(after_resp["ext_id"]).mode_status == 0:
            return True
        if decode_ext_id(after_resp["ext_id"]).mode_status == 2:
            print("  note: mms=running after iq_test — one reset before 0x05...")
    resp = probe(session, motor_id, PROBE_RESET, 0.55, bus=bus)
    if resp and resp.get("ext_id") and probe_response_valid(resp, motor_id):
        mms = decode_ext_id(resp["ext_id"]).mode_status
        if mms == 0:
            print("  mms=rest")
            return True
        print(f"  WARNING: mms={mms_label(mms)} after reset — 0x05 may not arm.")
    return False


def _cal_body(
    session: PcbSession,
    bus: int,
    motor_id: int,
    cal_listen_s: float,
    usb_wait_s: float,
    skip_iq_test: bool,
    strict_cali: bool,
) -> int:
    print("--- prep: reset comm 0x04 (fault clear, motor at rest) ---")
    resp = probe(session, motor_id, PROBE_RESET, 0.55, bus=bus)
    print(format_probe_line("reset comm 0x04", motor_id, resp))
    if resp and resp.get("ext_id"):
        ext = decode_ext_id(resp["ext_id"])
        if ext.mode_status == 2:
            print("  note: mms=running after reset — re-reset until rest before 0x05...")
            for _ in range(8):
                time.sleep(0.15)
                resp = probe(session, motor_id, PROBE_RESET, 0.45, bus=bus)
                if resp and resp.get("ext_id"):
                    if decode_ext_id(resp["ext_id"]).mode_status == 0:
                        print("  mms=rest")
                        break
    print()

    if not skip_iq_test:
        print("--- iq_test parawrite 0x702D=1 (§4.2.7 — extended init reference) ---")
        resp = parawrite(session, motor_id, PARAM_IQ_TEST, 1, 1.5, bus=bus)
        print(format_probe_line("iq_test 0x702D=1", motor_id, resp))
        _ensure_rest_before_cali(session, bus, motor_id, resp)
        print()

    hit, _ = pararead(session, motor_id, PARAM_BUS_VOLT, 0.45, bus=bus)
    if hit is not None:
        vbus = float(hit["position"])
        if vbus < VBUS_MIN_V or vbus > VBUS_MAX_V:
            print(f"  WARNING: VBUS={vbus:.1f} V outside {VBUS_MIN_V:.0f}–{VBUS_MAX_V:.0f} V range.")
        else:
            print(f"  VBUS={vbus:.1f} V (in range)")
        print()

    listen_param = int(cal_listen_s) & 0xFF
    print(
        f"--- motor_cali comm 0x05 (one {cal_listen_s:.0f}s passive listen — shaft must spin) ---"
    )
    if is_mcp_bus(bus):
        print("  MCP: firmware reset+drain immediately before comm 0x05 (after iq_test gap).")
        time.sleep(0.25)
    else:
        print("  FDCAN: reset+settle before comm 0x05 (required after MIT teleop).")
        time.sleep(0.25)
    last_progress = 0.0

    def on_progress(parsed: dict, saw: bool) -> None:
        nonlocal last_progress
        now = time.monotonic()
        if now - last_progress >= _CAL_PROGRESS_INTERVAL_S:
            last_progress = now
            _cali_progress_line(motor_id, parsed, saw)

    try:
        resp, saw_cali = probe_cali(
            session,
            motor_id,
            cal_listen_s,
            bus=bus,
            usb_wait_s=usb_wait_s,
            probe_param=listen_param,
            on_progress=on_progress,
        )
    except KeyboardInterrupt:
        print("\n  cali aborted (Ctrl+C)")
        return 130

    print(format_probe_line("motor_cali", motor_id, resp))

    cal_ok = resp is not None and cali_finished(resp, motor_id, saw_cali=saw_cali)
    if resp and resp.get("ext_id") and probe_response_valid(resp, motor_id):
        ext = decode_ext_id(resp["ext_id"])
        if ext.mode_status == 1 and not cal_ok:
            print(
                f"  final readback mms={mms_label(ext.mode_status)} after listen "
                f"(MCU may exit early when spin settles; zero/save still valid if saw_cali)."
            )
    elif resp and resp.get("ext_id"):
        ext = decode_ext_id(resp["ext_id"])
        print(f"         final mms={mms_label(ext.mode_status)}")
    print()

    if not saw_cali:
        print("  motor never entered mms=cali — 0x05 did not start encoder cal.")
        if resp is None:
            print(
                "  (no cali USB progress — reflash MCU with probe-progress TX fix, "
                "then retry; listen may still have run on-device.)"
            )
        if is_mcp_bus(bus):
            print("  MCP: confirm CH4 ACT LED blinks on 0x05 TX; try higher bus voltage if marginal.")
        print("  Check: shaft free, bus/ID, motor at rest, firmware PROBE_CALI (kind 16).")
        return 1

    if not cal_ok:
        if strict_cali:
            print("  SKIP zero/save — strict mode requires mms→rest/running in readback.")
            print("  Power-cycle motor if shaft still spinning; retry when at rest.")
            return 1
        print(
            f"  proceeding to zero/save ({CAL_SETTLE_S:.0f}s settle, then comm 0x06/0x16 ~10s) — "
            "mms=cali was seen during listen."
        )
        time.sleep(CAL_SETTLE_S)

    if resp is not None and not probe_response_valid(resp, motor_id) and strict_cali:
        print("  SKIP zero/save — last cal reply was bus noise (wrong motor id / comm).")
        return 1

    print("--- motor_zero comm 0x06 ---")
    resp = probe(session, motor_id, PROBE_ZERO, 4.0, bus=bus)
    print(format_probe_line("motor_zero", motor_id, resp))
    if resp is not None and not probe_response_valid(resp, motor_id):
        print("  SKIP data_save — motor_zero reply was bus noise (wrong id / comm).")
        return 1
    print()

    print("--- data_save comm 0x16 ---")
    resp = probe(session, motor_id, PROBE_DATA_SAVE, 5.0, bus=bus)
    print(format_probe_line("data_save", motor_id, resp))
    print()

    print("--- pararead verify ---")
    got_pararead = False
    mech_pos: Optional[float] = None
    for label, index in VERIFY_READS:
        hit, sniff = pararead(session, motor_id, index, 0.55, bus=bus)
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
        idx_echo = hit["can_data"][0] | (hit["can_data"][1] << 8)
        print(
            f"  {label}: {comm_label(hit['comm_mode'])}  "
            f"index_echo=0x{idx_echo:04X}  float={hit['position']:+.4f}  "
            f"raw={hit['can_data'].hex()}"
        )
    print()

    if got_pararead and mech_pos is not None and abs(mech_pos) < 0.1:
        print("Result: cal sequence OK — mechPos near zero.")
        return 0
    if got_pararead:
        print(f"Result: pararead OK but mechPos={mech_pos:+.4f} rad — retry zero/save or cal.")
        return 1
    print("Result: zero/save ran but pararead verify failed.")
    return 1
