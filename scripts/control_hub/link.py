"""USB link heal and plant-runtime / bench handoff."""
from __future__ import annotations

import time
from contextlib import contextmanager
from typing import Iterator, Optional

from controls_pcb_host.feedback import parse_feedback_header
from controls_pcb_host.feedback import parse_actuator_feedback
from controls_pcb_host.actuator_config import slot_config
from controls_pcb_host.protocol.can_bus import can_bus_label, is_rs02_plant_bus, normalize_can_bus
from controls_pcb_host.session import PcbSession

# Bench reply tags in feedback pdu[0] — runtime should see actuator slots, not these.
BENCH_PDU_TAGS = frozenset({"r", "m", "d"})

# plant_block_reason_t values that block 500 Hz apply (host_stale=5 is OK during prep).
BLOCKING_PLANT_BLOCKS = frozenset({1, 2, 3, 4, 6})


class PlantRuntimeError(RuntimeError):
    """Plant 500 Hz path not ready (bench gates, USB, or wrong bus)."""


def heal_usb(session: PcbSession, *, rounds: int = 20, drain_first: bool = False) -> bool:
    if drain_first:
        session.reader.drain()
    for _ in range(rounds):
        session.send_plant()
        time.sleep(0.04)
        frame = session.reader.pop()
        while frame is not None:
            if parse_feedback_header(frame) is not None:
                print("  USB link OK (562 B plant feedback).")
                return True
            frame = session.reader.pop()
    return False


def release_stuck(session: PcbSession, *, rounds: int = 16) -> None:
    """Neutral plant stream — firmware plant_diag_release_actuator_can() on each RX command."""
    for _ in range(rounds):
        session.send_plant()
        time.sleep(0.04)
        frame = session.reader.pop()
        while frame is not None:
            frame = session.reader.pop()


def release_bench_gates(session: PcbSession, *, bus: Optional[int] = None) -> None:
    """RECOVERY mount + plant stream. No RS2/DM SESSION_END (avoids motor reset / pdu='m')."""
    _ = bus
    session.recover()
    time.sleep(0.1)
    release_stuck(session, rounds=20)


def ensure_plant_runtime(
    session: PcbSession,
    *,
    label: str = "plant runtime",
    bus: Optional[int] = None,
    max_attempts: int = 8,
) -> dict:
    """
    Prep plant teleop: confirm USB feedback and plant_block not bench-gated.

    Uses poll_status (same pattern as link-test). Does not RS2 SESSION_END.
    """
    _ = bus
    time.sleep(0.12)

    if not heal_usb(session, rounds=12):
        release_bench_gates(session)
        if not heal_usb(session, rounds=16):
            raise PlantRuntimeError(f"{label}: no USB feedback — power-cycle board and retry.")

    hdr: Optional[dict] = None
    for attempt in range(max_attempts):
        if attempt == 1:
            release_bench_gates(session)

        hdr = session.poll_status(timeout_s=1.5)
        if hdr is None:
            print(f"  no feedback header (attempt {attempt + 1}/{max_attempts})...")
            time.sleep(0.15)
            continue

        block = int(hdr.get("plant_block", 0))
        tag = str(hdr.get("pdu_tag", ""))

        if block == 3:
            print(f"  post-bench quiet_period (attempt {attempt + 1}/{max_attempts})...")
            time.sleep(0.45)
            continue

        if tag in BENCH_PDU_TAGS:
            print(f"  clearing bench pdu={tag!r} (attempt {attempt + 1}/{max_attempts})...")
            release_stuck(session, rounds=24)
            continue

        if block in BLOCKING_PLANT_BLOCKS:
            print(
                f"  clearing plant_block={hdr.get('plant_block_name')} "
                f"(attempt {attempt + 1}/{max_attempts})..."
            )
            release_bench_gates(session)
            continue

        if block == 5:
            release_stuck(session, rounds=16)
            hdr = session.poll_status(timeout_s=1.0)
            if hdr is None:
                continue
            block = int(hdr.get("plant_block", 0))
            if block == 5:
                print("  note: plant_block=host_stale — keep streaming...")
                continue

        block = int(hdr.get("plant_block", 0))
        tag = str(hdr.get("pdu_tag", ""))
        if block in BLOCKING_PLANT_BLOCKS or tag in BENCH_PDU_TAGS:
            continue

        if block != 0:
            print(f"  note: plant_block={hdr.get('plant_block_name')} — continuing anyway")

        print(f"  {label} OK  plant_block={hdr.get('plant_block_name')}  pdu={tag}")
        return hdr

    block_name = hdr.get("plant_block_name", "?") if hdr else "no_feedback"
    tag = str(hdr.get("pdu_tag", "?")) if hdr else "?"
    raise PlantRuntimeError(
        f"{label}: plant path not clear (plant_block={block_name}, pdu={tag}). "
        "Run: python scripts/control_hub.py --port COMx recover --bus N"
    )


def _arm_rs02_plant_slot(
    session: PcbSession,
    slot: int,
    *,
    kd: float = 0.45,
    hz: float = 40.0,
    arm_kp: float = 8.0,
) -> bool:
    """Arm RS02 on the 500 Hz plant path (maintain_enable + MIT), not bench probe."""
    dt = 1.0 / hz
    pos = 1e-6
    got_fb = False
    for _ in range(max(16, int(hz * 0.6))):
        session.send_plant({slot: (pos, 0.0, arm_kp, kd, 0.0)})
        time.sleep(dt)
        frame = session.reader.pop()
        while frame is not None:
            fb = parse_actuator_feedback(frame, slot=slot)
            if fb is not None:
                pos = float(fb["position"])
                got_fb = True
            frame = session.reader.pop()
    for _ in range(6):
        session.send_plant({slot: (pos, 0.0, 0.0, 0.0, 0.0)})
        time.sleep(dt)
    return got_fb


def warmup_plant_actuators(session: PcbSession, slots: list[int]) -> None:
    """Recover + brief plant MIT per teleop slot (arms drives without bench SESSION_END reset)."""
    if not slots:
        return
    print("Waking actuators (plant path arm)...")
    release_bench_gates(session)
    for slot in slots:
        cfg = slot_config(slot)
        if not cfg.enabled or cfg.protocol_name != "robstride":
            continue
        if not is_rs02_plant_bus(cfg.bus):
            continue
        label = f"slot{slot} {can_bus_label(cfg.bus)} 0x{cfg.motor_id:02X}"
        ok = _arm_rs02_plant_slot(session, slot)
        if ok:
            print(f"  arm OK  {label}")
        else:
            print(
                f"  WARNING: arm failed {label} — "
                "check harness, motor_id vs `config show`, or mechanical fault"
            )
    release_stuck(session, rounds=16)


def assert_plant_teleop_slot(slot: int, bus: int, protocol_name: str) -> None:
    """Plant teleop: RS02 on FDCAN CH1–2 or MCP CH4–6; Damiao on CH3."""
    if protocol_name == "damiao":
        if bus != 3:
            raise PlantRuntimeError(f"slot {slot}: Damiao plant teleop expects CH3 (bus 3).")
        return
    if protocol_name != "robstride":
        raise PlantRuntimeError(f"slot {slot}: protocol {protocol_name!r} has no plant teleop.")
    if not is_rs02_plant_bus(bus):
        raise PlantRuntimeError(
            f"slot {slot} bus {bus}: RS02 plant teleop supports CH1–2 (FDCAN) or CH4–6 (MCP)."
        )


def assert_fdcan_rs02_slot(slot: int, bus: int, protocol_name: str) -> None:
    """Deprecated alias — use assert_plant_teleop_slot."""
    assert_plant_teleop_slot(slot, bus, protocol_name)


@contextmanager
def rs2_bench(session: PcbSession, bus: int) -> Iterator[None]:
    """Scoped RS2 maintenance (cal / discover / probe). Always releases gates on exit."""
    bus = normalize_can_bus(bus)
    session.rs2_session_begin(bus)
    try:
        yield
    finally:
        try:
            session.rs2_session_end(bus)
        except Exception:
            pass
        release_bench_gates(session)
        time.sleep(0.15)
        heal_usb(session, rounds=10)
