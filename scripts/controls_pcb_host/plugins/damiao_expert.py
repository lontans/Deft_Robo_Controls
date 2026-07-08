"""Damiao bench expert paths — library for damiao_scan CLI."""
from __future__ import annotations

import argparse
import time
from typing import List, Optional, Sequence, Tuple

import serial

from controls_pcb_host import commands as cmd
from controls_pcb_host.feedback import (
    dm_fault_found,
    parse_dm_probe_pdu,
    parse_dm_from_actuator,
    parse_feedback_header,
)
from controls_pcb_host.protocol import PLANT_MCU_STATE_DIAG_ONLY, PLANT_MCU_STATE_NORMAL
from controls_pcb_host.protocol.diag_pdu import (
    DM_MASTER_ANY,
    DM_PROBE_ALL,
    DM_PROBE_CLEAR_FAULT,
    DM_PROBE_DISABLE,
    DM_PROBE_ENABLE,
    DM_PROBE_ID_SWEEP,
    DM_PROBE_KIND_NAMES,
    DM_PROBE_MIT,
    DM_PROBE_READ_PARAM,
    DM_PROBE_REG_SCAN,
    DM_REG_CTRL_MODE,
    DM_REG_ESC_ID,
    DM_REG_MST_ID,
    SESSION_BEGIN,
    SESSION_END,
)
from controls_pcb_host.protocol.schema import HOST_FEEDBACK_MAGIC, IMAGE_BYTES
from controls_pcb_host.plugins.damiao import clamp_listen_ms, probe_timeout_s, wait_probe_response
from controls_pcb_host.transport import FrameReader, SerialRxPump

DAMIAO_PLANT_SLOT = 2
DEFAULT_MASTER_IDS: Tuple[int, ...] = (
    0x16, 0x11, 0x12, 0x13, 0, 1, 2, 0x7F, 0x7E, 0xFD, 0xFC,
)
PRIORITY_MOTOR_IDS: Tuple[int, ...] = (0, 1, 2, 3, 4, 5, 0x10, 0x11, 0x7F)
PROBE_KIND_NAMES = DM_PROBE_KIND_NAMES


def normalize_bus(bus: int) -> int:
    return max(1, min(6, int(bus)))


def pdu_tag_name(frame: bytes) -> str:
    if len(frame) != IMAGE_BYTES:
        return "?"
    tag = frame[530]
    if tag == 0:
        return "0"
    if 32 <= tag <= 126:
        return chr(tag)
    return f"0x{tag:02X}"


def pdu_has_dm_fw_marker(frame: bytes) -> bool:
    if len(frame) != IMAGE_BYTES:
        return False
    return frame[559] == ord("D") and frame[560] == ord("1")


def format_miss(
    motor_id: int,
    master_id: int,
    probe_kind: int,
    resp: Optional[dict],
    mcu_timeout: bool,
) -> str:
    kind_name = PROBE_KIND_NAMES.get(probe_kind, f"0x{probe_kind:02X}")
    if mcu_timeout or resp is None:
        return f"MISS  id=0x{motor_id:02X}  {kind_name}  timeout (no 'm' PDU or slot2 mirror)"
    via = " slot2" if resp.get("via_actuator_slot") else ""
    rx_id = resp.get("rx_can_id", 0)
    extra = f"  rx_id=0x{rx_id:03X}" if rx_id else ""
    raw = resp.get("raw_frames", 0)
    if raw > 0 and not resp.get("found"):
        extra += "  (bus traffic — not a valid register reply for this ID)"
    elif raw > 0 and resp.get("found") and not is_can_motor_hit(resp):
        extra += "  (stale/partial parse — ignore found bit)"
    return (
        f"MISS  id=0x{motor_id:02X}  {kind_name}{via}  "
        f"tx={resp.get('tx_frames', 0)}  rx_raw={resp['raw_frames']}{extra}  "
        f"found={int(resp['found'])}"
    )


def send_dm_probe(
    ser: serial.Serial,
    reader: FrameReader,
    motor_id: int,
    probe_kind: int,
    seq: int,
    timeout_s: float,
    bus: int = 3,
    master_id: int = 0,
    listen_ms: int = 15,
    param_rid: int = DM_REG_ESC_ID,
    end_id: int = 0,
) -> Tuple[Optional[dict], int, bool]:
    reader.drain()
    ser.write(
        cmd.build_dm_probe_command(
            motor_id, probe_kind, seq, bus=bus,
            master_id=master_id, listen_ms=listen_ms,
            param_rid=param_rid, end_id=end_id,
        )
    )
    ser.flush()
    pk = probe_kind if probe_kind in (SESSION_BEGIN, SESSION_END) else probe_kind
    resp = wait_probe_response(reader, motor_id, timeout_s, probe_kind=pk)
    return resp, seq + 1, resp is None


def feedback_motor_id(byte0: int) -> int:
    err = (byte0 >> 4) & 0x0F
    lo = byte0 & 0x0F
    if byte0 <= 0x0F:
        return byte0
    if err >= 0x08:
        return lo
    return byte0 & 0xFF


def format_hit(resp: dict, motor_id: int, master_id: int, kind: int) -> str:
    data = resp["can_data"]
    bid = feedback_motor_id(data[0]) if data else resp["discovered_id"]
    kind_name = PROBE_KIND_NAMES.get(resp["probe_kind"], f"0x{resp['probe_kind']:02X}")
    reg = ""
    rid = resp.get("param_rid", 0)
    if rid in (DM_REG_ESC_ID, DM_REG_MST_ID, DM_REG_CTRL_MODE):
        rval = resp.get("param_value", 0)
        rname = {DM_REG_ESC_ID: "ESC_ID", DM_REG_MST_ID: "MST_ID", DM_REG_CTRL_MODE: "CTRL_MODE"}.get(rid, f"0x{rid:02X}")
        reg = f"  {rname}=0x{rval & 0xFFFFFFFF:X}"
    is_param = resp.get("probe_kind") in (DM_PROBE_REG_SCAN, DM_PROBE_READ_PARAM, DM_PROBE_ID_SWEEP)
    if is_param:
        motion = "  motion=n/a (register read; use MIT probe while enabled for pos/temp)"
    else:
        motion = f"  pos={resp['position']:.4f}  temp={resp['temperature']:.1f}C"
    return (
        f"FOUND  probe=0x{motor_id:02X}  esc_id=0x{bid:02X}  "
        f"master_rx=0x{resp['master_id']:02X}  "
        f"mode={kind_name}{reg}{motion}  "
        f"err=0x{resp['err']:X}  raw={resp['raw_frames']}  tx={resp['tx_frames']}"
    )


def build_id_list(start: int, end: int, deep: bool) -> List[int]:
    start = max(0, min(255, start))
    end = max(0, min(255, end))
    if end < start:
        start, end = end, start
    full = list(range(start, end + 1))
    if deep:
        head = [i for i in PRIORITY_MOTOR_IDS if start <= i <= end]
        seen = set(head)
        return head + [i for i in full if i not in seen]
    head = [i for i in (1, 2, 3, 4, 5) if start <= i <= end]
    seen = set(head)
    return head + [i for i in full if i not in seen]


def resolve_master_id_list(master_id_arg: str) -> Sequence[int]:
    if master_id_arg == "any":
        return (DM_MASTER_ANY,)
    if master_id_arg == "scan":
        return DEFAULT_MASTER_IDS
    return (int(master_id_arg, 0) & 0xFF,)


def true_esc_id(resp: Optional[dict]) -> int:
    if resp is None:
        return 0
    if resp.get("param_rid") == DM_REG_ESC_ID and resp.get("param_value", 0):
        return int(resp["param_value"]) & 0xFF
    return int(resp.get("discovered_id", 0)) & 0xFF


def is_can_motor_hit(resp: Optional[dict]) -> bool:
    """True when MCU parsed a Damiao CAN response for this probe."""
    if resp is None:
        return False
    if resp.get("probe_kind") in (SESSION_BEGIN, SESSION_END):
        return False
    if resp.get("raw_frames", 0) <= 0:
        return False
    if not resp.get("found"):
        return False
    if resp.get("probe_kind") in (DM_PROBE_CLEAR_FAULT, DM_PROBE_ENABLE):
        return False
    if resp.get("probe_kind") in (DM_PROBE_REG_SCAN, DM_PROBE_READ_PARAM, DM_PROBE_ID_SWEEP):
        esc = true_esc_id(resp)
        probe_id = int(resp.get("probe_id", 0)) & 0xFF
        if esc == 0 and resp.get("master_id", 0) in (0, 0xFF):
            return False
        # Register discovery is ESC_ID read only (MST is supplemental metadata).
        if resp.get("param_rid") == DM_REG_MST_ID:
            return False
        if resp.get("probe_kind") == DM_PROBE_ID_SWEEP:
            return esc != 0
        return esc == probe_id
    return True


def probe_has_hit(resp: Optional[dict]) -> bool:
    return is_can_motor_hit(resp)


def collect_frames(reader: FrameReader, duration_s: float) -> List[bytes]:
    deadline = time.monotonic() + duration_s
    out: List[bytes] = []
    while time.monotonic() < deadline:
        frame = reader.pop()
        while frame is not None:
            out.append(frame)
            frame = reader.pop()
        time.sleep(0.002)
    return out


def summarize_usb_frames(reader: FrameReader, frames: List[bytes], label: str) -> None:
    print(f"{label}: {len(frames)} frame(s)  (reader total={reader.total_frames})")
    if not frames:
        print("  No 562 B feedback — check COM port, CDC driver, or MCU app loop.")
        return
    hdr = parse_feedback_header(frames[-1])
    if hdr:
        print(f"  last tick=0x{hdr['tick']:03X}  ack={hdr['last_cmd_seq']}  "
              f"mcu_state={hdr['mcu_state']}")
    tags: dict[str, int] = {}
    dm_slot = 0
    dm_fw = 0
    for fr in frames:
        tag = pdu_tag_name(fr)
        tags[tag] = tags.get(tag, 0) + 1
        if pdu_has_dm_fw_marker(fr):
            dm_fw += 1
        act = parse_actuator_feedback(fr, DAMIAO_PLANT_SLOT)
        if act and dm_fault_is_probe(act["fault"]):
            dm_slot += 1
    print(f"  PDU tags: {dict(sorted(tags.items()))}")
    if dm_fw:
        print(f"  DM firmware marker D1 in {dm_fw}/{len(frames)} frame(s)")
    elif any(tags.get(t, 0) for t in tags if t == "S"):
        print("  DM firmware marker D1: absent (image predates DM plant_diag)")
    if dm_slot:
        print(f"  slot2 DM fault marker in {dm_slot}/{len(frames)} frame(s)")


def diagnose_mcu_ack(
    sent_seq: int,
    expect_mcu_state: int,
    frames: List[bytes],
    expect_kind: Optional[int] = None,
    probe_id: Optional[int] = None,
) -> None:
    print("--- MCU ack diagnosis ---")
    print(f"  sent seq={sent_seq}  mcu_state={expect_mcu_state} (2=DIAG_ONLY required)")
    m_count = 0
    slot_hits = 0
    for i, fr in enumerate(frames[:32]):
        hdr = parse_feedback_header(fr)
        if hdr is None:
            continue
        tag = pdu_tag_name(fr)
        dm = parse_dm_probe_pdu(fr)
        slot = parse_dm_from_actuator(fr, probe_id or 0) if probe_id is not None else None
        if dm is not None:
            m_count += 1
        if slot is not None:
            slot_hits += 1
        extra = ""
        if dm is not None:
            extra = (
                f" id=0x{dm['probe_id']:02X} kind={dm['probe_kind']} "
                f"found={int(dm['found'])} raw={dm['raw_frames']} tx={dm['tx_frames']}"
            )
        elif slot is not None:
            extra = (
                f" slot2 esc=0x{slot['discovered_id']:02X} "
                f"master=0x{slot['master_id']:02X} raw={slot['raw_frames']}"
            )
        print(
            f"  [{i:2d}] ack_seq={hdr['last_cmd_seq']:3d} "
            f"mcu_rb={hdr['mcu_state']} pdu={tag}{extra}"
        )
    print()
    if expect_kind in (SESSION_BEGIN, SESSION_END):
        ok = any(
            (p := parse_dm_probe_pdu(fr)) is not None and p["probe_kind"] == expect_kind
            for fr in frames
        )
        if ok:
            print("  OK: session 'm' PDU seen")
        else:
            print("  FAIL: no session 'm' PDU — reflash plant_diag.c (DM0 path)")
        return
    if m_count:
        print(f"  OK: 'm' PDU in {m_count}/{min(len(frames), 32)} sampled frame(s)")
    elif slot_hits:
        print(f"  OK: slot-2 mirror in {slot_hits} frame(s) (PDU window may be short)")
    else:
        print("  FAIL: no 'm' PDU or slot-2 mirror")
        print("  -> reflash firmware; run --link-test first")


def run_ack_debug(ser: serial.Serial, args: argparse.Namespace) -> int:
    """Trace USB command -> MCU ack -> probe done without guessing."""
    reader = FrameReader()
    bus = normalize_bus(args.bus)
    motor_id = (args.probe_id if args.probe_id is not None else 1) & 0xFF
    listen_ms = clamp_listen_ms(args.listen_ms)

    with SerialRxPump(ser, reader):
        print(f"MCU ack debug on {ser.port}  bus=CH{bus}  probe_id=0x{motor_id:02X}")
        print()

        seq = 1
        reader.drain()
        cmd = cmd.build_dm_probe_command(0, SESSION_BEGIN, seq, bus=bus)
        ser.write(cmd)
        ser.flush()
        frames = collect_frames(reader, 1.0)
        diagnose_mcu_ack(seq, PLANT_MCU_STATE_DIAG_ONLY, frames, expect_kind=SESSION_BEGIN)
        seq += 1

        reader.drain()
        probe_kind = DM_PROBE_REG_SCAN
        timeout_s = probe_timeout_s(probe_kind, listen_ms)
        cmd = cmd.build_dm_probe_command(
            motor_id, probe_kind, seq, bus=bus,
            master_id=DM_MASTER_ANY, listen_ms=listen_ms,
        )
        ser.write(cmd)
        ser.flush()
        print(f"\n--- REG_SCAN probe seq={seq} timeout={timeout_s:.2f}s ---")
        frames = collect_frames(reader, timeout_s + 0.5)
        diagnose_mcu_ack(
            seq, PLANT_MCU_STATE_DIAG_ONLY, frames,
            expect_kind=probe_kind, probe_id=motor_id,
        )

        send_dm_probe(ser, reader, 0, SESSION_END, seq + 1, 0.5, bus=bus)
    return 0


def run_link_test(ser: serial.Serial, args: argparse.Namespace) -> int:
    reader = FrameReader()

    with SerialRxPump(ser, reader):
        print(f"USB link test on {ser.port} @ {args.baud}")
        print(f"  expect magic=0x{HOST_FEEDBACK_MAGIC:08X}  image={IMAGE_BYTES} B")
        print()

        idle = collect_frames(reader, 1.0)
        summarize_usb_frames(reader, idle, "--- 1) Unsolicited feedback (1 s)")

        seq = 1
        reader.drain()
        ser.write(cmd.build_plant_command(seq))
        ser.flush()
        ping_frames = collect_frames(reader, 0.35)
        acks = [
            parse_feedback_header(f)["last_cmd_seq"]
            for f in ping_frames
            if parse_feedback_header(f)
        ]
        print(f"--- 2) Plain DIAG_ONLY ping seq={seq}: acks = {acks[:8]}")

        bus = normalize_bus(args.bus)
        session_cmd = cmd.build_dm_probe_command(0, SESSION_BEGIN, seq + 1, bus=bus)
        print(f"--- 3) DM session begin TX: {fmt_dm_command(session_cmd)}")
        ser.write(session_cmd)
        ser.flush()
        resp = wait_probe_response(reader, 0, 1.5, probe_kind=SESSION_BEGIN)
        session_frames = collect_frames(reader, 0.35)
        print(f"--- 3) DM session begin (kind={SESSION_BEGIN} bus={bus})")
        if resp:
            print(f"  OK  probe_kind={resp['probe_kind']}  found={resp['found']}")
        else:
            print("  No 'm' PDU for session begin.")
            summarize_usb_frames(reader, session_frames, "  post-session frames")

        has_fw = any(pdu_has_dm_fw_marker(f) for f in idle + ping_frames + session_frames)
        has_m = any(parse_dm_probe_pdu(f) is not None for f in idle + ping_frames + session_frames)

    if not idle and not acks:
        print("\nFAIL: no USB feedback at all.")
        return 1
    if not has_fw:
        print("\nFAIL: USB OK but firmware lacks DM plant_diag (no D1 marker in PDU bytes 29-30).")
        print("  Rebuild + flash from this repo (App/Src/plant/plant_diag.c + plant_feedback.c).")
        return 1
    if not has_m:
        print("\nFAIL: DM firmware present (D1) but DM0 session did not return 'm' PDU.")
        print("  Check plant_command.c routes pdu_dm -> plant_diag_on_dm_command.")
        return 1
    print("\nUSB + DM session path OK.")
    return 0


def run_discover(ser: serial.Serial, args: argparse.Namespace) -> int:
    reader = FrameReader()

    with SerialRxPump(ser, reader):
        wake_mcu_from_teleop(ser)
        reader.drain()
        bus = normalize_bus(args.bus)
        master_ids = resolve_master_id_list(args.master_id)

        id_list = build_id_list(args.start, args.end, args.deep)
        probe_kind = DM_PROBE_REG_SCAN
        listen_ms = clamp_listen_ms(args.listen_ms)
        per_probe_timeout = probe_timeout_s(probe_kind, listen_ms)

        print(f"Damiao discover on bus {bus} (schematic CH{bus})")
        print(f"  motor IDs: 0x{args.start:02X}..0x{args.end:02X} ({len(id_list)} candidates)")
        print(f"  probe: {PROBE_KIND_NAMES.get(probe_kind, probe_kind)}  listen={listen_ms}ms")
        print("  Register scan: TX 0x7FF read ESC_ID/MST_ID, RX on motor Master ID (no filter needed)")
        print("  Works while motor is disabled — no enable required for param read.")
        print()

        seq = 1
        hits: List[dict] = []
        mcu_timeouts = 0
        zero_tx = 0

        seq, _ = begin_dm_session(ser, reader, seq, bus)
        if getattr(args, "debug_usb", False):
            peek = collect_frames(reader, 0.2)
            summarize_usb_frames(reader, peek, "  USB peek after session")

        start_id = max(0, min(255, args.start))
        end_id = max(0, min(255, args.end))
        sweep_listen = clamp_listen_ms(args.listen_ms, minimum=40)
        span = abs(end_id - start_id) + 1
        sweep_timeout = probe_timeout_s(DM_PROBE_ID_SWEEP, sweep_listen, id_span=span)

        if not getattr(args, "host_only", False):
            print(f"MCU-side sweep first (listen={sweep_listen}ms/id, span={span})...")
            resp, seq, mcu_timeout = send_dm_probe(
                ser, reader, start_id, DM_PROBE_ID_SWEEP, seq, sweep_timeout,
                bus=bus, master_id=DM_MASTER_ANY, listen_ms=sweep_listen,
                param_rid=DM_REG_ESC_ID, end_id=end_id,
            )
            if probe_has_hit(resp) and resp:
                esc = true_esc_id(resp)
                master = int(resp.get("master_id", 0)) & 0xFF
                print(format_hit(resp, esc, master or DM_MASTER_ANY, DM_PROBE_ID_SWEEP))
                hits.append(resp)
            elif not mcu_timeout and resp is not None:
                print(f"  Sweep: no hit (raw={resp.get('raw_frames', 0)} tx={resp.get('tx_frames', 0)})")
            else:
                print("  Sweep: timeout — falling back to per-ID reg_scan")
            print()

        if hits:
            best = hits[0]
            esc = true_esc_id(best)
            master = int(best.get("master_id", 0)) & 0xFF
            print(f"Motor on bus: ESC_ID=0x{esc:02X}  Master ID=0x{master:02X}  "
                  f"(rx_raw={best['raw_frames']}  rx_can_id=0x{best.get('rx_can_id', 0):03X})")
            print("  Update plant_config slot 2: motor_id=0x{:02X}  "
                  "master_id=DM_MASTER_ID_AUTO or 0x{:02X}".format(esc, master))
            print("  Confirm: python scripts/damiao_scan.py --port {} --probe-id 0x{:02X} --bus {}".format(
                ser.port, esc, bus))
            send_dm_probe(ser, reader, 0, SESSION_END, seq, 0.5, bus=bus)
            print("\nDone: motor candidate(s) seen during scan.")
            return 0

        print("Per-ID reg_scan fallback (slow on this motor — replies often arrive late):")
        try:
            for mid in id_list:
                resp, seq, mcu_timeout = send_dm_probe(
                    ser, reader, mid, probe_kind, seq, per_probe_timeout,
                    bus=bus, master_id=DM_MASTER_ANY, listen_ms=listen_ms,
                )
                if mcu_timeout:
                    mcu_timeouts += 1
                if resp is not None and resp.get("tx_frames", 0) == 0:
                    zero_tx += 1

                hit = probe_has_hit(resp)
                if hit and resp:
                    esc = true_esc_id(resp)
                    master = int(resp.get("master_id", 0)) & 0xFF
                    print(format_hit(resp, mid, master or DM_MASTER_ANY, probe_kind))
                    hits.append(resp)
                    continue

                if not args.quiet:
                    print(format_miss(mid, DM_MASTER_ANY, probe_kind, resp, mcu_timeout))

            if hits:
                best = max(hits, key=lambda r: (true_esc_id(r), r.get("raw_frames", 0)))
                esc = true_esc_id(best)
                master = int(best.get("master_id", 0)) & 0xFF
                print()
                print(f"Motor on bus: ESC_ID=0x{esc:02X}  Master ID=0x{master:02X}  "
                      f"(rx_raw={best['raw_frames']}  rx_can_id=0x{best.get('rx_can_id', 0):03X})")
                print("  Update plant_config slot 2: motor_id=0x{:02X}  "
                      "master_id=DM_MASTER_ID_AUTO or 0x{:02X}".format(esc, master))
                print("  Confirm: python scripts/damiao_scan.py --port {} --probe-id 0x{:02X} --bus {}".format(
                    ser.port, esc, bus))

            if not hits and getattr(args, "mit_fallback", False):
                print("\nNo reg-scan hits — MIT feedback fallback (one MIT frame per ID)...")
                fallback_kind = DM_PROBE_MIT
                fallback_timeout = probe_timeout_s(fallback_kind, listen_ms)
                for mid in id_list[:32]:
                    resp, seq, mcu_timeout = send_dm_probe(
                        ser, reader, mid, fallback_kind, seq, fallback_timeout,
                        bus=bus, master_id=DM_MASTER_ANY, listen_ms=listen_ms,
                    )
                    if probe_has_hit(resp):
                        print(format_hit(resp, mid, DM_MASTER_ANY, fallback_kind))
                        hits.append(resp)
                        break
                    if not args.quiet:
                        print(format_miss(mid, DM_MASTER_ANY, fallback_kind, resp, mcu_timeout))

            if not hits and getattr(args, "all_modes", False):
                print("\nNo param hits — retrying with MIT+POS+VEL (DM_PROBE_ALL)...")
                retry_kind = DM_PROBE_ALL
                retry_timeout = probe_timeout_s(retry_kind, listen_ms + 8)
                for mid in [i for i in PRIORITY_MOTOR_IDS if i in id_list][:12]:
                    for master in master_ids:
                        resp, seq, mcu_timeout = send_dm_probe(
                            ser, reader, mid, retry_kind, seq, retry_timeout,
                            bus=bus, master_id=master, listen_ms=listen_ms + 8,
                        )
                        if probe_has_hit(resp):
                            print(format_hit(resp, mid, master, retry_kind))
                            hits.append(resp)
                            break
                        if not args.quiet:
                            print(format_miss(mid, master, retry_kind, resp, mcu_timeout))

            if not hits and DM_MASTER_ANY not in master_ids:
                print("\nNo fixed master ID — final pass with master=ANY (0xFF)...")
                any_timeout = probe_timeout_s(DM_PROBE_ALL, listen_ms + 10)
                for mid in id_list[:32]:
                    resp, seq, mcu_timeout = send_dm_probe(
                        ser, reader, mid, DM_PROBE_ALL, seq, any_timeout,
                        bus=bus, master_id=DM_MASTER_ANY, listen_ms=listen_ms + 10,
                    )
                    if probe_has_hit(resp):
                        print(format_hit(resp, mid, DM_MASTER_ANY, DM_PROBE_ALL))
                        hits.append(resp)
                    elif not args.quiet:
                        print(format_miss(mid, DM_MASTER_ANY, DM_PROBE_ALL, resp, mcu_timeout))
        finally:
            send_dm_probe(ser, reader, 0, SESSION_END, seq, 0.5, bus=bus)

        print()
        if mcu_timeouts:
            print(f"MCU feedback timeouts: {mcu_timeouts}")
            print("  Run --link-test; reflash if DM0 never appears in PDU tags.")
        if zero_tx:
            print(f"Probes with tx=0: {zero_tx} (wrong bus index or CAN TX not running on CH{bus})")
        if hits:
            print(f"\nDone: motor candidate(s) seen during scan.")
        else:
            print("No Damiao feedback found.")
            if zero_tx == 0 and mcu_timeouts == 0:
                print("  TX looked healthy (tx>0) but rx_raw=0 on all IDs.")
            print("  See docs/damiao-bringup.md — symptom tree for tx>0 / rx_raw=0.")
            print("  - Motor 24V, common GND, CAN on XT30 (not debug UART).")
            print("  - J4310 has no software termination — need 120R at both bus ends.")
            print("  - Confirm 1 Mbps (register 0x23) via Damiao Assistant + USB2CAN.")
            print("  - Flash latest firmware; run --link-test and --ack-debug.")
            print("  - Try: --mit-fallback  or  --probe-id 6 --bus 3")
        return 0 if hits else 1


def run_fw_sweep(ser: serial.Serial, args: argparse.Namespace) -> int:
    """One command sweeps the whole ESC_ID range on the MCU -- no per-ID USB
    round trip. Same fixed-0x7FF register read as --reg-scan; master ID is
    read out of the reply's arbitration ID, never guessed."""
    reader = FrameReader()

    with SerialRxPump(ser, reader):
        wake_mcu_from_teleop(ser)
        reader.drain()
        bus = normalize_bus(args.bus)
        start_id = max(0, min(255, args.start))
        end_id = max(0, min(255, args.end))
        listen_ms = clamp_listen_ms(args.sweep_listen_ms, minimum=1)
        span = abs(end_id - start_id) + 1
        timeout_s = probe_timeout_s(DM_PROBE_ID_SWEEP, listen_ms, id_span=span)

        print(f"Damiao firmware-side ID sweep on bus {bus} (schematic CH{bus})")
        print(f"  ESC_ID 0x{start_id:02X}..0x{end_id:02X} ({span} candidates), "
              f"listen={listen_ms}ms/id, worst-case {timeout_s:.1f}s")
        print("  Single command: MCU loops the range internally, replies once done or on first hit.")
        print()

        seq = 1
        seq, _ = begin_dm_session(ser, reader, seq, bus)

        print(f"Sweeping (MCU-side, blocking) -- waiting up to {timeout_s:.1f}s for a single reply...")
        try:
            resp, seq, mcu_timeout = send_dm_probe(
                ser, reader, start_id, DM_PROBE_ID_SWEEP, seq, timeout_s,
                bus=bus, master_id=DM_MASTER_ANY, listen_ms=listen_ms,
                param_rid=DM_REG_ESC_ID, end_id=end_id,
            )
        finally:
            end_dm_session(ser, reader, seq, bus)

        if mcu_timeout or resp is None:
            print("No MCU response (timeout) -- sweep may still be running past our wait,")
            print("  or the command never reached plant_diag. Try --ack-debug or --link-test.")
            return 1

        if probe_has_hit(resp):
            hit_id = resp.get("discovered_id") or resp["probe_id"]
            print(format_hit(resp, hit_id, resp.get("master_id", DM_MASTER_ANY), DM_PROBE_ID_SWEEP))
            print(f"\nDone: motor found at ESC_ID=0x{hit_id:02X}, "
                  f"Master ID=0x{resp['master_id']:02X}. Update plant_config with these.")
            return 0

        print(f"No hit across 0x{start_id:02X}..0x{end_id:02X}  "
              f"(raw_frames_seen={resp['raw_frames']}, tx={resp['tx_frames']})")
        if resp["raw_frames"] == 0:
            print("  No CAN traffic at all -- wrong bus, wrong baud (this motor may not be at 1 Mbps),")
            print("  or wiring/termination issue. See docs/damiao-bringup.md.")
        return 1


def send_can_disable(
    ser: serial.Serial,
    reader: FrameReader,
    motor_id: int,
    seq: int,
    bus: int = 3,
) -> int:
    """Disable motor (FF..FF FD in D[7]) — safe idle state, no comm-timeout fault."""
    mid = motor_id & 0xFF
    print(f"  CAN disable  id=0x{mid:02X}  (FF..FF FD in D[7])")
    _, seq, _ = send_dm_probe(
        ser, reader, mid, DM_PROBE_DISABLE, seq, 0.8,
        bus=bus, master_id=DM_MASTER_ANY, listen_ms=12,
    )
    time.sleep(0.1)
    return seq


def send_can_enable(
    ser: serial.Serial,
    reader: FrameReader,
    motor_id: int,
    seq: int,
    bus: int = 3,
    clear_fault: bool = False,
) -> int:
    """Send a single Damiao enable frame (0xFC). Optional clear-fault first."""
    mid = motor_id & 0xFF
    if clear_fault:
        print(f"  CAN clear fault  id=0x{mid:02X}  (FF..FF FB)")
        _, seq, _ = send_dm_probe(
            ser, reader, mid, DM_PROBE_CLEAR_FAULT, seq, 0.8,
            bus=bus, master_id=DM_MASTER_ANY, listen_ms=12,
        )
        time.sleep(0.15)
    print(f"  CAN enable  id=0x{mid:02X}  (FF..FF FC in D[7], legacy J4310 layout)")
    _, seq, _ = send_dm_probe(
        ser, reader, mid, DM_PROBE_ENABLE, seq, 0.8,
        bus=bus, master_id=DM_MASTER_ANY, listen_ms=12,
    )
    time.sleep(0.2)
    return seq


def wake_mcu_from_teleop(ser: serial.Serial) -> None:
    """NORMAL empty frame clears DM session / quiet gates after plant teleop."""
    ser.write(cmd.build_plant_command(0, mcu_state=PLANT_MCU_STATE_NORMAL))
    ser.flush()
    time.sleep(0.08)


def begin_dm_session(
    ser: serial.Serial,
    reader: FrameReader,
    seq: int,
    bus: int,
) -> Tuple[int, bool]:
    """One SESSION_BEGIN (1s). Warn on miss but always continue — probes may still run."""
    reader.drain()
    resp, seq, timed_out = send_dm_probe(
        ser, reader, 0, SESSION_BEGIN, seq, 1.0, bus=bus,
    )
    if timed_out or resp is None:
        print("WARN: DM session begin — no MCU ack")
        print("  Run: python scripts/damiao_scan.py --port COM5 --link-test")
        return seq, False
    return seq, True


def end_dm_session(
    ser: serial.Serial,
    reader: FrameReader,
    seq: int,
    bus: int,
) -> int:
    _, seq, _ = send_dm_probe(ser, reader, 0, SESSION_END, seq, 0.5, bus=bus)
    return seq


def send_mit_hold_stream(
    ser: serial.Serial,
    reader: FrameReader,
    motor_id: int,
    seq: int,
    bus: int,
    hold_ms: int,
    listen_ms: int = 32,
) -> int:
    """Stream MIT bursts so enabled motors do not hit TIMEOUT (ERR=D)."""
    if hold_ms <= 0:
        return seq
    listen_ms = clamp_listen_ms(listen_ms)
    per_probe_s = probe_timeout_s(DM_PROBE_MIT, listen_ms)
    deadline = time.monotonic() + hold_ms / 1000.0
    bursts = 0
    print(f"  MIT hold {hold_ms} ms (burst listen_ms={listen_ms}) — keeps ERR=1 alive...")
    try:
        while time.monotonic() < deadline:
            _, seq, _ = send_dm_probe(
                ser, reader, motor_id & 0xFF, DM_PROBE_MIT, seq, per_probe_s,
                bus=bus, master_id=DM_MASTER_ANY, listen_ms=listen_ms,
            )
            bursts += 1
    except KeyboardInterrupt:
        print("\n  Hold interrupted.")
    print(f"  Hold done ({bursts} MIT burst(s)). Start teleop before TIMEOUT expires.")
    return seq


def resolve_keep_hold_ms(args: argparse.Namespace) -> int:
    hold_ms = getattr(args, "hold_ms", None)
    if hold_ms is not None:
        return max(0, int(hold_ms))
    if getattr(args, "keep_enabled", False):
        return 3000
    return 0


def run_single_probe(ser: serial.Serial, args: argparse.Namespace) -> int:
    reader = FrameReader()

    with SerialRxPump(ser, reader):
        wake_mcu_from_teleop(ser)
        reader.drain()
        bus = normalize_bus(args.bus)
        if args.mit_fallback:
            listen_ms = clamp_listen_ms(max(args.listen_ms, 80))
        else:
            listen_ms = clamp_listen_ms(args.listen_ms)
        if getattr(args, "reg_scan", False) or args.all_modes:
            kind = DM_PROBE_REG_SCAN
        elif getattr(args, "mit_fallback", False) or getattr(args, "feedback", False):
            kind = DM_PROBE_MIT
        else:
            kind = DM_PROBE_REG_SCAN
        master_ids = resolve_master_id_list(args.master_id)
        per_probe_timeout = probe_timeout_s(kind, listen_ms)
        motor_id = args.probe_id & 0xFF
        seq = 1

        seq, _ = begin_dm_session(ser, reader, seq, bus)

        if getattr(args, "enable", False) and not args.mit_fallback and not getattr(args, "feedback", False):
            mid = (args.probe_id if args.probe_id is not None else 1) & 0xFF
            seq = send_can_enable(
                ser, reader, mid, seq, bus=bus,
                clear_fault=True,
            )
            if not args.all_modes and not args.reg_scan:
                verify_listen = clamp_listen_ms(max(args.listen_ms, 40))
                verify_timeout = probe_timeout_s(DM_PROBE_MIT, verify_listen)
                print("  Verifying enable (MIT feedback, ERR nibble should be 1)...")
                resp, seq, mcu_timeout = send_dm_probe(
                    ser, reader, mid, DM_PROBE_MIT, seq, verify_timeout,
                    bus=bus, master_id=DM_MASTER_ANY, listen_ms=verify_listen,
                )
                if resp and not mcu_timeout:
                    err = int(resp.get("err", 0)) & 0x0F
                    if err == 1:
                        print(f"  OK: motor reports enabled (ERR=0x{err:X}, green LED expected).")
                    elif err == 0:
                        print(f"  WARN: feedback ERR=0x{err:X} (disabled).")
                    elif err == 0xD:
                        print(f"  WARN: feedback ERR=0x{err:X} (comm timeout).")
                    else:
                        print(f"  WARN: feedback ERR=0x{err:X} (fault).")
                    if probe_has_hit(resp):
                        print(format_hit(resp, mid, DM_MASTER_ANY, DM_PROBE_MIT))
                elif mcu_timeout:
                    print("  WARN: no MIT feedback after enable (timeout).")
                if getattr(args, "keep_enabled", False):
                    hold_ms = resolve_keep_hold_ms(args)
                    hold_listen = clamp_listen_ms(max(args.listen_ms, 32))
                    seq = send_mit_hold_stream(
                        ser, reader, mid, seq, bus, hold_ms, listen_ms=hold_listen,
                    )
                    print("Enable sequence done — motor left enabled (no disable sent).")
                else:
                    print("Sending disable (safe idle)...")
                    seq = send_can_disable(ser, reader, mid, seq, bus=bus)
                    print("Enable test done — motor disabled.")
                end_dm_session(ser, reader, seq, bus)
                return 0

        if getattr(args, "mit_fallback", False) and getattr(args, "enable", False):
            mid = (args.probe_id if args.probe_id is not None else 1) & 0xFF
            seq = send_can_enable(
                ser, reader, mid, seq, bus=bus,
                clear_fault=getattr(args, "clear_fault", False),
            )

        try:
            resp = None
            mcu_timeout = True
            master = DM_MASTER_ANY
            masters = (DM_MASTER_ANY,) if kind in (
                DM_PROBE_REG_SCAN, DM_PROBE_READ_PARAM, DM_PROBE_DISCOVER,
                DM_PROBE_MIT,
            ) else master_ids
            for master in masters:
                resp, seq, mcu_timeout = send_dm_probe(
                    ser, reader, motor_id, kind, seq, per_probe_timeout,
                    bus=bus, master_id=master, listen_ms=listen_ms,
                )
                if probe_has_hit(resp):
                    break
                if not mcu_timeout and resp is not None and kind not in (DM_PROBE_REG_SCAN, DM_PROBE_READ_PARAM):
                    break
        finally:
            if getattr(args, "enable", False) and not getattr(args, "keep_enabled", False):
                mid = (args.probe_id if args.probe_id is not None else 1) & 0xFF
                seq = send_can_disable(ser, reader, mid, seq, bus=bus)
            send_dm_probe(ser, reader, 0, SESSION_END, seq, 0.5, bus=bus)

        if mcu_timeout or resp is None:
            print("No MCU probe response (timeout).")
            print(f"  USB frames seen: {reader.total_frames} total since open")
            print("  Run: python scripts/damiao_scan.py --port COM5 --link-test")
            return 1
        if probe_has_hit(resp):
            print(format_hit(resp, motor_id, master, kind))
            return 0
        print(format_miss(motor_id, master, kind, resp, mcu_timeout=False))
        return 1


