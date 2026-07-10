#!/usr/bin/env python3
import struct
import sys
import time

sys.path.insert(0, "scripts")

import serial
from controls_pcb_host.plugins.damiao import probe_timeout_s
from controls_pcb_host.plugins.damiao_expert import SESSION_BEGIN, SESSION_END, send_dm_probe
from controls_pcb_host.protocol.diag_pdu import (
    DM_MASTER_ANY,
    DM_PROBE_READ_PARAM,
    DM_PROBE_REG_SCAN,
)
from controls_pcb_host.transport import FrameReader, SerialRxPump

REGS = [
    (0x08, "ESC_ID"),
    (0x07, "MST_ID"),
    (0x0A, "CTRL_MODE"),
    (0x15, "PMAX"),
    (0x16, "VMAX"),
    (0x17, "TMAX"),
    (0x23, "can_br"),
    (0x14, "Gr"),
]


def main() -> int:
    with serial.Serial("COM5", 115200, timeout=0.05) as ser:
        time.sleep(0.3)
        reader = FrameReader()
        with SerialRxPump(ser, reader):
            seq = 0
            _, seq, _ = send_dm_probe(ser, reader, 0, SESSION_BEGIN, seq, 0.5, bus=1)
            for mid in (0x05, 0x06, 0x07):
                print(f"==== id=0x{mid:02X} ====")
                to = probe_timeout_s(DM_PROBE_REG_SCAN, 80)
                resp, seq, _ = send_dm_probe(
                    ser,
                    reader,
                    mid,
                    DM_PROBE_REG_SCAN,
                    seq,
                    to,
                    bus=1,
                    master_id=DM_MASTER_ANY,
                    listen_ms=80,
                )
                ok = bool(resp and resp.get("found"))
                print(f"  reg_scan found={ok}")
                for rid, name in REGS:
                    to = probe_timeout_s(DM_PROBE_READ_PARAM, 80)
                    resp, seq, timed = send_dm_probe(
                        ser,
                        reader,
                        mid,
                        DM_PROBE_READ_PARAM,
                        seq,
                        to,
                        bus=1,
                        master_id=DM_MASTER_ANY,
                        listen_ms=80,
                        param_rid=rid,
                    )
                    if resp is None or timed or not resp.get("found"):
                        raw = None if resp is None else resp.get("raw_frames")
                        print(f"  {name}: miss raw={raw}")
                        continue
                    pv = int(resp.get("param_value", 0)) & 0xFFFFFFFF
                    fv = struct.unpack("<f", struct.pack("<I", pv))[0]
                    print(f"  {name}: u32=0x{pv:08X}  float={fv}")
            send_dm_probe(ser, reader, 0, SESSION_END, seq, 0.5, bus=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
