#!/usr/bin/env python3
"""Jetson-side PDB (Power Distribution Board) UART simulator.

Stands in for the real PDB MCU on the 64 B UART4 link described in
docs/pdb-uart-v1.md, so the Jetson<->Controls wiring and the USB
host_exchange `pdb[64]` mirror can be exercised before real PDB firmware
exists. Speaks PDBC/PDBF frames only -- see deft_controls_sdk.pdb for the
pack/parse/CRC contract (bit-exact port of App/Src/host/pdb_link.c).

Wire-up (SSH'd into the Jetson, talking to a spare USB-UART or the Jetson's
own UART header -- NOT to the Controls board's own USB CDC):

    Jetson UART1 TX  -> Controls PC11 (UART4 RX)
    Jetson UART1 RX  <- Controls PC10 (UART4 TX)
    GND              <- common ground, both boards
    115200 8N1

    Optional: Jetson GPIO08 (header pin 16) <- Controls hard-ESTOP wire, for
    a live --gpio-estop reading instead of a fixed --estop-sense value. This
    is a *read*, matching the PDB's own documented role (docs/pdb-uart-v1.md):
    the wire is active-low (HIGH = power allowed, LOW = asserted/cut) and is
    driven *by* Controls, never by this script -- --gpio-estop only senses it.

Controls-side firmware must have UART4_MODE == UART4_MODE_PDB in
App/Inc/host/uart4_mode.h (this is the shipping default -- see that header's
comment for why the alternate roles are unsafe on a board with the PDB
connector populated).

Usage:
    python pdb_uart_sim.py --port /dev/ttyTHS1 --hz 20
    python pdb_uart_sim.py --port COM9 --hz 20 --rail-v 4800 1900 1200 500
    python pdb_uart_sim.py --port /dev/ttyUSB0 --simulate-kill-after 10

    # Continuous randomized telemetry + periodic random fault cycling,
    # live ESTOP sense off Jetson header pin 16 (GPIO08, BOARD numbering):
    python pdb_uart_sim.py --port /dev/ttyTHS1 --hz 20 --random \
        --gpio-estop 16 --seed 1

NEVER point --port at the Controls board's USB CDC (COM5 in this repo's
convention) -- that is the separate host_exchange link, owned by whichever
agent currently holds the hot-COM lane. This script refuses that port name
as a guardrail; use a spare USB-UART / Jetson UART header instead.
"""
from __future__ import annotations

import argparse
import os
import random
import sys
import time
from pathlib import Path
from typing import List, Optional, Sequence

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from deft_controls_sdk.pdb import (
    KILL_NORMAL,
    KILL_REASON_BUTTON,
    KILL_REASON_NAMES,
    KILL_REASON_NONE,
    KILL_REASON_OTHER,
    KILL_REASON_OVERCURRENT,
    KILL_REASON_OVERTEMP,
    KILL_REASON_UNDERVOLTAGE,
    KILL_SOFT_READY,
    KILL_SOFT_REQ,
    KILL_STATE_NAMES,
    MAGIC_CMD,
    PdbFrameReader,
    pack_feedback,
    parse_command,
)

try:
    import serial
except ImportError as exc:
    raise ImportError("pyserial required: pip install pyserial") from exc


class GpioEstopReader:
    """Live hard-ESTOP wire sense off a Jetson header pin (BOARD numbering).

    Read-only -- this script never drives the pin. Controls owns and drives
    the wire (docs/pdb-uart-v1.md); the PDB (this sim) only cross-checks it.
    """

    def __init__(self, board_pin: Optional[int]) -> None:
        self._pin = board_pin
        self._gpio = None
        if board_pin is None:
            return
        try:
            import Jetson.GPIO as GPIO  # type: ignore[import-not-found]
        except ImportError as exc:
            raise ImportError(
                "--gpio-estop needs Jetson.GPIO ('pip install Jetson.GPIO') "
                "and must run on the Jetson itself -- omit --gpio-estop to "
                "use the fixed --estop-sense value instead"
            ) from exc
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(board_pin, GPIO.IN)
        self._gpio = GPIO

    def read(self, fallback: int) -> int:
        if self._gpio is None:
            return fallback
        return 1 if self._gpio.input(self._pin) else 0

    def close(self) -> None:
        if self._gpio is not None:
            self._gpio.cleanup(self._pin)


class TelemetryJitter:
    """Random-walk-within-bounds telemetry, centered on the --pack-v/--rail-v/
    etc. values (used as the fixed values when --random is off). Voltages
    wander a small +/- pct around nominal; currents wander a wider pct since
    real load current swings more than bus voltage -- both clipped to
    [0, 65535] raw counts (uint16 wire field, 10 mV or 10 mA per count per
    docs/pdb-uart-v1.md's placeholder scale)."""

    def __init__(
        self,
        pack_v: Sequence[int],
        rail_v: Sequence[int],
        pack_i: Sequence[int],
        rail_i: Sequence[int],
        *,
        voltage_jitter_pct: float,
        current_jitter_pct: float,
        current_floor: int,
    ) -> None:
        self._v_pct = voltage_jitter_pct
        self._i_pct = current_jitter_pct
        # Idle current defaults are often 0 -- jittering around a literal 0
        # center never produces a plausible nonzero draw, so give currents a
        # small floor to wander around when the user didn't override them.
        self._pack_v_center = list(pack_v)
        self._rail_v_center = list(rail_v)
        self._pack_i_center = [max(int(v), current_floor) for v in pack_i]
        self._rail_i_center = [max(int(v), current_floor) for v in rail_i]

    @staticmethod
    def _clip(v: float) -> int:
        return max(0, min(65535, int(round(v))))

    def _wander(self, centers: Sequence[int], pct: float) -> List[int]:
        return [
            self._clip(c * (1.0 + random.uniform(-pct, pct) / 100.0))
            for c in centers
        ]

    def sample(self) -> tuple:
        return (
            self._wander(self._pack_v_center, self._v_pct),
            self._wander(self._rail_v_center, self._v_pct),
            self._wander(self._pack_i_center, self._i_pct),
            self._wander(self._rail_i_center, self._i_pct),
        )


class KillSim:
    """Stubs the PDB side of the soft-kill / hard-ESTOP handshake described
    in pdb-uart-v1.md: NORMAL -> SOFT_KILL_REQ -> (controls parks + sends
    SOFT_KILL_READY) -> we "open contactors" (stay at SOFT_KILL_READY here;
    a real PDB would report the final state once contactors actually open).
    Never opens on soft-kill status alone -- only after the controls ack.
    """

    def __init__(self, simulate_kill_after: Optional[float]) -> None:
        self._simulate_kill_after = simulate_kill_after
        self._start = time.monotonic()
        self._triggered = False
        self._opened = False
        self.state = KILL_NORMAL
        self.reason = KILL_REASON_NONE

    def tick(self, last_cmd_kill_request: int) -> None:
        if (
            self._simulate_kill_after is not None
            and not self._triggered
            and (time.monotonic() - self._start) >= self._simulate_kill_after
        ):
            self._triggered = True
            self.state = KILL_SOFT_REQ
            self.reason = KILL_REASON_OTHER
            print("[pdb-sim] injected SOFT_KILL_REQ (fault simulation)")

        if (
            self._triggered
            and not self._opened
            and last_cmd_kill_request == KILL_SOFT_READY
        ):
            self._opened = True
            self.state = KILL_SOFT_READY
            print("[pdb-sim] controls acked SOFT_KILL_READY -> contactors opened (simulated)")


class RandomFaultSim:
    """Same ack-gated staging as KillSim (NORMAL -> SOFT_KILL_REQ -> wait for
    controls' SOFT_KILL_READY -> "contactors open"), but self-triggers
    indefinitely at random intervals with a randomly chosen plausible reason,
    then auto-recovers to NORMAL after a random hold -- for exercising the
    state machine continuously instead of once. Same invariant as KillSim:
    never reports contactors open on soft-kill status alone, only after the
    controls ack in the command frame."""

    _REASONS = (
        KILL_REASON_UNDERVOLTAGE,
        KILL_REASON_OVERCURRENT,
        KILL_REASON_OVERTEMP,
        KILL_REASON_BUTTON,
    )

    def __init__(self, interval_s: Sequence[float], hold_s: Sequence[float]) -> None:
        self._interval_lo, self._interval_hi = interval_s
        self._hold_lo, self._hold_hi = hold_s
        self.state = KILL_NORMAL
        self.reason = KILL_REASON_NONE
        self._recover_at: Optional[float] = None
        self._next_trigger = time.monotonic() + random.uniform(
            self._interval_lo, self._interval_hi
        )

    def tick(self, last_cmd_kill_request: int) -> None:
        now = time.monotonic()

        if self.state == KILL_NORMAL and now >= self._next_trigger:
            self.state = KILL_SOFT_REQ
            self.reason = random.choice(self._REASONS)
            print(
                f"[pdb-sim] random fault: SOFT_KILL_REQ "
                f"({KILL_REASON_NAMES[self.reason]})"
            )
            return

        if self.state == KILL_SOFT_REQ and last_cmd_kill_request == KILL_SOFT_READY:
            self.state = KILL_SOFT_READY
            self._recover_at = now + random.uniform(self._hold_lo, self._hold_hi)
            print(
                "[pdb-sim] controls acked SOFT_KILL_READY -> "
                "contactors opened (simulated)"
            )
            return

        if (
            self.state == KILL_SOFT_READY
            and self._recover_at is not None
            and now >= self._recover_at
        ):
            self.state = KILL_NORMAL
            self.reason = KILL_REASON_NONE
            self._recover_at = None
            self._next_trigger = now + random.uniform(
                self._interval_lo, self._interval_hi
            )
            print("[pdb-sim] fault cleared -> NORMAL")


def main(argv: Optional[list] = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--port", required=True, help="Jetson-side UART device (NOT Controls' COM5)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--hz", type=float, default=20.0, help="PDBF feedback TX rate")
    ap.add_argument("--pack-v", nargs=4, type=int, default=[4800, 4800, 0, 0], metavar=("V0", "V1", "V2", "V3"))
    ap.add_argument("--rail-v", nargs=4, type=int, default=[4800, 1900, 1200, 500], metavar=("V0", "V1", "V2", "V3"))
    ap.add_argument("--pack-i", nargs=4, type=int, default=[0, 0, 0, 0], metavar=("I0", "I1", "I2", "I3"))
    ap.add_argument("--rail-i", nargs=4, type=int, default=[0, 0, 0, 0], metavar=("I0", "I1", "I2", "I3"))
    ap.add_argument("--contactor-state", type=lambda s: int(s, 0), default=0x0F, help="bitmask when NORMAL/SOFT_KILL_REQ; forced 0 while SOFT_KILL_READY (simulated open)")
    ap.add_argument("--estop-sense", type=int, default=1, choices=(0, 1), help="fixed hard-ESTOP wire readback, used when --gpio-estop is not given")
    ap.add_argument("--gpio-estop", type=int, default=None, metavar="BOARD_PIN", help="live-read ESTOP sense off this Jetson header pin (BOARD numbering, e.g. 16 for GPIO08) instead of --estop-sense; needs Jetson.GPIO, Jetson-only")
    ap.add_argument(
        "--simulate-kill-after",
        type=float,
        default=None,
        help="seconds after start to inject a single SOFT_KILL_REQ and exercise the handshake once; ignored if --random is set",
    )
    ap.add_argument(
        "--force-kill-state",
        type=int,
        default=None,
        choices=(0, 1, 2, 3),
        metavar="N",
        help="override PDBF kill_state every TX (0 NORMAL / 1 SOFT_KILL_REQ / "
        "2 SOFT_KILL_READY / 3 HARD_ESTOP); skips KillSim for LED/PDU prove",
    )
    ap.add_argument(
        "--force-kill-reason",
        type=int,
        default=7,
        metavar="N",
        help="kill_reason when --force-kill-state is set (default 7=OTHER)",
    )
    ap.add_argument("--random", action="store_true", help="continuous randomized telemetry (within --*-jitter-pct of the given centers) + repeated random fault-cycling (see --fault-interval-s/--fault-hold-s), instead of fixed values / a single scripted kill")
    ap.add_argument("--seed", type=int, default=None, help="seed the RNG for reproducible --random runs")
    ap.add_argument("--voltage-jitter-pct", type=float, default=2.0, help="--random: +/- pct wander around --pack-v/--rail-v centers")
    ap.add_argument("--current-jitter-pct", type=float, default=40.0, help="--random: +/- pct wander around --pack-i/--rail-i centers (wider than voltage -- real load current swings more)")
    ap.add_argument("--current-floor", type=int, default=20, help="--random: minimum current center (counts, 10 mA/count) so a 0 default still wanders to something nonzero")
    ap.add_argument("--fault-interval-s", nargs=2, type=float, default=[20.0, 60.0], metavar=("MIN", "MAX"), help="--random: seconds between auto-recovering and the next random fault trigger")
    ap.add_argument("--fault-hold-s", nargs=2, type=float, default=[3.0, 10.0], metavar=("MIN", "MAX"), help="--random: seconds to hold SOFT_KILL_READY (contactors open) before auto-recovering to NORMAL")
    ap.add_argument("--quiet", action="store_true", help="suppress the periodic status line")
    ap.add_argument(
        "--tx-pace-us",
        type=int,
        default=None,
        metavar="US",
        help="delay between TX bytes in microseconds (0=bulk write). "
        "Default: 500 on /dev/ttyTHS* (tegra HSUART bulk write returns NULs "
        "on header loopback; paced TX works), else 0",
    )
    args = ap.parse_args(argv)

    if args.seed is not None:
        random.seed(args.seed)

    if args.port.strip().upper() == "COM5":
        print(
            "Refusing --port COM5: that is the Controls board's USB CDC "
            "host_exchange link, not the PDB UART. Use a spare USB-UART or "
            "the Jetson's own UART header instead.",
            file=sys.stderr,
        )
        return 2

    try:
        gpio = GpioEstopReader(args.gpio_estop)
    except ImportError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    pace_us = args.tx_pace_us
    if pace_us is None:
        # tegra194-hsuart (/dev/ttyTHS*): bulk write() loopback reads back
        # leading/all 0x00; ~500 us/byte pacing echoes cleanly. Not needed on
        # normal USB-UARTs.
        port_name = args.port.strip()
        pace_us = 500 if ("ttyTHS" in port_name or "ttyTHS" in os.path.basename(port_name)) else 0

    def uart_write(ser: "serial.Serial", data: bytes) -> None:
        if pace_us <= 0:
            ser.write(data)
            return
        gap = pace_us / 1_000_000.0
        for b in data:
            ser.write(bytes([b]))
            time.sleep(gap)

    try:
        # write_timeout=0: non-blocking bulk TX when unpaced. With pacing we
        # need a real timeout so a stuck CTS/driver cannot hang forever.
        ser = serial.Serial(
            args.port,
            args.baud,
            timeout=0,
            write_timeout=(1.0 if pace_us > 0 else 0),
        )
    except serial.SerialException as exc:
        print(f"Cannot open {args.port}: {exc}", file=sys.stderr)
        gpio.close()
        return 1

    period = 1.0 / args.hz
    reader = PdbFrameReader(MAGIC_CMD)
    kill = (
        RandomFaultSim(args.fault_interval_s, args.fault_hold_s)
        if args.random
        else KillSim(args.simulate_kill_after)
    )
    jitter = (
        TelemetryJitter(
            args.pack_v,
            args.rail_v,
            args.pack_i,
            args.rail_i,
            voltage_jitter_pct=args.voltage_jitter_pct,
            current_jitter_pct=args.current_jitter_pct,
            current_floor=args.current_floor,
        )
        if args.random
        else None
    )
    tx_seq = 0
    last_cmd: Optional[dict] = None
    last_status_print = 0.0
    next_tx = time.monotonic()

    mode_desc = "random telemetry+fault-cycling" if args.random else "fixed values"
    estop_desc = f"GPIO board pin {args.gpio_estop}" if args.gpio_estop is not None else f"fixed={args.estop_sense}"
    pace_desc = f"tx_pace={pace_us}us/byte" if pace_us > 0 else "tx_pace=bulk"
    print(
        f"[pdb-sim] TX PDBF @ {args.hz:.1f} Hz on {args.port} ({args.baud} 8N1) "
        f"-- {mode_desc}, {pace_desc}, estop_sense: {estop_desc} -- Ctrl+C to stop"
    )

    try:
        while True:
            now = time.monotonic()

            chunk = ser.read(4096)
            if chunk:
                for frame in reader.feed(chunk):
                    parsed = parse_command(frame)
                    if parsed is not None:
                        last_cmd = parsed

            kill.tick(last_cmd["kill_request"] if last_cmd is not None else KILL_NORMAL)

            if args.force_kill_state is not None:
                kill_state = args.force_kill_state
                kill_reason = args.force_kill_reason
            else:
                kill_state = kill.state
                kill_reason = kill.reason
            estop_sense = gpio.read(args.estop_sense)

            if now >= next_tx:
                next_tx = now + period if next_tx < now - period else next_tx + period
                heartbeat_echo = last_cmd["heartbeat"] if last_cmd is not None else 0
                if jitter is not None:
                    pack_v, rail_v, pack_i, rail_i = jitter.sample()
                else:
                    pack_v, rail_v = args.pack_v, args.rail_v
                    pack_i, rail_i = args.pack_i, args.rail_i
                # Contactor readback reflects reality: forced 0 (all open)
                # while SOFT_KILL_READY, so this byte isn't just an echo of
                # the CLI flag regardless of the simulated kill state.
                contactor_state = 0 if kill_state == KILL_SOFT_READY else args.contactor_state
                frame = pack_feedback(
                    seq=tx_seq & 0xFF,
                    pack_v=pack_v,
                    rail_v=rail_v,
                    pack_i=pack_i,
                    rail_i=rail_i,
                    contactor_state=contactor_state,
                    kill_state=kill_state,
                    kill_reason=kill_reason,
                    estop_sense=estop_sense,
                    fault_flags=0,
                    heartbeat_echo=heartbeat_echo,
                )
                try:
                    uart_write(ser, frame)
                except serial.SerialTimeoutException:
                    pass  # peer not draining yet (e.g. no Controls board attached) -- drop and retry
                tx_seq += 1

            if not args.quiet and (now - last_status_print) >= 1.0:
                last_status_print = now
                if last_cmd is None:
                    rx_desc = "no PDBC seen yet"
                else:
                    rx_desc = (
                        f"rx_seq={last_cmd['seq']:3d} "
                        f"rail_enable_cmd=0x{last_cmd['rail_enable_cmd']:02X} "
                        f"kill_request={last_cmd['kill_request']} "
                        f"heartbeat={last_cmd['heartbeat']}"
                    )
                print(
                    f"[pdb-sim] tx_seq={tx_seq:6d} "
                    f"kill_state={KILL_STATE_NAMES.get(kill_state, str(kill_state)):16s} "
                    f"kill_reason={KILL_REASON_NAMES.get(kill_reason, str(kill_reason)):8s} "
                    f"estop_sense={estop_sense} "
                    f"{rx_desc}"
                )

            time.sleep(0.001)
    except KeyboardInterrupt:
        print("\n[pdb-sim] stopped")
    finally:
        ser.close()
        gpio.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
