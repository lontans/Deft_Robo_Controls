#!/usr/bin/env python3
"""controls_hub_dashboard - desktop GUI for normal-operation motor/servo/LED control.

    python scripts/controls_hub_dashboard.py
    python scripts/controls_hub_dashboard.py --port COM5

Talks to the PCB over the 562 B host exchange contract only (docs/host-exchange-v1.md)
via controls_hub_controller.HubSession - the same normal-operation path as
controls_hub_controller, not the RS2/DM/DXL PDU bench backdoors. `pdu` is always zero
on every command frame this GUI sends (asserted in HubSession.build_command()).

Layout
------
Top bar     : port select, connect/disconnect, stream rate, Recover, mcu_state buttons
Status strip: plant_block (color-coded), tick/ack_seq/mcu_state/fb_hz/cmd_hz, banner
Actuators   : one tab per configured slot (position/velocity/kp/kd/torque + feedback)
Servos      : 2 slots (goal position/speed/torque enable + feedback)
LED         : mode/brightness/led_count
Advanced    : hex dump of last command/feedback, scrolling status log, stream toggle

Process model
-------------
A background StreamWorker thread owns all serial I/O: it resends the full held
actuator+servo+LED state (HubSession.build_command()) at the configured rate and polls
feedback every tick, publishing a FeedbackSnapshot the UI thread reads on a timer. The
UI thread never touches the serial port directly - button callbacks only mutate
HubSession's held-state dicts (thread-safe via HubSession's internal lock) or call
recover()/set_mcu_state(), which take the same lock the worker uses for writes.

Stale watchdog: firmware blocks actuator CAN apply if no valid command image has
arrived in the last 500 ms and no enabled slot's desire is non-idle (see
docs/controls_hub-controller-design.md §3). Default stream rate here is 50 Hz, well
under that limit; use the "Stream commands" checkbox in Advanced to pause sending and
observe plant_block flip to HOST_STALE.

Non-goals (see docs/controls_hub_dashboard.md): no RS2/DM/DXL PDU probes, no arrow-key
teleop, no trajectory generator, no firmware changes.
"""
from __future__ import annotations

import argparse
import os
import struct
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from tkinter import messagebox, ttk
from typing import Callable, List, Optional

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from serial.tools import list_ports  # noqa: E402

from controls_hub_controller import (  # noqa: E402
    ActuatorDesire,
    FeedbackImage,
    FeedbackState,
    HubSession,
    InvalidFrameError,
    McuState,
    PlantBlockReason,
)
from controls_hub_controller.config import Protocol, host_table  # noqa: E402
from controls_pcb_host.feedback import parse_feedback_header, parse_servo_feedback  # noqa: E402
from controls_pcb_host.protocol import ACTUATOR_COUNT, MCP_CAN_BUSES, PDU_OFF  # noqa: E402
from controls_pcb_host.transport import DEFAULT_BAUD, STM32_VID  # noqa: E402

UI_REFRESH_MS = 66  # ~15 Hz
DEFAULT_STREAM_HZ = 50.0
MIN_STREAM_HZ = 10.0
MAX_STREAM_HZ = 200.0

PLANT_BLOCK_COLOR = {
    PlantBlockReason.NONE: "#1a7f37",
    PlantBlockReason.HOST_STALE: "#c0392b",
}
_DEFAULT_BLOCK_COLOR = "#b8860b"  # amber for everything else


def _block_name(block: object) -> str:
    return block.name if isinstance(block, PlantBlockReason) else f"unknown({block})"


def _mcu_state_name(value: int) -> str:
    try:
        return McuState(value).name
    except ValueError:
        return f"unknown({value})"


# --------------------------------------------------------------------------------- #
# Data model
# --------------------------------------------------------------------------------- #


@dataclass
class FeedbackSnapshot:
    connected: bool = False
    tick: int = 0
    ack_seq: int = 0
    mcu_state: int = 0
    plant_block: object = PlantBlockReason.NONE
    pdu_tag: str = ""
    cmd_seq_low: Optional[int] = None
    fb_hz: float = 0.0
    cmd_hz: float = 0.0
    last_fb_age_s: float = float("inf")
    actuators: List[Optional[FeedbackState]] = field(default_factory=lambda: [None] * ACTUATOR_COUNT)
    servos: List[Optional[dict]] = field(default_factory=lambda: [None, None])
    last_cmd_bytes: bytes = b""
    last_fb_bytes: bytes = b""


class StreamWorker(threading.Thread):
    """Owns all serial I/O for a connected HubSession. Publishes FeedbackSnapshot via
    on_snapshot; never touches Tk widgets (Tk is not thread-safe)."""

    def __init__(
        self,
        hub: HubSession,
        *,
        get_hz: Callable[[], float],
        get_streaming_enabled: Callable[[], bool],
        on_snapshot: Callable[[FeedbackSnapshot], None],
        on_error: Callable[[Exception], None],
    ) -> None:
        super().__init__(name="hub-dashboard-stream", daemon=True)
        self._hub = hub
        self._get_hz = get_hz
        self._get_streaming_enabled = get_streaming_enabled
        self._on_snapshot = on_snapshot
        self._on_error = on_error
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        cmd_count = 0
        fb_count = 0
        window_t0 = time.perf_counter()
        cmd_hz = 0.0
        fb_hz = 0.0
        last_fb_time: Optional[float] = None
        last_cmd_bytes = b""
        last_fb_bytes = b""
        cmd_seq_low: Optional[int] = None
        next_tick: Optional[float] = None

        try:
            while not self._stop.is_set():
                hz = max(MIN_STREAM_HZ, min(MAX_STREAM_HZ, self._get_hz()))
                period = 1.0 / hz

                if self._get_streaming_enabled():
                    frame = self._hub.build_command()
                    self._hub.write_command(frame)
                    cmd_seq_low = struct.unpack_from("<I", frame, 8)[0] & 0xFF
                    last_cmd_bytes = frame
                    cmd_count += 1

                fb = self._hub.poll_feedback()
                if fb is not None:
                    fb_count += 1
                    last_fb_time = time.perf_counter()
                    last_fb_bytes = fb.raw

                now = time.perf_counter()
                if now - window_t0 >= 0.5:
                    dt = now - window_t0
                    cmd_hz = cmd_count / dt
                    fb_hz = fb_count / dt
                    cmd_count = 0
                    fb_count = 0
                    window_t0 = now

                fb_image: Optional[FeedbackImage] = None
                if last_fb_bytes:
                    try:
                        fb_image = FeedbackImage(last_fb_bytes)
                    except InvalidFrameError:
                        fb_image = None
                # FeedbackImage doesn't carry pdu_tag/fb_seq (deliberately thin core
                # API) - parse_feedback_header separately for that one debug field.
                header = parse_feedback_header(last_fb_bytes) if last_fb_bytes else None
                servos = [
                    parse_servo_feedback(last_fb_bytes, slot) if last_fb_bytes else None for slot in (0, 1)
                ]
                snap = FeedbackSnapshot(
                    connected=True,
                    tick=fb_image.tick if fb_image else 0,
                    ack_seq=fb_image.ack_seq if fb_image else 0,
                    mcu_state=fb_image.mcu_state if fb_image else 0,
                    plant_block=fb_image.plant_block if fb_image else PlantBlockReason.NONE,
                    pdu_tag=header["pdu_tag"] if header else "",
                    cmd_seq_low=cmd_seq_low,
                    fb_hz=fb_hz,
                    cmd_hz=cmd_hz,
                    last_fb_age_s=(now - last_fb_time) if last_fb_time is not None else float("inf"),
                    actuators=fb_image.actuators if fb_image else [None] * ACTUATOR_COUNT,
                    servos=servos,
                    last_cmd_bytes=last_cmd_bytes,
                    last_fb_bytes=last_fb_bytes,
                )
                self._on_snapshot(snap)

                if next_tick is None or next_tick <= now - period:
                    next_tick = now + period
                else:
                    sleep_s = next_tick - now
                    if sleep_s > 0:
                        time.sleep(sleep_s)
                    next_tick += period
        except Exception as exc:  # serial closed/unplugged mid-stream, etc.
            self._on_error(exc)


# --------------------------------------------------------------------------------- #
# UI
# --------------------------------------------------------------------------------- #


class ActuatorPanel(ttk.Frame):
    def __init__(self, master: tk.Widget, slot: int, apply_cb: Callable[[int, ActuatorDesire], None]) -> None:
        super().__init__(master, padding=8)
        self.slot = slot
        self._apply_cb = apply_cb
        cfg = host_table()[slot]
        is_mcp = cfg.bus in MCP_CAN_BUSES
        is_robstride = cfg.protocol == Protocol.ROBSTRIDE

        header = ttk.Label(
            self, text=f"Slot {slot} - CH{cfg.bus} {cfg.protocol_name} id=0x{cfg.motor_id:02X}",
            font=("TkDefaultFont", 10, "bold"),
        )
        header.grid(row=0, column=0, columnspan=4, sticky="w")

        self.vars = {name: tk.StringVar(value="0.0") for name in ("position", "velocity", "kp", "kd", "torque")}
        for i, name in enumerate(("position", "velocity", "kp", "kd", "torque")):
            ttk.Label(self, text=name).grid(row=1 + i, column=0, sticky="w")
            ttk.Entry(self, textvariable=self.vars[name], width=10).grid(row=1 + i, column=1, sticky="w")

        ttk.Button(self, text="Apply slot", command=self._on_apply).grid(row=6, column=0, sticky="w", pady=(4, 0))
        ttk.Button(self, text="Idle", command=self._on_idle).grid(row=6, column=1, sticky="w", pady=(4, 0))

        ttk.Separator(self, orient="horizontal").grid(row=7, column=0, columnspan=4, sticky="ew", pady=6)
        ttk.Label(self, text="Feedback", font=("TkDefaultFont", 9, "bold")).grid(row=8, column=0, sticky="w")
        self.fb_var = tk.StringVar(value="pos=--  vel=--  torque=--  temp=--  fault=--")
        ttk.Label(self, textvariable=self.fb_var, justify="left").grid(row=9, column=0, columnspan=4, sticky="w")

        note_y = 10
        if is_mcp:
            ttk.Label(
                self,
                text="MCP bus (CH4-6): feedback may update in grouped bursts - see docs/bringup.md §7.",
                foreground="#8a6d00", wraplength=360, justify="left",
            ).grid(row=note_y, column=0, columnspan=4, sticky="w", pady=(6, 0))
            note_y += 1
        self._robstride_warn = ttk.Label(
            self,
            text="RS02 may interpolate position between host updates when |velocity|<0.01 (P1 gap) - "
                 "send an explicit nonzero velocity to avoid it.",
            foreground="#8a6d00", wraplength=360, justify="left",
        )
        if is_robstride:
            self._robstride_warn.grid(row=note_y, column=0, columnspan=4, sticky="w", pady=(6, 0))
        self._is_robstride = is_robstride

    def _on_apply(self) -> None:
        try:
            desire = ActuatorDesire(**{k: float(v.get()) for k, v in self.vars.items()})
        except ValueError:
            messagebox.showerror("Invalid value", "Actuator fields must be numeric.")
            return
        self._apply_cb(self.slot, desire)

    def _on_idle(self) -> None:
        for var in self.vars.values():
            var.set("0.0")
        self._apply_cb(self.slot, ActuatorDesire())

    def update_feedback(self, state: Optional[FeedbackState]) -> None:
        if state is None:
            self.fb_var.set("pos=--  vel=--  torque=--  temp=--  fault=--")
            return
        self.fb_var.set(
            f"pos={state.position:+.4f}  vel={state.velocity:+.4f}  torque={state.torque:+.3f}  "
            f"temp={state.temperature:.1f}C  fault=0x{state.fault:08X}"
        )


class ServoPanel(ttk.Frame):
    def __init__(self, master: tk.Widget, slot: int, apply_cb: Callable[[int, dict], None]) -> None:
        super().__init__(master, padding=8)
        self.slot = slot
        self._apply_cb = apply_cb

        ttk.Label(self, text=f"Servo slot {slot}", font=("TkDefaultFont", 10, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w"
        )
        self.servo_id = tk.StringVar(value=str(slot + 1))
        self.position = tk.StringVar(value="0")
        self.speed = tk.StringVar(value="0")
        self.torque_enable = tk.BooleanVar(value=True)

        for i, (label, var) in enumerate((("servo_id", self.servo_id), ("goal_position", self.position),
                                           ("speed", self.speed))):
            ttk.Label(self, text=label).grid(row=1 + i, column=0, sticky="w")
            ttk.Entry(self, textvariable=var, width=10).grid(row=1 + i, column=1, sticky="w")
        ttk.Checkbutton(self, text="torque_enable", variable=self.torque_enable).grid(
            row=4, column=0, columnspan=2, sticky="w"
        )
        ttk.Button(self, text="Apply", command=self._on_apply).grid(row=5, column=0, sticky="w", pady=(4, 0))

        ttk.Separator(self, orient="horizontal").grid(row=6, column=0, columnspan=2, sticky="ew", pady=6)
        self.fb_var = tk.StringVar(value="present=--  speed=--  moving=--")
        ttk.Label(self, textvariable=self.fb_var).grid(row=7, column=0, columnspan=2, sticky="w")

    def _on_apply(self) -> None:
        try:
            payload = {
                "servo_id": int(self.servo_id.get()),
                "position": int(self.position.get()),
                "speed": int(self.speed.get()),
                "torque_enable": 1 if self.torque_enable.get() else 0,
            }
        except ValueError:
            messagebox.showerror("Invalid value", "Servo fields must be integers.")
            return
        self._apply_cb(self.slot, payload)

    def update_feedback(self, fb: Optional[dict]) -> None:
        if fb is None:
            self.fb_var.set("present=--  speed=--  moving=--")
            return
        self.fb_var.set(
            f"present={fb['present_position']}  speed={fb['present_speed']}  moving={bool(fb['moving'])}"
        )


class DashboardApp(tk.Tk):
    def __init__(self, initial_port: Optional[str] = None) -> None:
        super().__init__()
        self.title("controls_hub_dashboard")
        self.geometry("900x640")

        self.hub: Optional[HubSession] = None
        self.worker: Optional[StreamWorker] = None
        self._snap_lock = threading.Lock()
        self._snapshot = FeedbackSnapshot()
        self._last_plant_block = None

        self._build_top_bar(initial_port)
        self._build_status_strip()
        self._build_tabs()
        self._build_advanced()

        self._refresh_after_id: Optional[str] = None
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._refresh_after_id = self.after(UI_REFRESH_MS, self._refresh_ui)

    # -- layout ---------------------------------------------------------------------

    def _build_top_bar(self, initial_port: Optional[str]) -> None:
        bar = ttk.Frame(self, padding=6)
        bar.pack(fill="x")

        ttk.Label(bar, text="Port:").pack(side="left")
        self.port_var = tk.StringVar(value=initial_port or "")
        self.port_combo = ttk.Combobox(bar, textvariable=self.port_var, width=12, values=self._list_ports())
        self.port_combo.pack(side="left", padx=(2, 6))
        ttk.Button(bar, text="Refresh", command=self._refresh_ports).pack(side="left")

        self.connect_btn = ttk.Button(bar, text="Connect", command=self._on_connect)
        self.connect_btn.pack(side="left", padx=(6, 2))
        self.disconnect_btn = ttk.Button(bar, text="Disconnect", command=self._on_disconnect, state="disabled")
        self.disconnect_btn.pack(side="left")

        ttk.Label(bar, text="  Stream Hz:").pack(side="left", padx=(12, 0))
        self.hz_var = tk.DoubleVar(value=DEFAULT_STREAM_HZ)
        ttk.Spinbox(
            bar, from_=MIN_STREAM_HZ, to=MAX_STREAM_HZ, textvariable=self.hz_var, width=6, increment=10,
        ).pack(side="left")

        ttk.Button(bar, text="Recover", command=self._on_recover).pack(side="left", padx=(12, 2))

        for state in (McuState.NORMAL, McuState.RECOVERY, McuState.ESTOP, McuState.DIAG_ONLY):
            btn = ttk.Button(bar, text=state.name, command=lambda s=state: self._on_mcu_state(s))
            btn.pack(side="left", padx=2)
        tip = ttk.Label(
            self, text="DIAG_ONLY blocks actuator CAN apply entirely (plant_block=DIAG_ONLY) - visibility only.",
            foreground="#8a6d00",
        )
        tip.pack(fill="x", padx=6)

    def _build_status_strip(self) -> None:
        strip = ttk.Frame(self, padding=(6, 2))
        strip.pack(fill="x")
        self.status_var = tk.StringVar(value="disconnected")
        self.status_label = tk.Label(strip, textvariable=self.status_var, anchor="w", font=("TkDefaultFont", 10, "bold"))
        self.status_label.pack(fill="x")
        self.banner_var = tk.StringVar(value="")
        self.banner_label = tk.Label(strip, textvariable=self.banner_var, anchor="w", fg="#c0392b")
        self.banner_label.pack(fill="x")

    def _build_tabs(self) -> None:
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True, padx=6, pady=6)

        act_tab = ttk.Frame(self.notebook)
        self.notebook.add(act_tab, text="Actuators")
        act_notebook = ttk.Notebook(act_tab)
        act_notebook.pack(fill="both", expand=True)
        self.actuator_panels: List[ActuatorPanel] = []
        for slot in range(ACTUATOR_COUNT):
            panel = ActuatorPanel(act_notebook, slot, self._apply_actuator)
            act_notebook.add(panel, text=f"Slot {slot}")
            self.actuator_panels.append(panel)

        servo_tab = ttk.Frame(self.notebook)
        self.notebook.add(servo_tab, text="Servos")
        self.servo_panels: List[ServoPanel] = []
        for slot in (0, 1):
            panel = ServoPanel(servo_tab, slot, self._apply_servo)
            panel.grid(row=0, column=slot, sticky="n", padx=12, pady=8)
            self.servo_panels.append(panel)

        led_tab = ttk.Frame(self.notebook, padding=8)
        self.notebook.add(led_tab, text="LED")
        ttk.Label(led_tab, text="mode (0=test scan, 1=off)").grid(row=0, column=0, sticky="w")
        self.led_mode = tk.IntVar(value=1)
        ttk.Spinbox(led_tab, from_=0, to=31, textvariable=self.led_mode, width=6).grid(row=0, column=1, sticky="w")
        ttk.Label(led_tab, text="brightness (0-31)").grid(row=1, column=0, sticky="w")
        self.led_brightness = tk.IntVar(value=8)
        ttk.Spinbox(led_tab, from_=0, to=31, textvariable=self.led_brightness, width=6).grid(row=1, column=1, sticky="w")
        ttk.Label(led_tab, text="led_count (0=firmware default)").grid(row=2, column=0, sticky="w")
        self.led_count = tk.IntVar(value=0)
        ttk.Spinbox(led_tab, from_=0, to=63, textvariable=self.led_count, width=6).grid(row=2, column=1, sticky="w")
        ttk.Button(led_tab, text="Apply LED", command=self._apply_led).grid(row=3, column=0, pady=(6, 0), sticky="w")

    def _build_advanced(self) -> None:
        frame = ttk.LabelFrame(self, text="Advanced / debug", padding=6)
        frame.pack(fill="both", padx=6, pady=(0, 6))

        stream_row = ttk.Frame(frame)
        stream_row.grid(row=0, column=0, sticky="w")
        self.streaming_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            stream_row,
            text="Stream commands",
            variable=self.streaming_var,
        ).pack(side="left")
        ttk.Button(stream_row, text="Idle all actuators", command=self._idle_all_actuators).pack(
            side="left", padx=(8, 0)
        )
        ttk.Label(
            frame,
            text="Uncheck stream + idle all actuators to demo HOST_STALE "
                 "(non-zero kp/kd/vel/torque bypasses the stale watchdog).",
            foreground="#8a6d00",
            wraplength=860,
            justify="left",
        ).grid(row=1, column=0, sticky="w", pady=(2, 0))

        ttk.Label(frame, text="Last command (hex, first 64 B):").grid(row=2, column=0, sticky="w", pady=(6, 0))
        self.cmd_hex_var = tk.StringVar(value="")
        ttk.Label(frame, textvariable=self.cmd_hex_var, font=("Courier New", 8), wraplength=860).grid(
            row=3, column=0, sticky="w"
        )
        ttk.Label(frame, text="Last command pdu (should be all zero):").grid(row=4, column=0, sticky="w")
        self.pdu_hex_var = tk.StringVar(value="")
        ttk.Label(frame, textvariable=self.pdu_hex_var, font=("Courier New", 8), wraplength=860).grid(
            row=5, column=0, sticky="w"
        )
        ttk.Label(frame, text="Last feedback (hex, first 64 B):").grid(row=6, column=0, sticky="w", pady=(6, 0))
        self.fb_hex_var = tk.StringVar(value="")
        ttk.Label(frame, textvariable=self.fb_hex_var, font=("Courier New", 8), wraplength=860).grid(
            row=7, column=0, sticky="w"
        )

        ttk.Label(frame, text="Status log:").grid(row=8, column=0, sticky="w", pady=(6, 0))
        self.log_text = tk.Text(frame, height=8, width=110, state="disabled", font=("Courier New", 8))
        self.log_text.grid(row=9, column=0, sticky="ew")
        self._last_log_time = 0.0

    # -- port / connect ---------------------------------------------------------------

    @staticmethod
    def _list_ports() -> List[str]:
        return [p.device for p in list_ports.comports()]

    def _refresh_ports(self) -> None:
        self.port_combo["values"] = self._list_ports()

    def _on_connect(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("No port", "Select a serial port first.")
            return
        try:
            hub = HubSession(port, baud=DEFAULT_BAUD)
            hub.open()
        except Exception as exc:
            messagebox.showerror("Connect failed", str(exc))
            return
        self.hub = hub
        self.worker = StreamWorker(
            hub,
            get_hz=lambda: self.hz_var.get(),
            get_streaming_enabled=lambda: self.streaming_var.get(),
            on_snapshot=self._on_snapshot,
            on_error=self._on_worker_error,
        )
        self.worker.start()
        self.connect_btn["state"] = "disabled"
        self.disconnect_btn["state"] = "normal"
        self.port_combo["state"] = "disabled"
        self._append_log(f"connected {port}")

    def _on_disconnect(self) -> None:
        self._stop_worker()
        if self.hub is not None:
            self.hub.close()
        self.hub = None
        self.connect_btn["state"] = "normal"
        self.disconnect_btn["state"] = "disabled"
        self.port_combo["state"] = "normal"
        with self._snap_lock:
            self._snapshot = FeedbackSnapshot()
        self._append_log("disconnected")

    def _stop_worker(self) -> None:
        if self.worker is not None:
            self.worker.stop()
            self.worker.join(timeout=1.5)
            self.worker = None

    def _on_worker_error(self, exc: Exception) -> None:
        msg = f"stream error: {exc}"
        self.after(0, lambda m=msg: self._append_log(m))

    def _on_close(self) -> None:
        if self._refresh_after_id is not None:
            self.after_cancel(self._refresh_after_id)
            self._refresh_after_id = None
        self._stop_worker()
        if self.hub is not None:
            self.hub.close()
        self.destroy()

    # -- commanding ---------------------------------------------------------------

    def _apply_actuator(self, slot: int, desire: ActuatorDesire) -> None:
        if self.hub is None:
            messagebox.showwarning("Not connected", "Connect to a port first.")
            return
        self.hub.set_actuator(slot, desire, send=False)

    def _apply_servo(self, slot: int, payload: dict) -> None:
        if self.hub is None:
            messagebox.showwarning("Not connected", "Connect to a port first.")
            return
        self.hub.set_servo(slot, payload["position"], servo_id=payload["servo_id"],
                            speed=payload["speed"], torque_enable=payload["torque_enable"], send=False)

    def _apply_led(self) -> None:
        if self.hub is None:
            messagebox.showwarning("Not connected", "Connect to a port first.")
            return
        self.hub.set_led(self.led_mode.get(), self.led_brightness.get(), self.led_count.get(), send=False)

    def _idle_all_actuators(self) -> None:
        if self.hub is None:
            messagebox.showwarning("Not connected", "Connect to a port first.")
            return
        for slot in range(ACTUATOR_COUNT):
            self.hub.set_actuator(slot, ActuatorDesire(), send=False)
        for panel in self.actuator_panels:
            for var in panel.vars.values():
                var.set("0.0")
        self._append_log("all actuators idled (held desires cleared)")

    def _on_recover(self) -> None:
        if self.hub is None:
            return
        self.hub.recover()
        self._append_log("recover: RECOVERY -> NORMAL")

    def _on_mcu_state(self, state: McuState) -> None:
        if self.hub is None:
            messagebox.showwarning("Not connected", "Connect to a port first.")
            return
        self.hub.set_mcu_state(state)
        self._append_log(f"mcu_state -> {state.name}")

    # -- snapshot / UI refresh -----------------------------------------------------

    def _on_snapshot(self, snap: FeedbackSnapshot) -> None:
        with self._snap_lock:
            self._snapshot = snap

    def _refresh_ui(self) -> None:
        with self._snap_lock:
            snap = self._snapshot

        if not snap.connected:
            self.status_var.set("disconnected")
            self.status_label.configure(fg="#666666")
            self.banner_var.set("")
        else:
            block = snap.plant_block
            block_name = _block_name(block)
            color = PLANT_BLOCK_COLOR.get(block, _DEFAULT_BLOCK_COLOR)
            if snap.mcu_state == int(McuState.ESTOP):
                color = "#c0392b"
            self.status_label.configure(fg=color)
            mismatch = ""
            if snap.cmd_seq_low is not None and snap.cmd_seq_low != snap.ack_seq:
                mismatch = f"  (cmd_seq_low={snap.cmd_seq_low} != ack_seq)"
            self.status_var.set(
                f"plant_block={block_name}  tick={snap.tick}  ack_seq={snap.ack_seq}  "
                f"mcu_state={_mcu_state_name(snap.mcu_state)}  "
                f"fb_hz={snap.fb_hz:.1f}  cmd_hz={snap.cmd_hz:.1f}{mismatch}"
            )
            if block == PlantBlockReason.HOST_STALE:
                self.banner_var.set(
                    "HOST_STALE: no fresh command in >500 ms and no non-idle desire held - actuator CAN apply is blocked."
                )
            elif block == PlantBlockReason.DIAG_ONLY:
                self.banner_var.set("DIAG_ONLY: actuator CAN apply is blocked while mcu_state=DIAG_ONLY.")
            elif isinstance(block, PlantBlockReason) and block != PlantBlockReason.NONE:
                self.banner_var.set(f"plant_block={block_name}: actuator CAN apply is blocked.")
            elif snap.last_fb_age_s > 1.0:
                self.banner_var.set(f"No feedback frame in {snap.last_fb_age_s:.1f}s.")
            else:
                self.banner_var.set("")

            if block != self._last_plant_block:
                self._append_log(f"plant_block -> {block_name}")
                self._last_plant_block = block

        for slot, panel in enumerate(self.actuator_panels):
            panel.update_feedback(snap.actuators[slot] if slot < len(snap.actuators) else None)
        for slot, panel in enumerate(self.servo_panels):
            panel.update_feedback(snap.servos[slot] if slot < len(snap.servos) else None)

        if snap.last_cmd_bytes:
            self.cmd_hex_var.set(snap.last_cmd_bytes[:64].hex(" "))
            self.pdu_hex_var.set(snap.last_cmd_bytes[PDU_OFF : PDU_OFF + 32].hex(" "))
        if snap.last_fb_bytes:
            self.fb_hex_var.set(snap.last_fb_bytes[:64].hex(" "))

        now = time.monotonic()
        if snap.connected and now - self._last_log_time > 1.0:
            self._last_log_time = now
            self._append_log(
                f"tick={snap.tick} ack={snap.ack_seq} plant_block={_block_name(snap.plant_block)} "
                f"fb_hz={snap.fb_hz:.1f}"
            )

        self._refresh_after_id = self.after(UI_REFRESH_MS, self._refresh_ui)

    def _append_log(self, line: str) -> None:
        self.log_text["state"] = "normal"
        self.log_text.insert("end", f"{time.strftime('%H:%M:%S')}  {line}\n")
        self.log_text.see("end")
        # keep last ~100 lines
        n_lines = int(self.log_text.index("end-1c").split(".")[0])
        if n_lines > 100:
            self.log_text.delete("1.0", f"{n_lines - 100}.0")
        self.log_text["state"] = "disabled"


def _default_port() -> Optional[str]:
    ports = list(list_ports.comports())
    stm32 = [p for p in ports if p.vid == STM32_VID]
    if len(stm32) == 1:
        return stm32[0].device
    return ports[0].device if ports else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=None, help="Serial port to preselect (e.g. COM5)")
    args = parser.parse_args()

    app = DashboardApp(initial_port=args.port or _default_port())
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
