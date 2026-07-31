"""HostProxy — one COM session over ControlsPcbHub.

Product path (deft_vbeta)::

    connect  → open USB + stream (default assembly = yam_product)
    arm      → plant_apply ON
    command  → set_section(name, ActuatorDesire…)  # vbeta authors gains
    disarm   → plant_apply OFF
    close    → blank + release COM

Lab / notebooks may also use ``proxy.actions`` (mount → apply → clear).

Link ``mode`` (fixed for the session — reconnect to change)::

    bandwidth — plant stream / motion / LED. No hub.debug (CFG/discover/cal).
    debug     — plant + hub.debug RPC (inventory, CFG, discover, cal).

Default demux is the static YAM product assembly (left/right arm, base
wheels 1–3, torso). Pass ``assembly=`` / ``profile=`` only for lab variants.
"""
from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any, Dict, List, Mapping, Optional, Sequence, Tuple

from deft_controls_sdk.actions import (
    ActuatorAction,
    Actions,
    LedAction,
    PduLinkAction,
    ServoAction,
)
from deft_controls_sdk.config import (
    BASE_DRIVE_SLOTS,
    BASE_SLOTS,
    BASE_STEER_SLOTS,
    BENCH_BASE_SLOTS,
    LEFT_ARM_SLOTS,
    LIFT_SLOT,
    NECK_PITCH_SERVO_SLOT,
    NECK_YAW_SERVO_SLOT,
    PRODUCT_ACTUATOR_SECTIONS,
    Profile,
    RIGHT_ARM_SLOTS,
    SPARE_SLOTS,
    TORSO_SLOT,
    bench_continuous_profile,
    ensure_yam_product_cfg,
    yam_product_assembly,
    yam_product_profile,
)
from deft_controls_sdk.controls_pcb_hub import ControlsPcbHub
from deft_controls_sdk.link import (
    ActuatorDesire,
    FeedbackImage,
    FeedbackState,
    LedDesire,
    McuState,
    ServoDesire,
)
from deft_controls_sdk.link.api_types import infer_effective_led
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT, DEFAULT_BAUD

if TYPE_CHECKING:
    from deft_controls_sdk.config.assembly import Assembly
    from deft_controls_sdk.telemetry import TelemetryCache

__all__ = [
    "ActuatorAction",
    "Actions",
    "BASE_DRIVE_SLOTS",
    "BASE_SLOTS",
    "BASE_STEER_SLOTS",
    "BENCH_BASE_SLOTS",
    "HostProxy",
    "LEFT_ARM_SLOTS",
    "LIFT_SLOT",
    "NECK_PITCH_SERVO_SLOT",
    "NECK_YAW_SERVO_SLOT",
    "PRODUCT_ACTUATOR_SECTIONS",
    "Profile",
    "RIGHT_ARM_SLOTS",
    "SPARE_SLOTS",
    "TORSO_SLOT",
    "bench_continuous_profile",
    "yam_product_profile",
]


class HostProxy:
    """One process, one COM — plant session API over ControlsPcbHub."""

    def __init__(
        self,
        hub: ControlsPcbHub,
        *,
        profile: Optional[Profile] = None,
        assembly: Optional["Assembly"] = None,
        owns_hub: bool = True,
        listen_pdu: bool = False,
        telemetry_hz: float = 200.0,
    ) -> None:
        self._hub = hub
        self._owns_hub = owns_hub
        if assembly is None and profile is None:
            assembly = yam_product_assembly()
        self._assembly = assembly
        if assembly is not None and profile is None:
            profile = assembly.to_demux_profile()
        self._profile = profile or yam_product_profile()
        self._stream_hz = 200.0
        self._telemetry_hz = float(telemetry_hz)
        self._closed = False
        self._actions: Optional[Actions] = None
        self._hub.listen_pdu = bool(listen_pdu)

    @classmethod
    def connect(
        cls,
        port: Optional[str] = None,
        *,
        serial: Optional[str] = None,
        baud: int = DEFAULT_BAUD,
        stream_hz: float = 200.0,
        telemetry_hz: float = 200.0,
        assembly: Optional["Assembly"] = None,
        profile: Optional[Profile] = None,
        armed: Optional[bool] = None,
        idle_first: Optional[bool] = None,
        persist_telemetry: bool = False,
        telemetry: Optional["TelemetryCache"] = None,
        apply_yam_cfg: bool = False,
        force_cfg: bool = False,
        listen_pdu: bool = False,
        mode: str = "bandwidth",
    ) -> "HostProxy":
        """Open USB and start the plant stream.

        Parameters
        ----------
        mode:
            ``"bandwidth"`` — motion / timing only (no CFG/discover).
            ``"debug"`` — motion + ``hub.debug`` (CFG, inventory, cal).
        assembly:
            Demux map. Default ``yam_product_assembly()`` when both
            ``assembly`` and ``profile`` are omitted. Lab: pass
            ``assembly_from_name("bench")``.
        armed:
            ``False`` (recommended for bringup) — stream with plant_apply OFF;
            call :meth:`arm_plant` before motors should track.
            ``True`` — start with plant apply armed (product teleop style).
            Default ``None`` keeps legacy behaviour via ``idle_first``.
        idle_first:
            Legacy alias: ``True`` ≡ ``armed=False``. Ignored when ``armed=``
            is set.
        telemetry:
            Optional shared ``TelemetryCache`` (e.g. debug_dashboard process
            cache). When omitted, the hub creates one (honouring
            ``persist_telemetry``).
        listen_pdu:
            ``True`` when a real PDU peer is present (soft-kill / LED policy).
        """
        if assembly is None and profile is None:
            assembly = yam_product_assembly()
        if assembly is not None and profile is None:
            profile = assembly.to_demux_profile()

        # Resolve armed vs idle_first (armed wins when set).
        if armed is not None:
            start_disarmed = not bool(armed)
        elif idle_first is not None:
            start_disarmed = bool(idle_first)
        else:
            # Legacy default: recover() path (plant typically armed).
            start_disarmed = False

        hub = ControlsPcbHub.connect(
            port,
            serial=serial,
            baud=baud,
            persist_telemetry=persist_telemetry,
            telemetry=telemetry,
            mode=mode,
        )
        proxy = cls(
            hub,
            profile=profile,
            assembly=assembly,
            owns_hub=True,
            listen_pdu=listen_pdu,
            telemetry_hz=telemetry_hz,
        )
        proxy._stream_hz = float(stream_hz)
        if start_disarmed:
            # Idle-anchor (p=1e-6, kp=0), not true blank p=0: keeps every bus
            # (incl. CH4-6 / MCP) in the plant apply path when only some slots
            # are actively held.
            anchored = {
                s: ActuatorDesire(position=1e-6) for s in range(ACTUATOR_COUNT)
            }
            hub.set_mcu_state(McuState.NORMAL, send=False)
            hub.set_plant_apply(False, send=False)
            proxy.set_actuators(anchored, send=False)
            hub.set_led(
                LedDesire(mode="follow", master_brightness=8),
                send=False,
            )
            hub.send_once()
        else:
            hub.recover()
        if apply_yam_cfg:
            if hub.link_mode != "debug":
                raise RuntimeError(
                    "apply_yam_cfg requires mode='debug' "
                    f"(connected as mode={hub.link_mode!r})"
                )
            ensure_yam_product_cfg(hub, force=force_cfg)
        hub.start_streaming(
            hz=proxy._stream_hz,
            telemetry_hz=proxy._telemetry_hz,
            auto_soft_kill=bool(listen_pdu),
        )
        return proxy

    @classmethod
    def wrap(
        cls,
        hub: ControlsPcbHub,
        *,
        stream_hz: float = 200.0,
        telemetry_hz: float = 200.0,
        profile: Optional[Profile] = None,
        assembly: Optional["Assembly"] = None,
        listen_pdu: Optional[bool] = None,
    ) -> "HostProxy":
        """Use an existing hub (tests / caller already owns COM)."""
        proxy = cls(
            hub,
            profile=profile,
            assembly=assembly,
            owns_hub=False,
            listen_pdu=hub.listen_pdu if listen_pdu is None else bool(listen_pdu),
            telemetry_hz=telemetry_hz,
        )
        proxy._stream_hz = float(stream_hz)
        if not hub.is_streaming:
            hub.start_streaming(
                hz=proxy._stream_hz,
                telemetry_hz=proxy._telemetry_hz,
                auto_soft_kill=proxy._hub.listen_pdu,
            )
        return proxy

    @property
    def listen_pdu(self) -> bool:
        return self._hub.listen_pdu

    @listen_pdu.setter
    def listen_pdu(self, enabled: bool) -> None:
        self._hub.listen_pdu = bool(enabled)
        if self._hub.is_streaming:
            self._hub.set_auto_soft_kill(bool(enabled))

    @property
    def hub(self) -> ControlsPcbHub:
        return self._hub

    @property
    def mode(self) -> str:
        """Link mode for this session: ``\"bandwidth\"`` or ``\"debug\"``."""
        return str(self._hub.link_mode)

    @property
    def armed(self) -> bool:
        """True when plant_apply is ON (motors track held desires)."""
        return bool(self._hub.plant_apply)

    @property
    def assembly(self) -> Optional["Assembly"]:
        """Assembly passed at connect, if any."""
        return self._assembly

    @property
    def profile(self) -> Profile:
        return self._profile

    def require_debug(self, op: str) -> None:
        """Raise if this session is not ``mode=\"debug\"`` (CFG/discover/cal)."""
        if self.mode != "debug":
            raise RuntimeError(
                f"{op} needs HostProxy.connect(..., mode='debug'); "
                f"this session is mode={self.mode!r}"
            )

    def arm_plant(self) -> None:
        """Arm plant apply — motors track held desires. Works in either link mode."""
        self._hub.set_mcu_state(McuState.NORMAL, send=False)
        self._hub.set_plant_apply(True, send=False)
        self.commit()

    def disarm_plant(self) -> None:
        """Disarm plant apply — desires may still be held, motors ignore them."""
        self._hub.set_mcu_state(McuState.NORMAL, send=False)
        self._hub.set_plant_apply(False, send=False)
        self.commit()

    def ensure_plant_control(self, *, enable: bool = True) -> None:
        """Deprecated alias of :meth:`arm_plant` / :meth:`disarm_plant`."""
        if enable:
            self.arm_plant()
        else:
            self.disarm_plant()

    def commit(self) -> None:
        """Push the currently held desires / system bits once (alias of send_once).

        Prefer ``proxy.actions.apply()`` for command batches. ``commit()`` is
        the low-level TX for held hub desires. Streaming also keeps sending
        whatever is held each tick.
        """
        self.send_once()

    @property
    def actions(self) -> Actions:
        """Session command batch: ``mount`` → ``apply`` → ``clear``."""
        if self._actions is None:
            self._actions = Actions(self)
        return self._actions

    def actuators(self, name: str) -> ActuatorAction:
        """Named assembly/profile group → unbound ``ActuatorAction`` (legacy).

        Prefer ``proxy.actions.actuator(name=…)`` for mount/apply batches.
        """
        if self._assembly is not None and name in self._assembly.actuators:
            return ActuatorAction.from_actuator_profile(
                self, self._assembly.actuator(name)
            )
        return ActuatorAction(self, self._profile, name)

    def component(self, name: str) -> ActuatorAction:
        """Alias of :meth:`actuators`."""
        return self.actuators(name)

    def led(self) -> LedAction:
        """Unbound LED helper (legacy). Prefer ``proxy.actions.led()``."""
        return LedAction(self)

    def servo(self, name: str = "neck") -> ServoAction:
        """Unbound servo helper (legacy). Prefer ``proxy.actions.servo()``."""
        if self._assembly is not None and name in self._assembly.servos:
            return ServoAction.from_servo_profile(
                self, self._assembly.servo(name)
            )
        return ServoAction(self)

    def pdu_link(self) -> PduLinkAction:
        return PduLinkAction(self)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            blank = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
            self.set_actuators(blank, send=False)
            self._hub.set_mcu_state(McuState.NORMAL, send=False)
            self._hub.set_plant_apply(False, send=False)
            # Do not overwrite LedDesire here — ``set --led-mode`` / debug
            # patterns must survive process exit until the next host session.
            # Soft-DFU post_flash stages listen_pdu + follow explicitly.
            self._hub.send_once()
            if not self._hub.is_streaming:
                self._hub.start_streaming(hz=5.0)
            time.sleep(0.25)
            self._hub.stop_streaming()
        finally:
            if self._owns_hub:
                self._hub.close()

    def __enter__(self) -> "HostProxy":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def set_actuator(self, slot: int, desire: ActuatorDesire, *, send: bool = False) -> None:
        self._hub.set_actuator(slot, desire, send=send)

    def set_actuators(self, desires: Mapping[int, ActuatorDesire], *, send: bool = False) -> None:
        conn = self._hub._connection  # noqa: SLF001 — batch hold update
        conn.set_actuators(desires, send=send)

    def section_slots(self, name: str) -> Tuple[int, ...]:
        """Ordered plant slots for an actuator demux section."""
        return self._profile.slots(name)

    def set_section(
        self,
        name: str,
        desires: Sequence[ActuatorDesire],
        *,
        send: bool = False,
    ) -> None:
        """Merge an ordered desire list into the held CMDH image for ``name``.

        Product path: vbeta authors ``ActuatorDesire`` (p/v/kp/kd/τ); HostProxy
        only demuxes onto fixed section slots. Length must match the section.
        """
        slots = self.section_slots(name)
        if len(desires) != len(slots):
            raise ValueError(
                f"section {name!r} expects {len(slots)} desires, got {len(desires)}"
            )
        self.set_actuators(
            {int(slot): desire for slot, desire in zip(slots, desires)},
            send=send,
        )

    def section_feedback(self, name: str) -> List[Optional[FeedbackState]]:
        """Latest actuator feedback aligned to :meth:`section_slots` order."""
        fb = self.latest_feedback()
        slots = self.section_slots(name)
        if fb is None:
            return [None] * len(slots)
        return [fb.actuator(int(slot)) for slot in slots]

    def set_servo_section(
        self,
        name: str,
        desires: Sequence[ServoDesire],
        *,
        send: bool = False,
    ) -> None:
        """Merge ordered servo desires for a named servo section (e.g. ``neck``)."""
        if self._assembly is not None and name in self._assembly.servos:
            slots = self._assembly.servo(name).slots
        elif name == "neck":
            slots = (NECK_PITCH_SERVO_SLOT, NECK_YAW_SERVO_SLOT)
        else:
            raise KeyError(f"unknown servo section {name!r}")
        if len(desires) != len(slots):
            raise ValueError(
                f"servo section {name!r} expects {len(slots)} desires, got {len(desires)}"
            )
        for slot, desire in zip(slots, desires):
            self.set_servo(int(slot), desire, send=False)
        if send:
            self.send_once()

    def set_servo(self, slot: int, desire: ServoDesire, *, send: bool = False) -> None:
        self._hub.set_servo(slot, desire, send=send)

    def set_led(self, desire: LedDesire, *, send: bool = False) -> None:
        """Apply LedDesire policy.

        ``pdu`` forces host + MCU ``listen_pdu``. ``follow`` / ``debug`` leave
        the NVM bit alone (``follow`` uses whatever is already set).
        """
        if desire.mode == "pdu":
            self.listen_pdu = True
            try:
                from deft_controls_sdk.debug.stream_pause import pause_plant_stream

                with pause_plant_stream(self._hub):
                    periph = self._hub.debug.cfg_get_periph()
                    periph["listen_pdu"] = True
                    self._hub.debug.cfg_set_periph(periph, persist=False)
            except Exception:  # noqa: BLE001 — LED still applies if CFG unavailable
                pass
        self._hub.set_led(desire, send=send)

    def service_soft_kill(self) -> bool:
        fn = getattr(self._hub, "soft_kill_park_if_requested", None)
        if fn is None:
            return False
        return bool(fn(send=False))

    def send_once(self) -> None:
        """Low-level TX of held plant frame. Prefer :meth:`commit` in notebooks."""
        self.service_soft_kill()
        self._hub.send_once()

    def cfg_snapshot(self) -> Dict[str, object]:
        """Live RAM CFG + periph. Requires ``mode=\"debug\"``."""
        self.require_debug("cfg_snapshot")
        from deft_controls_sdk.debug.snapshot import collect_cfg

        return collect_cfg(self)

    def poll_feedback(self) -> Optional[FeedbackImage]:
        return self._hub._connection.poll_feedback()  # noqa: SLF001

    def latest_feedback(self) -> Optional[FeedbackImage]:
        fb = self.poll_feedback()
        if fb is not None:
            return fb
        raw = self._hub._connection._latest_fb_raw  # noqa: SLF001
        return FeedbackImage(raw) if raw is not None else None

    def sleep(self, seconds: float) -> None:
        time.sleep(seconds)

    def demux_report(self) -> Dict[str, object]:
        """Profile → slots joined with CFG rows (when DEBUG allows).

        Shows both demux layers. CFG get often times out while streaming —
        pause stream or expect ``cfg_ok=False`` during plant.
        """
        report: Dict[str, object] = {
            "profile": self._profile.name,
            "layers": {
                "1_profile": "name → actuator slots (host, at connect)",
                "2_cfg": "slot → bus/protocol/motor_id (MCU flash)",
                "servo": "neck DXL uses servo slots 0/1, not Profile",
            },
            "components": {k: list(v) for k, v in self._profile.components.items()},
            "servo_slots": {
                "neck_pitch": NECK_PITCH_SERVO_SLOT,
                "neck_yaw": NECK_YAW_SERVO_SLOT,
            },
            "streaming": bool(self._hub.is_streaming),
            "port": self._hub.port,
        }
        table = None
        try:
            table = self._hub.debug.cfg_get_table()
            report["cfg_ok"] = table is not None
            report["cfg_slots"] = len(table) if table is not None else 0
        except Exception as exc:  # noqa: BLE001 — report must not raise
            report["cfg_ok"] = False
            report["cfg_error"] = str(exc)

        by_component: Dict[str, list] = {}
        for name, slots in self._profile.components.items():
            rows = []
            for slot in slots:
                row: Dict[str, object] = {"slot": int(slot)}
                if table is not None and 0 <= slot < len(table) and table[slot] is not None:
                    cfg = table[slot]
                    # CFG row shape varies; expose common attrs if present.
                    for key in ("bus", "protocol", "motor_id", "master_id", "enabled"):
                        if hasattr(cfg, key):
                            row[key] = getattr(cfg, key)
                        elif isinstance(cfg, Mapping) and key in cfg:
                            row[key] = cfg[key]
                rows.append(row)
            by_component[name] = rows
        report["by_component"] = by_component
        return report

    def doctor(self) -> Dict[str, object]:
        """Health snapshot: demux + stream + MCU/PDB/LED traffic-light.

        ``listen_pdu`` (on-connect / ``proxy.listen_pdu``) gates whether PDB
        kill bytes drive soft-kill + LED inference. When False, wire may still
        show HARD+COMMS_LOSS under ``pdb.wire`` — that is MCU stale failsafe.
        """
        hub_stream = getattr(self._hub, "stream_hz", None)
        hub_telem = getattr(self._hub, "telemetry_hz", None)
        report: Dict[str, object] = {
            "profile": self._profile.name,
            "components": {k: list(v) for k, v in self._profile.components.items()},
            "port": self._hub.port,
            "stream": {
                "streaming": bool(self._hub.is_streaming),
                "hz": float(hub_stream) if hub_stream is not None else float(self._stream_hz),
                "telemetry_hz": (
                    float(hub_telem)
                    if hub_telem is not None
                    else float(self._telemetry_hz)
                ),
            },
            # Back-compat shorthand (prefer ``stream.streaming``).
            "streaming": bool(self._hub.is_streaming),
            "listen_pdu": bool(getattr(self._hub, "listen_pdu", False)),
        }
        try:
            table = self._hub.debug.cfg_get_table()
            report["cfg_slots"] = len(table) if table is not None else 0
            report["cfg_ok"] = table is not None
        except Exception as exc:  # noqa: BLE001 — doctor must not raise
            report["cfg_ok"] = False
            report["cfg_error"] = str(exc)

        host_mcu = getattr(self._hub, "mcu_state", None)
        if host_mcu is not None:
            report["mcu"] = {
                "host_command": int(host_mcu),
                "host_command_name": McuState(host_mcu).name,
            }
        else:
            report["mcu"] = {}

        fb = self.latest_feedback()
        report["has_feedback"] = fb is not None
        if fb is not None:
            try:
                fb_mcu = int(fb.mcu_state)
                report["mcu"]["feedback"] = fb_mcu
                report["mcu"]["feedback_name"] = McuState(fb_mcu).name
            except Exception:  # noqa: BLE001
                report["mcu"]["feedback"] = getattr(fb, "mcu_state", None)

        listening = bool(getattr(self._hub, "listen_pdu", False))
        kill_state: Optional[int] = None
        estop_sense: Optional[int] = None
        pdb_fn = getattr(self._hub, "pdb_status", None)
        wire: Optional[Dict[str, object]] = None
        if callable(pdb_fn):
            try:
                pdb = pdb_fn()
            except Exception as exc:  # noqa: BLE001
                report["pdb"] = {"listening": listening, "ok": False, "error": str(exc)}
                pdb = None
            if pdb is not None:
                wire = {
                    "ok": True,
                    "kill_state": int(pdb.kill_state),
                    "kill_state_name": pdb.kill_state_name,
                    "kill_reason": int(pdb.kill_reason),
                    "kill_reason_name": pdb.kill_reason_name,
                    "estop_sense": int(pdb.estop_sense),
                    "stale_failsafe": bool(pdb.stale_failsafe),
                }
                if listening:
                    kill_state = int(pdb.kill_state)
                    estop_sense = int(pdb.estop_sense)
                    report["pdb"] = {"listening": True, **wire}
                else:
                    report["pdb"] = {
                        "listening": False,
                        "note": (
                            "host listen_pdu=False — kill ignored for soft-kill/LED; "
                            "MCU may still publish stale HARD+COMMS_LOSS on the wire"
                        ),
                        "wire": wire,
                    }
            elif "pdb" not in report:
                report["pdb"] = {
                    "listening": listening,
                    "ok": False,
                    "note": "no plant FB with kill bytes yet",
                }

        host_led = getattr(self._hub, "led_desire", None)
        report["led"] = infer_effective_led(
            host_led=host_led,
            listen_pdu=listening,
            kill_state=kill_state,
            estop_sense=estop_sense,
        )
        if host_led is not None:
            report["led"]["host_brightness"] = int(host_led.master_brightness)
            report["led"]["host_pattern"] = int(host_led.pattern)
        return report
