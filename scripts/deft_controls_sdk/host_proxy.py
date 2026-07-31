"""HostProxy — platform demux on top of ControlsPcbHub.

One COM owner. Apps (pcb_lab, vbeta, i2rt bridge) talk in profile *components*
(left_arm, base, …), not raw slot indexes. Does not import vbeta.

Demux is **not** runtime auto-detect. Two layers decide where packets go:

1. **Profile (``config.Profile``, at HostProxy construction)** — name → slots.
   ``proxy.actuators("left_arm")`` / ``proxy.component(...)`` returns
   ``actions.ActuatorAction``.
2. **CFG (MCU flash, via hub.debug / ensure_*_cfg)** — each slot's
   ``{bus, protocol, motor_id, master_id, enabled}``.

Profiles / slot maps live in ``deft_controls_sdk.config``; plant commands in
``deft_controls_sdk.actions``. This module re-exports slot constants for
back-compat.
"""
from __future__ import annotations

import time
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from deft_controls_sdk.actions import (
    ActuatorAction,
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
    Profile,
    RIGHT_ARM_SLOTS,
    SPARE_SLOTS,
    bench_continuous_profile,
    yam_product_profile,
)
from deft_controls_sdk.controls_pcb_hub import ControlsPcbHub
from deft_controls_sdk.link import ActuatorDesire, FeedbackImage, LedDesire, McuState, ServoDesire
from deft_controls_sdk.link.api_types import infer_effective_led
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT, DEFAULT_BAUD

__all__ = [
    "ActuatorAction",
    "BASE_DRIVE_SLOTS",
    "BASE_SLOTS",
    "BASE_STEER_SLOTS",
    "BENCH_BASE_SLOTS",
    "HostProxy",
    "LEFT_ARM_SLOTS",
    "LIFT_SLOT",
    "NECK_PITCH_SERVO_SLOT",
    "NECK_YAW_SERVO_SLOT",
    "Profile",
    "RIGHT_ARM_SLOTS",
    "SPARE_SLOTS",
    "bench_continuous_profile",
    "yam_product_profile",
]


class HostProxy:
    """One process, one COM — component MIT API over ControlsPcbHub."""

    def __init__(
        self,
        hub: ControlsPcbHub,
        *,
        profile: Optional[Profile] = None,
        owns_hub: bool = True,
        listen_pdu: bool = False,
        telemetry_hz: float = 200.0,
    ) -> None:
        self._hub = hub
        self._owns_hub = owns_hub
        self._profile = profile or yam_product_profile()
        self._stream_hz = 200.0
        self._telemetry_hz = float(telemetry_hz)
        self._closed = False
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
        profile: Optional[Profile] = None,
        idle_first: bool = False,
        persist_telemetry: bool = False,
        apply_yam_cfg: bool = False,
        force_cfg: bool = False,
        listen_pdu: bool = False,
        mode: str = "bandwidth",
    ) -> "HostProxy":
        """Connect. ``listen_pdu`` default False (bench): ignore stale PDU kill.

        ``mode`` defaults to ``bandwidth`` (timing-safe, no hub.debug RPC).
        Pass ``mode="debug"`` for CFG / discover / pcb_lab.debug.

        Pass ``listen_pdu=True`` when a real PDB/PDU peer is on UART4 so
        soft-kill park + status LED traffic-light use kill bytes. MCU may
        still publish HARD+COMMS_LOSS when the peer is absent — that is
        firmware stale failsafe, not host policy.
        """
        hub = ControlsPcbHub.connect(
            port,
            serial=serial,
            baud=baud,
            persist_telemetry=persist_telemetry,
            mode=mode,
        )
        proxy = cls(
            hub,
            profile=profile,
            owns_hub=True,
            listen_pdu=listen_pdu,
            telemetry_hz=telemetry_hz,
        )
        proxy._stream_hz = float(stream_hz)
        if idle_first:
            # Idle-anchor (p=1e-6, kp=0), not true blank p=0: keeps every bus
            # (incl. CH4-6 / MCP) in the plant apply path when only some slots
            # are actively held. True blank on an uncommanded bus is still
            # skipped by the shared blank-bus policy (same for FDCAN and MCP).
            anchored = {
                s: ActuatorDesire(position=1e-6) for s in range(ACTUATOR_COUNT)
            }
            hub.set_mcu_state(McuState.NORMAL, send=False)
            hub.set_plant_apply(False, send=False)
            proxy.set_actuators(anchored, send=False)
            # follow (wire mode 0): MCU uses NVM default / listen_pdu traffic-light.
            # Do not force debug cornflower — that masks listen_pdu LED policy.
            hub.set_led(
                LedDesire(mode="follow", master_brightness=8),
                send=False,
            )
            hub.send_once()
        else:
            hub.recover()
        if apply_yam_cfg:
            # Late import: CFG helpers live in vbeta; optional product path only.
            from deft_controls_sdk.vbeta.cfg import ensure_yam_product_cfg

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
        listen_pdu: Optional[bool] = None,
    ) -> "HostProxy":
        """Use an existing hub (tests / caller already owns COM)."""
        proxy = cls(
            hub,
            profile=profile,
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
        # Refresh soft-kill hook to match.
        if self._hub.is_streaming:
            self._hub.set_auto_soft_kill(bool(enabled))

    @property
    def hub(self) -> ControlsPcbHub:
        return self._hub

    @property
    def profile(self) -> Profile:
        return self._profile

    def actuators(self, name: str) -> ActuatorAction:
        """Named profile actuator group — ``actions.ActuatorAction``."""
        return ActuatorAction(self, self._profile, name)

    def component(self, name: str) -> ActuatorAction:
        """Alias of :meth:`actuators` (profile demux name → slot group)."""
        return self.actuators(name)

    def led(self) -> LedAction:
        return LedAction(self)

    def servo(self) -> ServoAction:
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
                from deft_controls_sdk.vbeta.cfg import pause_plant_stream

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
        self.service_soft_kill()
        self._hub.send_once()

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
