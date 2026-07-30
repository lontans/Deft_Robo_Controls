"""HostProxy — platform demux on top of ControlsPcbHub.

One COM owner. Apps (pcb_lab, vbeta, i2rt bridge) talk in *components*
(left_arm, base, …), not raw slot indexes. Does not import vbeta.

Demux is **not** runtime auto-detect. Two layers decide where packets go:

1. **Profile (host, at HostProxy construction)** — name → ordered actuator
   slots. Default ``yam_product_profile()``. Callers then use
   ``proxy.component("left_arm")``; that only packs desires into those slots.
2. **CFG (MCU flash, via hub.debug / ensure_*_cfg)** — each slot's
   ``{bus, protocol, motor_id, master_id, enabled}``. Plugins on the board
   use CFG every control tick; HostProxy never invents IDs.

Neck DXL uses **servo** slots (0/1), outside ``Profile.components``.
Bench continuous maps CH5/CH6 motors onto spare actuator slots 22–25
(``bench_continuous_profile``); product ``base`` remains slots 14–19.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from deft_controls_sdk.controls_pcb_hub import ControlsPcbHub
from deft_controls_sdk.link import ActuatorDesire, FeedbackImage, LedDesire, McuState, ServoDesire
from deft_controls_sdk.link.api_types import infer_effective_led
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT, DEFAULT_BAUD

# --- product slot map (platform truth; vbeta.slots re-exports) ---------------

LEFT_ARM_SLOTS: Tuple[int, ...] = tuple(range(0, 7))
RIGHT_ARM_SLOTS: Tuple[int, ...] = tuple(range(7, 14))
BASE_STEER_SLOTS: Dict[str, int] = {"BwC": 14, "BwR": 15, "BwL": 16}
BASE_DRIVE_SLOTS: Dict[str, int] = {"BpC": 17, "BpR": 18, "BpL": 19}
BASE_SLOTS: Tuple[int, ...] = (14, 15, 16, 17, 18, 19)
LIFT_SLOT = 20
SPARE_SLOTS: Tuple[int, ...] = (21, 22, 23, 24, 25)
# Continuous / bus56 lab: CH5+CH6 motors on spare slots (not product base 14–19).
BENCH_BASE_SLOTS: Tuple[int, ...] = (22, 23, 24, 25)
NECK_PITCH_SERVO_SLOT = 0
NECK_YAW_SERVO_SLOT = 1


@dataclass(frozen=True)
class Profile:
    """Named groups of actuator slots (host demux layer 1)."""

    name: str
    components: Mapping[str, Tuple[int, ...]]

    def slots(self, component: str) -> Tuple[int, ...]:
        try:
            return self.components[component]
        except KeyError as exc:
            known = ", ".join(sorted(self.components))
            raise KeyError(f"unknown component {component!r}; known: {known}") from exc

    def all_slots(self) -> Tuple[int, ...]:
        seen: List[int] = []
        for slots in self.components.values():
            for s in slots:
                if s not in seen:
                    seen.append(s)
        return tuple(seen)


def yam_product_profile() -> Profile:
    return Profile(
        name="yam_product",
        components={
            "left_arm": LEFT_ARM_SLOTS,
            "right_arm": RIGHT_ARM_SLOTS,
            "base": BASE_SLOTS,
            "lift": (LIFT_SLOT,),
        },
    )


def bench_continuous_profile() -> Profile:
    """Host demux for continuous / bus56 bench (spare-slot base).

    ``base`` here is slots 22–25 (CFG IDs set by continuous BASE_ROWS).
    Product drivetrain map stays available as ``base_product`` (14–19).
    """
    return Profile(
        name="yam_bench_continuous",
        components={
            "left_arm": LEFT_ARM_SLOTS,
            "right_arm": RIGHT_ARM_SLOTS,
            "base": BENCH_BASE_SLOTS,
            "base_product": BASE_SLOTS,
            "lift": (LIFT_SLOT,),
        },
    )


class ComponentView:
    """MIT desire/FB for one named component (ordered slots)."""

    def __init__(self, proxy: "HostProxy", name: str) -> None:
        self._proxy = proxy
        self.name = name
        self.slots = proxy.profile.slots(name)

    def set_desires(self, desires: Sequence[ActuatorDesire], *, send: bool = False) -> None:
        if len(desires) != len(self.slots):
            raise ValueError(
                f"{self.name}: expected {len(self.slots)} desires, got {len(desires)}"
            )
        batch = {slot: desire for slot, desire in zip(self.slots, desires)}
        self._proxy.set_actuators(batch, send=send)

    def blank(self, *, send: bool = False) -> None:
        self.set_desires([ActuatorDesire() for _ in self.slots], send=send)

    def hold(
        self,
        positions: Sequence[float],
        *,
        kp: float = 8.0,
        kd: float = 0.5,
        send: bool = False,
    ) -> None:
        if len(positions) != len(self.slots):
            raise ValueError(
                f"{self.name}: expected {len(self.slots)} positions, got {len(positions)}"
            )
        desires = [
            ActuatorDesire(position=float(p), kp=float(kp), kd=float(kd)) for p in positions
        ]
        self.set_desires(desires, send=send)

    def positions(self) -> Optional[List[float]]:
        """Latest FB positions for this component, or None if no feedback yet."""
        fb = self._proxy.latest_feedback()
        if fb is None:
            return None
        out: List[float] = []
        for slot in self.slots:
            st = fb.actuator(slot)
            out.append(float(st.position) if st is not None else 0.0)
        return out


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
    ) -> "HostProxy":
        """Connect. ``listen_pdu`` default False (bench): ignore stale PDU kill.

        Pass ``listen_pdu=True`` when a real PDB/PDU peer is on UART4 so
        soft-kill park + status LED traffic-light use kill bytes. MCU may
        still publish HARD+COMMS_LOSS when the peer is absent — that is
        firmware stale failsafe, not host policy.
        """
        hub = ControlsPcbHub.connect(
            port, serial=serial, baud=baud, persist_telemetry=persist_telemetry
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
            # Idle-anchor (p=1e-6, kp=0), not true blank p=0: firmware skips
            # MCP SPI on buses 4–6 for blank desires, so a later NORMAL observe
            # would only TX FDCAN 1–3 (CubeMars/Damiao keep streaming).
            anchored = {
                s: ActuatorDesire(position=1e-6) for s in range(ACTUATOR_COUNT)
            }
            hub.set_mcu_state(McuState.DIAG_ONLY, send=False)
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

    def component(self, name: str) -> ComponentView:
        return ComponentView(self, name)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            blank = {s: ActuatorDesire() for s in range(ACTUATOR_COUNT)}
            self.set_actuators(blank, send=False)
            self._hub.set_mcu_state(McuState.DIAG_ONLY, send=False)
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
