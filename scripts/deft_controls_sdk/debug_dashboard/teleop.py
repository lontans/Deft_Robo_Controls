"""Teleop slot-group model + host-side slew engine for the debug dashboard.

Turns "type a position/kp/kd and hit Apply" into: pick a target (arm — a
bounded slider; base — a jog direction + speed) and let the host ramp the
held desire toward it at a bounded cruise rate every tick, instead of
snapping there instantly. This is the same shape as the plant stream's
existing held-desire resend (see ``AppState.set_actuator`` in ``app.py``) —
the engine below just moves the target smoothly instead of requiring the
operator to type intermediate points.

Ranges/gains here mirror the bench-verified constants in
``scripts/yam_continuous_all.py`` / ``deft_controls_sdk/vbeta/slots.py`` /
``deft_controls_sdk/vbeta/yam_bench_clear_left.py`` and the
``docs/peripherals/*.md`` pages they're sourced from (arm-damiao-ch1.md,
base-robstride-mcp.md, base-damiao-ch6.md, dxl-neck.md). Keep this file's
numbers in sync with those — don't invent new ranges for slots that have no
live-verified evidence; mark them ``verified=False`` instead (gates
target-teleop in the UI, not Idle — blanking a desire is always safe).
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

from deft_controls_sdk.link import ActuatorDesire, ServoDesire
from deft_controls_sdk.link.exchange import parse_servo_feedback
from deft_controls_sdk.vbeta.slots import (
    BASE_DRIVE_SLOTS,
    BASE_STEER_SLOTS,
    DEFAULT_ARM_KD,
    DEFAULT_ARM_KP,
    DEFAULT_DRIVE_KD,
    DEFAULT_STEER_KD,
    DEFAULT_STEER_KP,
    LEFT_ARM_SLOTS,
    NECK_PITCH_DXL_ID,
    NECK_PITCH_SERVO_SLOT,
    NECK_YAW_DXL_ID,
    NECK_YAW_SERVO_SLOT,
    RIGHT_ARM_SLOTS,
)
from deft_controls_sdk.vbeta.yam_bench_clear_left import CLEAR_HI as ARM_LEFT_HI
from deft_controls_sdk.vbeta.yam_bench_clear_left import CLEAR_LO as ARM_LEFT_LO

# -- Base, bench wiring (live-verified 2026-07-24) — docs/peripherals/base-robstride-mcp.md
# + base-damiao-ch6.md. Mirrors yam_continuous_all.py's BASE_ROWS / RS_* / DM_BASE_* — do
# not let these drift from that script without updating both.
RS_P_MIN = -12.57
RS_P_MAX = 12.57
RS_MARGIN = 0.35
RS_KP = 20.0
RS_KD = 1.0
RS_LO = RS_P_MIN + RS_MARGIN
RS_HI = RS_P_MAX - RS_MARGIN

DM_BASE_TRAVEL = 2.0 * math.pi  # +/- one turn from seed center — soft window, not a hardware rail
DM_BASE_MIN_KP = 2.0  # Damiao: a kp=0 desire never TXes on MCP — floor so enable can latch
DM_BASE_KP = 10.0
DM_BASE_KD = 0.5

# (slot, bus, label) — CH5/CH6 MCP RobStride pair + the Damiao sharing CH6.
BASE_BENCH_ROWS: Tuple[Tuple[int, str], ...] = (
    (22, "CH5 RS02"),
    (23, "CH5 RS01"),
    (24, "CH6 RS01"),
    (25, "CH6 Damiao"),
)
BASE_BENCH_DAMIAO_SLOTS: Tuple[int, ...] = (25,)

# -- Neck DXL, native steps — docs/peripherals/dxl-neck.md. Mirrors
# yam_continuous_all.py's DXL_LO/DXL_HI/DXL_CRUISE_TICK_S (40-step inset off a captured
# clear range; the firmware hard clamp is wider — this is the "actual mechanically clear"
# range, use it, not the raw firmware table).
DXL_LO: Dict[int, int] = {NECK_PITCH_SERVO_SLOT: 2193, NECK_YAW_SERVO_SLOT: 911}
DXL_HI: Dict[int, int] = {NECK_PITCH_SERVO_SLOT: 3018, NECK_YAW_SERVO_SLOT: 2582}
DXL_IDS: Dict[int, int] = {NECK_PITCH_SERVO_SLOT: NECK_PITCH_DXL_ID, NECK_YAW_SERVO_SLOT: NECK_YAW_DXL_ID}
DXL_CRUISE_MAX = 280.0  # native steps/s — continuous script's own autonomous cruise rate; GUI ceiling

# GUI-side cruise ceilings — deliberately gentler than the autonomous continuous-cruise rates
# above (BASE_RATE=pi/4~=0.785 rad/s, DXL 280 steps/s) since a human is driving live, not a
# proven bounce pattern.
ARM_CRUISE_MAX = 0.8  # rad/s
ARM_CRUISE_DEFAULT = 0.35  # rad/s — "walk it there," not a snap
BASE_CRUISE_MAX = 0.7  # rad/s
BASE_CRUISE_DEFAULT = 0.3  # rad/s
DXL_CRUISE_DEFAULT = 150.0  # native steps/s

# -- Settled-hold robustness (2026-07-24 field report: J3 released -> J2 sagged -> whole arm
# dropped). NOT live-verified yet — bench/CDC is owned by the Mission Impossible stress suite
# right now, so this ships offline-tested only; re-check against a live board once CDC is free.
#
# Deliberately does *not* touch kp or let the commanded target drift toward feedback — see
# arm-damiao-ch1.md's "J2=160 was a mistaken 'gravity' bump; it vibrates under multi-joint CLEAR
# motion" note, i.e. don't blindly stiffen position gain, and don't quietly give the target away
# either (a target that silently complies with a sag isn't "holding," it's "giving up" and can
# feed a cascade). Once a slot has arrived and is holding station, this only (a) reactively adds
# damping (kd) when live feedback shows the joint is actually moving while it's supposed to be at
# rest — kd penalizes velocity error, which directly opposes an in-progress gravity/coupling sag
# without changing the static position-holding stiffness the arm was already tuned with — and
# (b) flags (never silently corrects) a large sustained position/feedback mismatch so the operator
# sees "this joint is under real load" instead of a target that either fights forever or gives up.
HOLD_VEL_DEADBAND = 0.05  # rad/s — below this, treat as "at rest," no boost (avoid noise chatter)
HOLD_KD_BOOST_GAIN = 6.0  # kd multiplier growth per rad/s of sag velocity past the deadband
HOLD_KD_BOOST_MAX = 1.8  # hard cap on the kd multiplier — bounded damping increase, never kp
HOLD_FLAG_RAD = 0.12  # |feedback - target| beyond this while "holding" -> flagged for the UI


@dataclass(frozen=True)
class SlotSpec:
    slot: int
    group: str  # "arm_left" | "arm_right" | "base" | "neck"
    label: str
    protocol: str
    kp: float
    kd: float
    verified: bool  # live-verified range/gains on the current bench; gates target-teleop
    lo: Optional[float]
    hi: Optional[float]
    seed_relative: bool  # True: [lo, hi] is (seed +/- window), computed when first engaged (Damiao base)
    kp_floor: float = 0.0
    cruise_max: float = ARM_CRUISE_MAX
    cruise_default: float = ARM_CRUISE_DEFAULT


def build_actuator_specs(cfg_map: str = "bench") -> Dict[int, SlotSpec]:
    """CFG map hint: ``bench`` is what's actually wired/live-verified on this bench today
    (base on slots 22-25 — see base-robstride-mcp.md's "Known falsehoods retired": product
    slots 14-19 are the idealized product CFG, *not* current bench wiring). ``product`` is
    offered only as a label for the wizard's CFG-map hint (plan ask); its base slots have no
    live-verified range on this bench, so they come back ``verified=False`` (Idle still works,
    target-teleop is gated in the UI)."""
    specs: Dict[int, SlotSpec] = {}
    for i, slot in enumerate(LEFT_ARM_SLOTS):
        specs[slot] = SlotSpec(
            slot=slot, group="arm_left", label=f"L-arm J{i + 1}", protocol="damiao",
            kp=DEFAULT_ARM_KP[i], kd=DEFAULT_ARM_KD[i], verified=True,
            lo=ARM_LEFT_LO[i], hi=ARM_LEFT_HI[i], seed_relative=False,
            cruise_max=ARM_CRUISE_MAX, cruise_default=ARM_CRUISE_DEFAULT,
        )
    for i, slot in enumerate(RIGHT_ARM_SLOTS):
        specs[slot] = SlotSpec(
            slot=slot, group="arm_right", label=f"R-arm J{i + 1}", protocol="damiao",
            kp=DEFAULT_ARM_KP[i], kd=DEFAULT_ARM_KD[i], verified=False,
            lo=None, hi=None, seed_relative=False,
            cruise_max=ARM_CRUISE_MAX, cruise_default=ARM_CRUISE_DEFAULT,
        )
    if cfg_map == "bench":
        for slot, label in BASE_BENCH_ROWS:
            is_damiao = slot in BASE_BENCH_DAMIAO_SLOTS
            specs[slot] = SlotSpec(
                slot=slot, group="base", label=label, protocol="damiao" if is_damiao else "robstride",
                kp=DM_BASE_KP if is_damiao else RS_KP, kd=DM_BASE_KD if is_damiao else RS_KD,
                verified=True,
                lo=None if is_damiao else RS_LO, hi=None if is_damiao else RS_HI,
                seed_relative=is_damiao, kp_floor=DM_BASE_MIN_KP if is_damiao else 0.0,
                cruise_max=BASE_CRUISE_MAX, cruise_default=BASE_CRUISE_DEFAULT,
            )
    else:
        for label_map, kp, kd in ((BASE_STEER_SLOTS, DEFAULT_STEER_KP, DEFAULT_STEER_KD),
                                   (BASE_DRIVE_SLOTS, 0.0, DEFAULT_DRIVE_KD)):
            for label, slot in label_map.items():
                specs[slot] = SlotSpec(
                    slot=slot, group="base", label=f"product {label}", protocol="robstride",
                    kp=kp, kd=kd, verified=False,
                    lo=RS_LO, hi=RS_HI, seed_relative=False,
                    cruise_max=BASE_CRUISE_MAX, cruise_default=BASE_CRUISE_DEFAULT,
                )
    return specs


def build_servo_specs() -> Dict[int, SlotSpec]:
    specs: Dict[int, SlotSpec] = {}
    for slot in (NECK_PITCH_SERVO_SLOT, NECK_YAW_SERVO_SLOT):
        specs[slot] = SlotSpec(
            slot=slot, group="neck", label="pitch" if slot == NECK_PITCH_SERVO_SLOT else "yaw",
            protocol="dxl", kp=0.0, kd=0.0, verified=True,
            lo=float(DXL_LO[slot]), hi=float(DXL_HI[slot]), seed_relative=False,
            cruise_max=DXL_CRUISE_MAX, cruise_default=DXL_CRUISE_DEFAULT,
        )
    return specs


def _clamp(v: float, lo: float, hi: float) -> float:
    return min(max(v, lo), hi)


def read_dxl_present_position(raw: Optional[bytes], slot: int) -> Optional[int]:
    """Conservative subset of ``_read_dxl_fb`` in ``yam_continuous_all.py`` (see dxl-neck.md
    "Present-position parse quirks"): mask to 12 bits when raw position reads above 4095,
    accept only when the reporting motor id matches (or is untagged/0). Unlike the reference
    implementation this never falls back to accepting an id mismatch just because the position
    happens to be nonzero — erring toward "reject and retry next tick" over seeding from a
    sample that might belong to the other joint."""
    if raw is None:
        return None
    fb = parse_servo_feedback(raw, slot)
    if fb is None:
        return None
    src = fb["motor_source_id"]
    if src not in (0, DXL_IDS.get(slot)):
        return None
    pos = fb["present_position"]
    if pos > 4095:
        pos &= 0x0FFF
    return int(pos)


class TeleopEngine:
    """One background thread ramps every engaged actuator/servo slot's held
    desire toward its target at a bounded cruise rate. Disconnect / observe
    must call :meth:`disengage_all` — the engine holds no hub reference of
    its own, it only writes through whatever ``hub_getter()`` returns *at
    tick time*, so a stale hub from a previous connection can never receive
    a write."""

    def __init__(
        self, *, hub_getter: Callable[[], object],
        feedback_getter: Optional[Callable[[], Dict[int, dict]]] = None,
        hz: float = 25.0,
    ) -> None:
        self._hub_getter = hub_getter
        # Returns {slot: {"position": float, "velocity": float}} for slots with live feedback
        # this tick — used only for the settled-hold damping/flag logic below, never to move the
        # commanded target. Optional so tests / callers that don't care about hold-robustness can
        # omit it and get the plain slew behavior.
        self._feedback_getter = feedback_getter
        self._dt = 1.0 / hz
        self._lock = threading.Lock()
        self._act: Dict[int, dict] = {}
        self._srv: Dict[int, dict] = {}
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    # -- introspection (for /api/state) ------------------------------------------------

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "actuators": {
                    slot: {
                        "pos": st["pos"], "target": st["target"], "cruise": st["cruise"],
                        "flagged": st.get("flagged", False),
                    }
                    for slot, st in self._act.items()
                },
                "servos": {
                    slot: {
                        "pos": st["pos"], "target": st["target"], "cruise": st["cruise"],
                        "seeded": st["seeded"],
                    }
                    for slot, st in self._srv.items()
                },
            }

    # -- actuator (arm / base) ----------------------------------------------------------

    def engage_actuator(
        self, slot: int, *, spec: SlotSpec, seed: float, target: float, cruise: float,
    ) -> None:
        if not spec.verified or spec.lo is None or spec.hi is None:
            raise ValueError(f"slot {slot} ({spec.label}) has no live-verified range on this bench")
        cruise = _clamp(abs(cruise), 0.0, spec.cruise_max)
        target = _clamp(target, spec.lo, spec.hi)
        with self._lock:
            cur = self._act.get(slot)
            pos = cur["pos"] if cur is not None else seed
            self._act[slot] = {
                "pos": pos, "target": target, "cruise": cruise,
                "lo": spec.lo, "hi": spec.hi, "kp": spec.kp, "kd": spec.kd, "kp_floor": spec.kp_floor,
            }
        self._ensure_thread()

    def engage_actuator_seed_relative(
        self, slot: int, *, spec: SlotSpec, seed: float, direction: int, cruise: float, window: float,
    ) -> None:
        """Damiao base slot: range is (seed +/- window), not an absolute rail — see
        base-damiao-ch6.md. Locks the range in at engage time from the live seed."""
        lo, hi = seed - window, seed + window
        cruise = _clamp(abs(cruise), 0.0, spec.cruise_max)
        target = hi if direction >= 0 else lo
        with self._lock:
            self._act[slot] = {
                "pos": seed, "target": target, "cruise": cruise,
                "lo": lo, "hi": hi, "kp": spec.kp, "kd": spec.kd, "kp_floor": spec.kp_floor,
            }
        self._ensure_thread()

    def jog_actuator(self, slot: int, *, spec: SlotSpec, seed: float, direction: int, cruise: float) -> None:
        if spec.seed_relative:
            self.engage_actuator_seed_relative(
                slot, spec=spec, seed=seed, direction=direction, cruise=cruise, window=DM_BASE_TRAVEL,
            )
            return
        if spec.lo is None or spec.hi is None:
            raise ValueError(f"slot {slot} ({spec.label}) has no live-verified range on this bench")
        target = spec.hi if direction >= 0 else spec.lo
        self.engage_actuator(slot, spec=spec, seed=seed, target=target, cruise=cruise)

    def stop_actuator(self, slot: int) -> None:
        """Freeze in place (target := current position) — does not blank the desire; use
        the existing per-slot / group Idle for that."""
        with self._lock:
            cur = self._act.get(slot)
            if cur is not None:
                cur["target"] = cur["pos"]

    def disengage_actuator(self, slot: int) -> None:
        with self._lock:
            self._act.pop(slot, None)

    # -- servo (neck DXL) -----------------------------------------------------------------

    def engage_servo(self, slot: int, *, spec: SlotSpec, target: float, cruise: float) -> None:
        cruise = _clamp(abs(cruise), 0.0, spec.cruise_max)
        target = _clamp(target, spec.lo, spec.hi)
        with self._lock:
            cur = self._srv.get(slot)
            if cur is not None:
                cur["target"] = target
                cur["cruise"] = cruise
            else:
                self._srv[slot] = {
                    "pos": None, "target": target, "cruise": cruise, "seeded": False,
                    "lo": spec.lo, "hi": spec.hi, "servo_id": DXL_IDS[slot],
                }
        self._ensure_thread()

    def stop_servo(self, slot: int) -> None:
        with self._lock:
            cur = self._srv.get(slot)
            if cur is not None and cur["pos"] is not None:
                cur["target"] = cur["pos"]

    def disengage_servo(self, slot: int) -> None:
        with self._lock:
            self._srv.pop(slot, None)

    def disengage_all(self) -> None:
        with self._lock:
            self._act.clear()
            self._srv.clear()

    # -- thread ------------------------------------------------------------------------

    def _ensure_thread(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="deft-dashboard-teleop", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self.disengage_all()

    def _run(self) -> None:
        while not self._stop.is_set():
            t0 = time.perf_counter()
            with self._lock:
                idle = not self._act and not self._srv
            if idle:
                return  # re-armed lazily by the next engage_*() call
            hub = self._hub_getter()
            if hub is not None:
                self._tick_actuators(hub)
                self._tick_servos(hub)
            elapsed = time.perf_counter() - t0
            time.sleep(max(0.0, self._dt - elapsed))

    def _tick_actuators(self, hub: object) -> None:
        with self._lock:
            items = list(self._act.items())
        fb = self._feedback_getter() if (self._feedback_getter and items) else {}
        for slot, st in items:
            pos, target, cruise = st["pos"], st["target"], st["cruise"]
            step = cruise * self._dt
            delta = target - pos
            kd = st["kd"]
            flagged = False
            if abs(delta) <= max(step, 1e-6):
                # Arrived — holding station. The target itself never drifts toward feedback
                # (see module docstring): a real gravity/coupling disturbance gets a bounded,
                # reactive damping boost plus a visibility flag, not a quieter target.
                new_pos, vel = target, 0.0
                sample = fb.get(slot)
                if sample is not None:
                    fb_pos = sample.get("position")
                    fb_vel = sample.get("velocity") or 0.0
                    if fb_pos is not None and abs(fb_pos - target) > HOLD_FLAG_RAD:
                        flagged = True
                    if abs(fb_vel) > HOLD_VEL_DEADBAND:
                        boost = 1.0 + min(
                            HOLD_KD_BOOST_GAIN * (abs(fb_vel) - HOLD_VEL_DEADBAND),
                            HOLD_KD_BOOST_MAX - 1.0,
                        )
                        kd = st["kd"] * boost
            else:
                direction = 1.0 if delta > 0 else -1.0
                new_pos, vel = pos + direction * step, direction * cruise
            new_pos = _clamp(new_pos, st["lo"], st["hi"])
            with self._lock:
                if slot in self._act:
                    self._act[slot]["pos"] = new_pos
                    self._act[slot]["flagged"] = flagged
            kp = st["kp"]
            if kp < st["kp_floor"]:
                kp = st["kp_floor"]
            try:
                hub.set_actuator(
                    slot, ActuatorDesire(position=new_pos, velocity=vel, kp=kp, kd=kd, torque=0.0),
                    send=False,
                )
            except Exception:
                pass

    def _tick_servos(self, hub: object) -> None:
        with self._lock:
            items = list(self._srv.items())
        raw = getattr(getattr(hub, "_connection", None), "_latest_fb_raw", None)  # noqa: SLF001
        for slot, st in items:
            if not st["seeded"]:
                # Torque-off discover first (dxl-neck.md) — never command a goal before we
                # have this joint's real present position.
                try:
                    hub.set_servo(
                        slot, ServoDesire(servo_id=st["servo_id"], native_step_position=0,
                                           torque_enable=False, operating_mode=3),
                        send=False,
                    )
                except Exception:
                    pass
                present = read_dxl_present_position(raw, slot)
                if present is None:
                    continue
                with self._lock:
                    if slot in self._srv:
                        self._srv[slot]["pos"] = float(present)
                        self._srv[slot]["seeded"] = True
                continue
            pos, target, cruise = st["pos"], st["target"], st["cruise"]
            step = cruise * self._dt
            delta = target - pos
            new_pos = target if abs(delta) <= max(step, 1.0) else pos + math.copysign(step, delta)
            new_pos = _clamp(new_pos, st["lo"], st["hi"])
            with self._lock:
                if slot in self._srv:
                    self._srv[slot]["pos"] = new_pos
            try:
                hub.set_servo(
                    slot, ServoDesire(servo_id=st["servo_id"], native_step_position=int(round(new_pos)),
                                       torque_enable=True, operating_mode=3),
                    send=False,
                )
            except Exception:
                pass
