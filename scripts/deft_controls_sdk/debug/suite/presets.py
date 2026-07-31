"""Component CFG presets for ``debug.suite set --cfg`` -> ``preset``.

Actuator presets write into the editor's slot rows (then ``a``/``p``).
Neck / LED presets update the NVM v2 peripheral block (servos + led + flags)
and stage host desires; ``a`` applies RAM via CFG SET_PERIPH, ``p`` SAVEs.

``gen_2`` is available per-component and as a one-shot that fills arms +
torso + base + neck + LED together.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from deft_controls_sdk.host_proxy import (
    BASE_SLOTS,
    LEFT_ARM_SLOTS,
    LIFT_SLOT,
    NECK_PITCH_SERVO_SLOT,
    NECK_YAW_SERVO_SLOT,
    RIGHT_ARM_SLOTS,
)
from deft_controls_sdk.config.actuator import _DAMIAO_MASTER
from deft_controls_sdk.config.led import LED_PRESETS as _CONFIG_LED_PRESETS
from deft_controls_sdk.config.led import LedPreset
from deft_controls_sdk.link.api_types import LED_MODE_IDLE_CORNFLOWER
from deft_controls_sdk.config import (
    NECK_PITCH_DXL_ID,
    NECK_YAW_DXL_ID,
    PROTO_CUBEMARS,
    PROTO_DAMIAO,
    PROTO_NONE,
    PROTO_ROBSTRIDE,
    PROTO_ZEROERR,
    SERVO_MODEL_XL330_M288,
    SERVO_MODEL_XL430,
)

# Joint template: (bus, protocol, motor_id, master_id) in joint order 0..N-1
JointSpec = Tuple[int, int, int, int]

# Gen-2 fixed slot maps (0..25), motor_id 0x00 placeholders throughout.
#   bus1: 0-3 CubeMars, 4-7 RobStride
#   bus2: 8-11 CubeMars, 12-15 RobStride
#   bus3: 16 CubeMars, 17-19 ZeroErr
#   bus4/5/6: 20-21 / 22-23 / 24-25 RobStride
GEN2_ARM_SLOTS: Tuple[int, ...] = tuple(range(0, 16))
GEN2_TORSO_SLOTS: Tuple[int, ...] = (16, 17, 18, 19)
GEN2_BASE_SLOTS: Tuple[int, ...] = (20, 21, 22, 23, 24, 25)

_GEN2_ARM_JOINTS: Tuple[JointSpec, ...] = (
    *((1, PROTO_CUBEMARS, 0x00, 0) for _ in range(4)),
    *((1, PROTO_ROBSTRIDE, 0x00, 0) for _ in range(4)),
    *((2, PROTO_CUBEMARS, 0x00, 0) for _ in range(4)),
    *((2, PROTO_ROBSTRIDE, 0x00, 0) for _ in range(4)),
)
_GEN2_TORSO_JOINTS: Tuple[JointSpec, ...] = (
    (3, PROTO_CUBEMARS, 0x00, 0),
    (3, PROTO_ZEROERR, 0x00, 0),
    (3, PROTO_ZEROERR, 0x00, 0),
    (3, PROTO_ZEROERR, 0x00, 0),
)
_GEN2_BASE_JOINTS: Tuple[JointSpec, ...] = (
    (4, PROTO_ROBSTRIDE, 0x00, 0),
    (4, PROTO_ROBSTRIDE, 0x00, 0),
    (5, PROTO_ROBSTRIDE, 0x00, 0),
    (5, PROTO_ROBSTRIDE, 0x00, 0),
    (6, PROTO_ROBSTRIDE, 0x00, 0),
    (6, PROTO_ROBSTRIDE, 0x00, 0),
)


@dataclass(frozen=True)
class ActuatorPreset:
    """Named motor chain that maps onto a user-chosen ordered slot list."""

    name: str
    component: str  # arm | torso | base
    joints: Tuple[JointSpec, ...]
    default_slots: Tuple[int, ...]
    blurb: str


@dataclass(frozen=True)
class NeckPreset:
    name: str
    pitch_id: int
    yaw_id: int
    pitch_slot: int
    yaw_slot: int
    model: int
    blurb: str
    pitch_pos_min: int = 1024
    pitch_pos_max: int = 3072
    yaw_pos_min: int = 700
    yaw_pos_max: int = 2500


def _yam_arm_joints(bus: int) -> Tuple[JointSpec, ...]:
    return tuple(
        (bus, PROTO_DAMIAO, 0x01 + i, int(_DAMIAO_MASTER[i])) for i in range(7)
    )


def _cubemars_arm_joints(bus: int, motor_id: int = 0x01) -> Tuple[JointSpec, ...]:
    return tuple((bus, PROTO_CUBEMARS, int(motor_id) & 0xFF, 0) for _ in range(7))


ARM_PRESETS: Dict[str, ActuatorPreset] = {
    "gen_2": ActuatorPreset(
        name="gen_2",
        component="arm",
        joints=_GEN2_ARM_JOINTS,
        default_slots=GEN2_ARM_SLOTS,
        blurb="gen-2: bus1 0-3 CM + 4-7 RS, bus2 8-11 CM + 12-15 RS, id 0x00",
    ),
    "yam": ActuatorPreset(
        name="yam",
        component="arm",
        joints=_yam_arm_joints(1),
        default_slots=LEFT_ARM_SLOTS,
        blurb="7× Damiao CH1, motor_id 0x01..0x07 (left-arm shaped)",
    ),
    "yam_left": ActuatorPreset(
        name="yam_left",
        component="arm",
        joints=_yam_arm_joints(1),
        default_slots=LEFT_ARM_SLOTS,
        blurb="alias of yam -> default slots 0-6",
    ),
    "yam_right": ActuatorPreset(
        name="yam_right",
        component="arm",
        joints=_yam_arm_joints(2),
        default_slots=RIGHT_ARM_SLOTS,
        blurb="7× Damiao CH2, motor_id 0x01..0x07 -> default slots 7-13",
    ),
    "cubemars": ActuatorPreset(
        name="cubemars",
        component="arm",
        joints=_cubemars_arm_joints(1, 0x01),
        default_slots=LEFT_ARM_SLOTS,
        blurb="7× CubeMars MIT CH1 (bench scaffold)",
    ),
}

# Product base 14-19: steer 0x01 on CH4-6, drive 0x02 on CH4-6
_PRODUCT_BASE_JOINTS: Tuple[JointSpec, ...] = (
    (4, PROTO_ROBSTRIDE, 0x01, 0),
    (5, PROTO_ROBSTRIDE, 0x01, 0),
    (6, PROTO_ROBSTRIDE, 0x01, 0),
    (4, PROTO_ROBSTRIDE, 0x02, 0),
    (5, PROTO_ROBSTRIDE, 0x02, 0),
    (6, PROTO_ROBSTRIDE, 0x02, 0),
)

_BENCH_BASE_JOINTS: Tuple[JointSpec, ...] = (
    (5, PROTO_ROBSTRIDE, 0x70, 0),
    (5, PROTO_ROBSTRIDE, 0x74, 0),
    (6, PROTO_ROBSTRIDE, 0x75, 0),
    (6, PROTO_DAMIAO, 0x06, 0x16),
)

BASE_PRESETS: Dict[str, ActuatorPreset] = {
    "gen_2": ActuatorPreset(
        name="gen_2",
        component="base",
        joints=_GEN2_BASE_JOINTS,
        default_slots=GEN2_BASE_SLOTS,
        blurb="gen-2: bus4 20-21 + bus5 22-23 + bus6 24-25 RobStride/0x00",
    ),
    "product": ActuatorPreset(
        name="product",
        component="base",
        joints=_PRODUCT_BASE_JOINTS,
        default_slots=BASE_SLOTS,
        blurb="YAM product base slots 14-19 (RS 0x01/0x02 on CH4-6)",
    ),
    "bench": ActuatorPreset(
        name="bench",
        component="base",
        joints=_BENCH_BASE_JOINTS,
        default_slots=(22, 23, 24, 25),
        blurb="bench continuous map slots 22-25 (0x70/0x74/0x75 + DM 0x06)",
    ),
}

TORSO_PRESETS: Dict[str, ActuatorPreset] = {
    "gen_2": ActuatorPreset(
        name="gen_2",
        component="torso",
        joints=_GEN2_TORSO_JOINTS,
        default_slots=GEN2_TORSO_SLOTS,
        blurb="gen-2: bus3 slot16 CubeMars + 17-19 ZeroErr, id 0x00",
    ),
    "lift_off": ActuatorPreset(
        name="lift_off",
        component="torso",
        joints=((3, PROTO_NONE, 0, 0),),
        default_slots=(LIFT_SLOT,),
        blurb="lift slot disabled",
    ),
    "lift_yam": ActuatorPreset(
        name="lift_yam",
        component="torso",
        joints=((3, PROTO_DAMIAO, 0x01, 0x11),),
        default_slots=(LIFT_SLOT,),
        blurb="placeholder Damiao on CH3 id 0x01 -> slot 20 (edit ids after)",
    ),
}

NECK_PRESETS: Dict[str, NeckPreset] = {
    "gen_2": NeckPreset(
        name="gen_2",
        pitch_id=0x00,
        yaw_id=0x00,
        pitch_slot=NECK_PITCH_SERVO_SLOT,
        yaw_slot=NECK_YAW_SERVO_SLOT,
        model=SERVO_MODEL_XL330_M288,
        blurb="gen-2: XL330-M288-T both axes, id 0x00 placeholders",
    ),
    "yam": NeckPreset(
        name="yam",
        pitch_id=NECK_PITCH_DXL_ID,
        yaw_id=NECK_YAW_DXL_ID,
        pitch_slot=NECK_PITCH_SERVO_SLOT,
        yaw_slot=NECK_YAW_SERVO_SLOT,
        model=SERVO_MODEL_XL430,
        blurb="DXL XL430 pitch/yaw ids 1/2 — NVM servo_table + host center hold",
    ),
}

# Identity lives in config.led; suite re-exports for apply helpers.
LED_PRESETS: Dict[str, LedPreset] = dict(_CONFIG_LED_PRESETS)

COMPONENTS = ("arm", "torso", "base", "neck", "led")


def parse_slot_list(text: str, *, expect: int) -> Tuple[int, ...]:
    """Parse ``0,1,2,3,4,5,6`` or ``6 5 4 3 2 1 0`` into ``expect`` unique slots."""
    raw = (text or "").replace(",", " ").split()
    if not raw:
        raise ValueError("empty slot list")
    slots = tuple(int(x, 0) for x in raw)
    if len(slots) != expect:
        raise ValueError(f"need {expect} slots, got {len(slots)}: {slots}")
    if len(set(slots)) != len(slots):
        raise ValueError(f"duplicate slots in {slots}")
    for s in slots:
        if not (0 <= s < 26):
            raise ValueError(f"slot {s} out of range 0..25")
    return slots


def apply_actuator_preset_to_rows(
    rows: List[dict],
    preset: ActuatorPreset,
    slots: Sequence[int],
    *,
    enabled: bool = True,
) -> List[int]:
    """Mutate editor rows; return dirty slot indices."""
    if len(slots) != len(preset.joints):
        raise ValueError(
            f"preset {preset.name} has {len(preset.joints)} joints, "
            f"got {len(slots)} slots"
        )
    by_slot = {int(r["slot"]): r for r in rows}
    dirty: List[int] = []
    for slot, (bus, proto, mid, master) in zip(slots, preset.joints):
        row = by_slot.get(int(slot))
        if row is None:
            raise KeyError(f"slot {slot} not in editor table")
        row["enabled"] = bool(enabled) and proto != PROTO_NONE
        row["bus"] = int(bus)
        row["protocol"] = int(proto)
        row["motor_id"] = int(mid) & 0xFF
        row["master_id"] = int(master) & 0xFF
        dirty.append(int(slot))
    return dirty


def apply_neck_preset(proxy, preset: NeckPreset, periph: dict) -> None:
    """Update NVM periph block + stage host ServoDesire (center hold)."""
    servos = list(periph.get("servos") or [])
    while len(servos) < 2:
        servos.append(
            {
                "slot": len(servos),
                "model": 0,
                "id": 0,
                "enabled": False,
                "pos_min": 0,
                "pos_max": 4095,
            }
        )
    servos[0] = {
        "slot": 0,
        "model": int(preset.model),
        "id": int(preset.pitch_id),
        "enabled": True,
        "pos_min": int(preset.pitch_pos_min),
        "pos_max": int(preset.pitch_pos_max),
    }
    servos[1] = {
        "slot": 1,
        "model": int(preset.model),
        "id": int(preset.yaw_id),
        "enabled": True,
        "pos_min": int(preset.yaw_pos_min),
        "pos_max": int(preset.yaw_pos_max),
    }
    periph["servos"] = servos
    periph.setdefault("flags", 0)

    from deft_controls_sdk.actions import ServoAction

    ServoAction(proxy).neck_center(
        pitch_id=preset.pitch_id,
        yaw_id=preset.yaw_id,
        pitch_slot=preset.pitch_slot,
        yaw_slot=preset.yaw_slot,
        position=2048,
        send=False,
    )
    proxy.send_once()


def apply_led_preset(proxy, preset: LedPreset, periph: dict) -> None:
    """Update NVM LED defaults + stage host LedDesire via actions.LedAction."""
    from deft_controls_sdk.actions import LedAction
    from deft_controls_sdk.link.exchange import CFG_FLAG_LISTEN_PDU

    prev = dict(periph.get("led") or {})
    count = int(prev.get("default_count", 300)) or 300
    if preset.name == "gen_2" or preset.policy == "debug":
        mode = (
            LED_MODE_IDLE_CORNFLOWER
            if preset.name == "gen_2"
            else int(preset.pattern)
        )
        periph["led"] = {
            "default_count": count,
            "default_mode": mode,
            "default_brightness": int(preset.brightness),
        }
    else:
        periph["led"] = {
            "default_count": count,
            "default_mode": int(prev.get("default_mode", LED_MODE_IDLE_CORNFLOWER)),
            "default_brightness": int(prev.get("default_brightness", 8)),
        }
    flags = int(periph.get("flags", 0)) & 0xFF
    if preset.policy == "pdu":
        flags |= CFG_FLAG_LISTEN_PDU
        periph["listen_pdu"] = True
    periph["flags"] = flags
    LedAction(proxy).apply_preset(preset, send=True)


def apply_gen2_all(proxy, rows: List[dict], dirty: set, periph: dict, periph_dirty: list) -> str:
    """Fill arms + torso + base + neck + LED for gen-2 (no prompts)."""
    dirty.update(
        apply_actuator_preset_to_rows(rows, ARM_PRESETS["gen_2"], GEN2_ARM_SLOTS)
    )
    dirty.update(
        apply_actuator_preset_to_rows(rows, TORSO_PRESETS["gen_2"], GEN2_TORSO_SLOTS)
    )
    dirty.update(
        apply_actuator_preset_to_rows(rows, BASE_PRESETS["gen_2"], GEN2_BASE_SLOTS)
    )
    apply_neck_preset(proxy, NECK_PRESETS["gen_2"], periph)
    apply_led_preset(proxy, LED_PRESETS["gen_2"], periph)
    periph_dirty[:] = [True]
    return (
        "preset gen_2 -> bus1 0-3 CM/4-7 RS, bus2 8-11 CM/12-15 RS, "
        "bus3 16 CM/17-19 ZE, bus4-6 20-25 RS (all id 0x00); "
        "neck XL330-M288-T/0x00; LED follow+cornflower "
        "(dirty; 'a' apply RAM, 'p' persist NVM v2)"
    )


def list_presets(component: str) -> Dict[str, object]:
    c = component.strip().lower()
    if c == "arm":
        return dict(ARM_PRESETS)
    if c == "base":
        return dict(BASE_PRESETS)
    if c == "torso":
        return dict(TORSO_PRESETS)
    if c == "neck":
        return dict(NECK_PRESETS)
    if c == "led":
        return dict(LED_PRESETS)
    raise KeyError(c)


def run_component_preset(
    proxy,
    rows: List[dict],
    dirty: set,
    periph: dict,
    periph_dirty: list,
    comp: str,
) -> Optional[str]:
    """Preset picker for one component (``arm`` / ``torso`` / …)."""
    comp = comp.strip().lower().replace("-", "_")
    if comp not in COMPONENTS:
        print(f"unknown component {comp!r}")
        return None

    catalog = list_presets(comp)
    print(f"{comp} presets: " + ", ".join(sorted(catalog)))
    for name, p in sorted(catalog.items()):
        print(f"  {name:12}  {p.blurb}")
    name = input(f"{comp} preset name> ").strip().lower().replace("-", "_")
    if name not in catalog:
        print(f"unknown preset {name!r}")
        return None
    preset = catalog[name]

    if comp in ("arm", "torso", "base"):
        assert isinstance(preset, ActuatorPreset)
        default = ",".join(str(s) for s in preset.default_slots)
        print(
            f"slot map for {len(preset.joints)} joints "
            f"(default {default}; reverse / custom ok)"
        )
        raw = input(f"slots [{default}]> ").strip()
        slots = parse_slot_list(raw or default, expect=len(preset.joints))
        changed = apply_actuator_preset_to_rows(rows, preset, slots)
        dirty.update(changed)
        return (
            f"preset {comp}/{preset.name} -> slots {list(slots)} "
            f"(dirty; 'a' apply RAM, 'p' persist NVM v2)"
        )

    if comp == "neck":
        assert isinstance(preset, NeckPreset)
        apply_neck_preset(proxy, preset, periph)
        periph_dirty[:] = [True]
        return (
            f"preset neck/{preset.name} -> periph dirty "
            f"(model={preset.model} ids {preset.pitch_id}/{preset.yaw_id}; "
            f"'a'/'p' writes NVM v2)"
        )

    if comp == "led":
        assert isinstance(preset, LedPreset)
        apply_led_preset(proxy, preset, periph)
        periph_dirty[:] = [True]
        return (
            f"preset led/{preset.name} -> periph dirty "
            f"(policy={preset.policy}; 'a'/'p' writes NVM v2)"
        )

    return None


def run_preset_dialogue(
    proxy, rows: List[dict], dirty: set, periph: dict, periph_dirty: list
) -> Optional[str]:
    """Interactive preset flow. Mutates rows/dirty/periph. Returns status line.

    At the component prompt, ``gen_2`` applies the full pack with no further
    questions.
    """
    print("component? " + " | ".join(COMPONENTS) + " | gen_2 (all)")
    comp = input("preset component> ").strip().lower().replace("-", "_")
    if comp in ("gen_2", "gen2"):
        return apply_gen2_all(proxy, rows, dirty, periph, periph_dirty)
    return run_component_preset(proxy, rows, dirty, periph, periph_dirty, comp)
