"""Action catalog — groundwork for a future "copy as script" button.

``ACTION_REGISTRY`` is a static catalog of every panel action the dashboard
knows how to wire up, keyed by a short slug. Each entry records where the
real implementation lives (``import_path``) and what calling it looks like
(``call_sig``) as *literal strings* — nothing here imports the panel modules
themselves; that happens in :mod:`routes` (:func:`routes.wire_panels`), which
tolerates the modules not existing yet (three other agents are building them
in parallel worktrees — see routes.py's module docstring).

This module must import cleanly with zero optional dependencies — it is pure
data, checked by tests that assert ``import_path``/``call_sig`` are
well-formed strings even when the underlying module is absent.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple


@dataclass(frozen=True)
class ParamSpec:
    name: str
    type: str
    default: object = None
    choices: Optional[tuple] = None
    min: Optional[float] = None
    max: Optional[float] = None
    help: str = ""


@dataclass(frozen=True)
class ActionSpec:
    key: str
    label: str
    blurb: str
    owner: str
    import_path: str
    call_sig: str
    params: tuple = ()
    requires_mode: str = "any"
    """"any" | "follow" | "active" | "debug" — dashboard-level precondition
    routes.py checks before dispatching (independent of whether the panel
    module itself is wired up)."""


def _import_path_ok(spec: ActionSpec) -> bool:
    parts = spec.import_path.split(".")
    return len(parts) >= 2 and all(p.isidentifier() for p in parts)


ACTION_REGISTRY: Dict[str, ActionSpec] = {
    "inventory": ActionSpec(
        key="inventory",
        label="Inventory",
        blurb="Enumerate live CFG-enabled actuators/servos on this bench.",
        owner="system",
        import_path="deft_controls_sdk.debug.panels.system.system_panel.run_inventory_panel",
        call_sig="run_inventory_panel(proxy, preset='bench')",
        params=(
            ParamSpec(name="preset", type="str", default="bench", choices=("bench", "product"),
                       help="CFG preset used to label slots"),
        ),
        requires_mode="debug",
    ),
    "bandwidth": ActionSpec(
        key="bandwidth",
        label="Bandwidth test",
        blurb="Short-lived raw bandwidth probe against a port string. Windows "
              "CDC is exclusive-open — only reachable while Follow/disconnected.",
        owner="system",
        import_path="deft_controls_sdk.debug.panels.system.system_panel.run_bandwidth_panel",
        call_sig="run_bandwidth_panel(port, seconds=2.0)",
        params=(
            ParamSpec(name="port", type="str", help="Serial port, e.g. COM5"),
            ParamSpec(name="seconds", type="float", default=2.0, min=0.5, max=30.0),
        ),
        requires_mode="follow",
    ),
    "cfg_show": ActionSpec(
        key="cfg_show",
        label="CFG show",
        blurb="Read the live per-slot CFG table (bus/protocol/motor_id/enabled).",
        owner="system",
        import_path="deft_controls_sdk.debug.panels.system.system_panel.show_cfg_panel",
        call_sig="show_cfg_panel(proxy)",
        requires_mode="debug",
    ),
    "cfg_set_slot": ActionSpec(
        key="cfg_set_slot",
        label="CFG set slot",
        blurb="Edit one slot's CFG row (bus/protocol/motor_id/enabled) in RAM.",
        owner="system",
        import_path="deft_controls_sdk.debug.panels.system.system_panel.set_cfg_slot_panel",
        call_sig="set_cfg_slot_panel(proxy, slot=0, bus=1, protocol=0, motor_id=0)",
        params=(
            ParamSpec(name="slot", type="int", min=0, max=25, help="Wire slot index"),
            ParamSpec(name="bus", type="int", min=1, max=6),
            ParamSpec(name="protocol", type="int", help="Protocol enum value"),
            ParamSpec(name="motor_id", type="int"),
            ParamSpec(name="master_id", type="int", default=0),
            ParamSpec(name="enabled", type="bool", default=True),
            ParamSpec(name="persist", type="bool", default=False, help="Also write to MCU NVM"),
        ),
        requires_mode="debug",
    ),
    "cfg_set_periph": ActionSpec(
        key="cfg_set_periph",
        label="CFG set peripherals",
        blurb="Edit the peripheral CFG block (LED/neck/listen_pdu) in RAM.",
        owner="system",
        import_path="deft_controls_sdk.debug.panels.system.system_panel.set_cfg_periph_panel",
        call_sig="set_cfg_periph_panel(proxy, periph={...})",
        params=(ParamSpec(name="persist", type="bool", default=False, help="Also write to MCU NVM"),),
        requires_mode="debug",
    ),
    "cfg_apply_table": ActionSpec(
        key="cfg_apply_table",
        label="CFG replace table",
        blurb="RAM-replace the whole actuator table at once (e.g. a pasted/preset table).",
        owner="system",
        import_path="deft_controls_sdk.debug.panels.system.system_panel.apply_cfg_table_panel",
        call_sig="apply_cfg_table_panel(proxy, rows=[...], disable_missing=True)",
        params=(
            ParamSpec(name="disable_missing", type="bool", default=True,
                       help="Slots with no row are disabled (a real replace, not a merge)"),
            ParamSpec(name="persist", type="bool", default=False, help="Also write to MCU NVM"),
        ),
        requires_mode="debug",
    ),
    "cfg_save_nvm": ActionSpec(
        key="cfg_save_nvm",
        label="CFG save to NVM",
        blurb="Persist the current RAM CFG table + peripheral block to MCU flash.",
        owner="system",
        import_path="deft_controls_sdk.debug.panels.system.system_panel.save_cfg_nvm_panel",
        call_sig="save_cfg_nvm_panel(proxy)",
        requires_mode="debug",
    ),
    "host_link": ActionSpec(
        key="host_link",
        label="Host-link diag",
        blurb="PDU/kill-state link evaluation for the active session.",
        owner="system",
        import_path="deft_controls_sdk.debug.panels.system.system_panel.host_link_eval_panel",
        call_sig="host_link_eval_panel(proxy)",
        params=(ParamSpec(name="require_peer", type="bool", default=False),),
        requires_mode="debug",
    ),
    "led_set": ActionSpec(
        key="led_set",
        label="LED set",
        blurb="Set the board LED mode/pattern/brightness directly.",
        owner="system",
        import_path="deft_controls_sdk.debug.panels.system.led_panel.set_led_panel",
        call_sig="set_led_panel(proxy, mode='follow', brightness=8)",
        params=(
            ParamSpec(name="mode", type="str", default="follow", choices=("follow", "pdu", "debug")),
            ParamSpec(name="brightness", type="int", default=8, min=0, max=255),
            ParamSpec(name="pattern", type="int", default=0, min=0, max=8),
            ParamSpec(name="count", type="int", default=0),
        ),
        requires_mode="debug",
    ),
    "led_preset": ActionSpec(
        key="led_preset",
        label="LED preset",
        blurb="Apply a named LED preset (gen_2/idle/pdu/follow).",
        owner="system",
        import_path="deft_controls_sdk.debug.panels.system.led_panel.apply_led_preset_panel",
        call_sig="apply_led_preset_panel(proxy, preset_name='idle')",
        params=(ParamSpec(name="preset_name", type="str", default="idle",
                            choices=("gen_2", "idle", "pdu", "follow")),),
        requires_mode="debug",
    ),
    "pdu_listen": ActionSpec(
        key="pdu_listen",
        label="listen_pdu toggle",
        blurb="Toggle whether the host obeys PDB kill-state for soft-kill/LED policy.",
        owner="system",
        import_path="deft_controls_sdk.debug.panels.system.led_panel.set_listen_pdu_panel",
        call_sig="set_listen_pdu_panel(proxy, enabled=True)",
        params=(ParamSpec(name="enabled", type="bool", default=False),),
        requires_mode="debug",
    ),
    "discover": ActionSpec(
        key="discover",
        label="Discover",
        blurb="Sweep CAN buses for live drives (RobStride/Damiao/Cubemars/ZeroErr).",
        owner="discover",
        import_path="deft_controls_sdk.debug.panels.discover.discover_panel.discover_panel",
        call_sig="discover_panel(proxy, buses=[1], protocols=['robstride'])",
        params=(
            ParamSpec(name="buses", type="str", default="1", help="Comma-separated bus list"),
            ParamSpec(name="protocols", type="str", default="robstride",
                       help="Comma-separated: robstride,damiao,cubemars,zeroerr"),
            ParamSpec(name="listen_ms", type="int", default=40, min=1, max=1000),
        ),
        requires_mode="debug",
    ),
    "calibrate": ActionSpec(
        key="calibrate",
        label="Calibrate (RobStride)",
        blurb="Run zero/offset calibration for one live RobStride slot. Shaft must "
              "spin freely; ~15-35s blocking; RobStride-only.",
        owner="discover",
        import_path="deft_controls_sdk.debug.panels.discover.calibrate_panel.calibrate_robstride_panel",
        call_sig="calibrate_robstride_panel(proxy, bus=1, motor_id=1)",
        params=(
            ParamSpec(name="bus", type="int", min=1, max=6),
            ParamSpec(name="motor_id", type="int"),
            ParamSpec(name="cal_listen_s", type="float", default=28.0, min=10.0, max=60.0),
            ParamSpec(name="skip_iq_test", type="bool", default=False),
            ParamSpec(name="strict_cali", type="bool", default=False),
        ),
        requires_mode="debug",
    ),
}

__all__ = ["ParamSpec", "ActionSpec", "ACTION_REGISTRY"]
