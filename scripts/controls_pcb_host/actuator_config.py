"""
Actuator slot configuration — host mirror of firmware actuator_table[].

When a USB session is available, `config show` / `config set` use the CFG PDU to
read or update the MCU table and optional flash NVM (`--persist`).

Move a slot to another schematic bus (daisy-chain / mixed protocol on one channel):

```powershell
python scripts/control_hub.py config set --port COM5 --slot 1 --bus 3 --motor-id 0x74
python scripts/control_hub.py config set --port COM5 --slot 1 --bus 3 --motor-id 0x74 --persist
```

`--bus` and `--channel` are equivalent (CH1–CH6).
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from enum import IntEnum
from typing import List, Optional, Tuple, TYPE_CHECKING

from .protocol import ACTUATOR_COUNT
from .protocol.can_bus import normalize_can_bus

if TYPE_CHECKING:
    from .session import PcbSession


class Protocol(IntEnum):
    NONE = 0
    ROBSTRIDE = 1
    CUBEMARS = 2
    DAMIAO = 3


PROTOCOL_NAMES = {
    Protocol.NONE: "none",
    Protocol.ROBSTRIDE: "robstride",
    Protocol.CUBEMARS: "cubemars",
    Protocol.DAMIAO: "damiao",
}

NAME_TO_PROTOCOL = {v: k for k, v in PROTOCOL_NAMES.items()}


@dataclass
class ActuatorSlotConfig:
    slot: int
    bus: int
    protocol: Protocol
    motor_id: int
    master_id: int = 0
    enabled: bool = True

    @property
    def protocol_name(self) -> str:
        return PROTOCOL_NAMES.get(self.protocol, "unknown")


# Mirror plant_config_load_factory_defaults() (dual YAM: arm1 CH1 daisy, arm2 CH2 daisy).
_DEFAULT_TABLE: Tuple[Tuple[int, Protocol, int, int, bool], ...] = (
    (1, Protocol.DAMIAO, 0x01, 0, True),  # slot 0   arm1 J1
    (1, Protocol.DAMIAO, 0x02, 0, True),  # slot 1   arm1 J2
    (1, Protocol.DAMIAO, 0x03, 0, True),  # slot 2   arm1 J3
    (1, Protocol.DAMIAO, 0x04, 0, True),  # slot 3   arm1 J4
    (1, Protocol.DAMIAO, 0x05, 0, True),  # slot 4   arm1 J5
    (1, Protocol.DAMIAO, 0x06, 0, True),  # slot 5   arm1 J6
    (1, Protocol.DAMIAO, 0x07, 0, True),  # slot 6   arm1 J7
    (2, Protocol.DAMIAO, 0x01, 0, True),  # slot 7   arm2 J1
    (2, Protocol.DAMIAO, 0x02, 0, True),  # slot 8   arm2 J2
    (2, Protocol.DAMIAO, 0x03, 0, True),  # slot 9   arm2 J3
    (2, Protocol.DAMIAO, 0x04, 0, True),  # slot 10  arm2 J4
    (2, Protocol.DAMIAO, 0x05, 0, True),  # slot 11  arm2 J5
    (2, Protocol.DAMIAO, 0x06, 0, True),  # slot 12  arm2 J6
    (2, Protocol.DAMIAO, 0x07, 0, True),  # slot 13  arm2 J7
)


def default_table() -> List[ActuatorSlotConfig]:
    out: List[ActuatorSlotConfig] = []
    for slot, (bus, proto, mid, master, en) in enumerate(_DEFAULT_TABLE):
        out.append(
            ActuatorSlotConfig(
                slot=slot,
                bus=bus,
                protocol=proto,
                motor_id=mid,
                master_id=master,
                enabled=en,
            )
        )
    return out


_HOST_TABLE: List[ActuatorSlotConfig] = default_table()
_FIRMWARE_VERIFIED = False


def host_table() -> List[ActuatorSlotConfig]:
    return list(_HOST_TABLE)


def slots_for_protocol(protocol: Protocol) -> List[ActuatorSlotConfig]:
    return [cfg for cfg in _HOST_TABLE if cfg.protocol == protocol and cfg.enabled]


def slot_config(slot: int) -> ActuatorSlotConfig:
    if slot < 0 or slot >= ACTUATOR_COUNT:
        raise ValueError(f"slot must be 0..{ACTUATOR_COUNT - 1}, got {slot}")
    return _HOST_TABLE[slot]


def resolve_slot(
    *,
    slot: Optional[int] = None,
    bus: Optional[int] = None,
    motor_id: Optional[int] = None,
) -> ActuatorSlotConfig:
    if slot is not None:
        return slot_config(slot)
    if bus is not None and motor_id is not None:
        mid = motor_id & 0xFF
        for cfg in _HOST_TABLE:
            if cfg.bus == bus and cfg.motor_id == mid:
                return cfg
        raise ValueError(f"no configured slot for bus={bus} motor_id=0x{mid:02X}")
    raise ValueError("provide --slot or (--bus and --motor-id)")


def parse_protocol(name: str) -> Protocol:
    key = name.strip().lower()
    if key not in NAME_TO_PROTOCOL:
        raise ValueError(f"unknown protocol {name!r} (try: {', '.join(NAME_TO_PROTOCOL)})")
    return NAME_TO_PROTOCOL[key]


def sync_host_table_from_mcu(slots: List[dict]) -> None:
    global _FIRMWARE_VERIFIED
    for entry in slots:
        slot = int(entry["slot"])
        if slot < 0 or slot >= ACTUATOR_COUNT:
            continue
        cfg = _HOST_TABLE[slot]
        cfg.bus = int(entry["bus"])
        cfg.protocol = Protocol(int(entry["protocol"]))
        cfg.motor_id = int(entry["motor_id"]) & 0xFF
        cfg.enabled = bool(entry.get("enabled", True))
    _FIRMWARE_VERIFIED = True


def refresh_host_table_from_mcu(
    session: "PcbSession",
    *,
    quiet: bool = False,
) -> bool:
    """Pull actuator_table[] from MCU (CFG GET) into the host mirror used by teleop."""
    from .plugins import plant_config as cfg_pdu

    try:
        slots = cfg_pdu.fetch_table(session)
        sync_host_table_from_mcu(slots)
        if not quiet:
            print("  MCU actuator table synced for teleop.")
        return True
    except (RuntimeError, TimeoutError) as exc:
        if not quiet:
            print(
                f"  WARNING: could not read MCU actuator table ({exc}) — "
                "using host defaults (run: control_hub.py config show --port COMx)."
            )
        return False


def apply_host_config(
    slot: int,
    *,
    bus: Optional[int] = None,
    protocol: Optional[Protocol] = None,
    motor_id: Optional[int] = None,
    master_id: Optional[int] = None,
    enabled: Optional[bool] = None,
    persist: bool = False,
    session: Optional["PcbSession"] = None,
) -> ActuatorSlotConfig:
    cfg = _HOST_TABLE[slot]
    if bus is not None:
        cfg.bus = normalize_can_bus(bus)
    if protocol is not None:
        cfg.protocol = protocol
    if motor_id is not None:
        cfg.motor_id = motor_id & 0xFF
    if master_id is not None:
        cfg.master_id = master_id & 0xFF
    if enabled is not None:
        cfg.enabled = enabled

    if session is not None:
        from .plugins import plant_config as cfg_pdu

        slots = cfg_pdu.fetch_table(session)
        sync_host_table_from_mcu(slots)

        cfg = _HOST_TABLE[slot]
        if bus is not None:
            cfg.bus = normalize_can_bus(bus)
        if protocol is not None:
            cfg.protocol = protocol
        if motor_id is not None:
            cfg.motor_id = motor_id & 0xFF
        if master_id is not None:
            cfg.master_id = master_id & 0xFF
        if enabled is not None:
            cfg.enabled = enabled

        cfg_pdu.apply_slot(session, cfg)
        if persist:
            time.sleep(0.05)
            cfg_pdu.save_nvm(session)
        slots = cfg_pdu.fetch_table(session)
        sync_host_table_from_mcu(slots)
    elif persist:
        raise RuntimeError("--persist requires an open MCU session (--port)")
    return _HOST_TABLE[slot]


def format_table(*, source: str = "host mirror") -> str:
    lines = [f"Actuator table ({source}, {ACTUATOR_COUNT} slots):"]
    if not _FIRMWARE_VERIFIED and source == "host mirror":
        lines.append("  (not verified against MCU — run: config show --port COMx)")
    for cfg in _HOST_TABLE:
        en = "on" if cfg.enabled else "off"
        lines.append(
            f"  slot {cfg.slot}: {cfg.protocol_name:10s}  "
            f"CH{cfg.bus}  id=0x{cfg.motor_id:02X}  master=0x{cfg.master_id:02X}  {en}"
        )
    return "\n".join(lines)


def plant_actuator_table_pairs() -> Tuple[Tuple[int, int], ...]:
    """(bus, motor_id) per slot — compat with legacy scripts."""
    return tuple((c.bus, c.motor_id) for c in _HOST_TABLE)
