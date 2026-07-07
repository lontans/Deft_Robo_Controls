"""Schematic CAN bus helpers (CH1..CH6)."""
from __future__ import annotations

from typing import Dict, FrozenSet

from .diag_pdu import RS2_COMM_NAMES
from .schema import MAX_CAN_BUS, MIN_CAN_BUS

MCP_CAN_BUSES: FrozenSet[int] = frozenset({4, 5, 6})

_CAN_PINS: Dict[int, str] = {
    1: "PB8/9 FDCAN1",
    2: "PA8/PA15 FDCAN3",
    3: "PB12/13 FDCAN2",
    4: "PB11 MCP SPI-CAN",
    5: "PB1 MCP SPI-CAN",
    6: "PA4 MCP SPI-CAN",
}

_ACTIVITY_LED: Dict[int, str] = {
    1: "PC7",
    2: "PC6",
    3: "PB15",
    4: "PB14",
    5: "PB2",
    6: "PC5",
}


def normalize_can_bus(bus: int) -> int:
    b = int(bus)
    if b < MIN_CAN_BUS or b > MAX_CAN_BUS:
        raise ValueError(f"CAN bus must be {MIN_CAN_BUS}..{MAX_CAN_BUS}, got {b}")
    return b


def can_bus_label(bus: int) -> str:
    b = normalize_can_bus(bus)
    return f"CH{b} ({_CAN_PINS[b]})"


def can_activity_led(bus: int) -> str:
    b = normalize_can_bus(bus)
    return f"{_ACTIVITY_LED[b]} (CH{b} ACT)"


def probe_target_label(motor_id: int, bus: int) -> str:
    return f"0x{motor_id & 0xFF:02X} on {can_bus_label(bus)}"


def print_can_bus_note(bus: int) -> None:
    b = normalize_can_bus(bus)
    led = can_activity_led(b)
    if b in MCP_CAN_BUSES:
        print(f"Probe target: MCP2518 {can_bus_label(b)}; CAN activity → {led}.")
        print("MCP: longer per-probe listen; bus-off clears at RS2 session begin.")
    else:
        print(f"Probe target: {can_bus_label(b)}; CAN activity → {led}.")


def rs2_comm_label(comm_mode: int) -> str:
    name = RS2_COMM_NAMES.get(comm_mode)
    if name is None:
        return f"0x{comm_mode:02X}"
    return f"0x{comm_mode:02X} {name}"
