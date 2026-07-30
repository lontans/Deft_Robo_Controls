"""Apply / verify YAM product actuator CFG (RAM)."""
from __future__ import annotations

from contextlib import contextmanager
from typing import TYPE_CHECKING, Dict, Iterator, List, Sequence, Tuple

from deft_controls_sdk.link.exchange import ACTUATOR_COUNT
from deft_controls_sdk.vbeta.slots import yam_left_arm_rows, yam_product_rows

if TYPE_CHECKING:
    from deft_controls_sdk import ControlsPcbHub


def _row_tuple(row) -> Tuple[int, bool, int, int, int]:
    if isinstance(row, dict):
        return (
            int(row.get("bus", 0)),
            bool(row.get("enabled", False)),
            int(row.get("protocol", 0)),
            int(row.get("motor_id", 0)),
            int(row.get("master_id", 0)),
        )
    return (
        int(getattr(row, "bus", 0)),
        bool(getattr(row, "enabled", False)),
        int(getattr(row, "protocol", 0)),
        int(getattr(row, "motor_id", 0)),
        int(getattr(row, "master_id", 0)),
    )


def _table_matches(table: Sequence, expect: Sequence[Tuple[int, bool, int, int, int]]) -> bool:
    if len(table) < ACTUATOR_COUNT:
        return False
    for i in range(ACTUATOR_COUNT):
        bus, en, proto, mid, master = _row_tuple(table[i])
        eb, ee, ep, em, emas = expect[i]
        if (bus, en, proto, mid) != (eb, ee, ep, em):
            return False
        if ee and ep == 3 and master != emas:
            return False
    return True


def table_matches_yam(table: Sequence) -> bool:
    return _table_matches(table, yam_product_rows())


def table_matches_yam_left(table: Sequence) -> bool:
    return _table_matches(table, yam_left_arm_rows())


@contextmanager
def pause_plant_stream(hub: "ControlsPcbHub") -> Iterator[None]:
    """Pause plant TX/FB drain around CFG / DEBUG exchange_raw calls.

    Canonical implementation: ``deft_controls_sdk.debug.stream_pause``.
    """
    from deft_controls_sdk.debug.stream_pause import (
        pause_plant_stream as _pause,
    )

    with _pause(hub):
        yield


def _apply_rows(
    hub: "ControlsPcbHub",
    expect: Sequence[Tuple[int, bool, int, int, int]],
    *,
    label: str,
    force: bool,
    persist: bool,
    quiet: bool,
) -> Dict[int, List[int]]:
    with pause_plant_stream(hub):
        table = hub.debug.cfg_get_table()
        matches = _table_matches(table, expect)
        if matches and not force:
            if not quiet:
                print(f"CFG already matches {label} ({ACTUATOR_COUNT} slots)")
        else:
            if not quiet:
                print(f"Applying {label} (RAM)" + (" + persist" if persist else ""))
            for slot, (bus, enabled, proto, mid, master) in enumerate(expect):
                hub.debug.cfg_set_slot(
                    slot=slot,
                    bus=bus,
                    protocol=proto,
                    motor_id=mid,
                    master_id=master,
                    enabled=enabled,
                    persist=persist,
                )
            table = hub.debug.cfg_get_table()

    by_bus: Dict[int, List[int]] = {b: [] for b in range(1, 7)}
    for slot, row in enumerate(table[:ACTUATOR_COUNT]):
        bus, enabled, _p, _m, _mas = _row_tuple(row)
        if enabled and 1 <= bus <= 6:
            by_bus[bus].append(slot)
    if not quiet:
        for b in range(1, 7):
            print(f"  CH{b}: {len(by_bus[b])} slots -> {by_bus[b]}")
    return by_bus


def ensure_yam_product_cfg(
    hub: "ControlsPcbHub",
    *,
    force: bool = False,
    persist: bool = False,
    quiet: bool = False,
) -> Dict[int, List[int]]:
    """RAM-apply full YAM product CFG if needed. Slot 20 (lift) stays disabled."""
    return _apply_rows(
        hub,
        yam_product_rows(),
        label="YAM product CFG",
        force=force,
        persist=persist,
        quiet=quiet,
    )


def ensure_yam_left_arm_cfg(
    hub: "ControlsPcbHub",
    *,
    force: bool = False,
    persist: bool = False,
    quiet: bool = False,
) -> Dict[int, List[int]]:
    """RAM-apply left-arm-only CFG (CH1 slots 0–6); disable CH2+ for bench work."""
    return _apply_rows(
        hub,
        yam_left_arm_rows(),
        label="YAM left-arm-only CFG",
        force=force,
        persist=persist,
        quiet=quiet,
    )
