"""Apply / verify YAM product actuator CFG (RAM)."""
from __future__ import annotations

from typing import TYPE_CHECKING, Dict, List, Sequence, Tuple

from deft_controls_sdk.link.exchange import ACTUATOR_COUNT
from deft_controls_sdk.vbeta.slots import yam_product_rows

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


def table_matches_yam(table: Sequence) -> bool:
    expect = yam_product_rows()
    if len(table) < ACTUATOR_COUNT:
        return False
    for i in range(ACTUATOR_COUNT):
        bus, en, proto, mid, master = _row_tuple(table[i])
        eb, ee, ep, em, emas = expect[i]
        if (bus, en, proto, mid) != (eb, ee, ep, em):
            return False
        # master_id: Damiao rows must match; others ignore
        if ee and ep == 3 and master != emas:
            return False
    return True


def ensure_yam_product_cfg(
    hub: "ControlsPcbHub",
    *,
    force: bool = False,
    persist: bool = False,
    quiet: bool = False,
) -> Dict[int, List[int]]:
    """RAM-apply YAM product CFG if needed. Slot 20 (lift) stays disabled."""
    table = hub.debug.cfg_get_table()
    expect = yam_product_rows()
    if table_matches_yam(table) and not force:
        if not quiet:
            print(f"CFG already matches YAM product layout ({ACTUATOR_COUNT} slots)")
    else:
        if not quiet:
            print("Applying YAM product CFG (RAM)" + (" + persist" if persist else ""))
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
