"""Plant actuator table CFG PDU — live RAM apply + flash NVM on MCU."""
from __future__ import annotations

import time
from typing import TYPE_CHECKING, List, Optional

from ..commands import build_cfg_command
from ..feedback import parse_cfg_feedback
from ..protocol import (
    CFG_OP_GET,
    CFG_OP_SAVE,
    CFG_OP_SET,
    CFG_STATUS_OK,
    IMAGE_BYTES,
)

if TYPE_CHECKING:
    from ..actuator_config import ActuatorSlotConfig
    from ..session import PcbSession


def _wait_cfg_response(
    session: "PcbSession",
    timeout_s: float,
    *,
    expect_op: Optional[int] = None,
) -> Optional[dict]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        frame = session.reader.pop()
        while frame is not None:
            if len(frame) == IMAGE_BYTES:
                pdu = frame[530:562]
                parsed = parse_cfg_feedback(pdu)
                if parsed is not None:
                    if expect_op is not None and parsed["op"] != expect_op:
                        frame = session.reader.pop()
                        continue
                    return parsed
            frame = session.reader.pop()
        time.sleep(0.005)
    return None


def exchange_cfg(
    session: "PcbSession",
    op: int,
    *,
    slot: int = 0,
    bus: int = 1,
    protocol: int = 0,
    motor_id: int = 0,
    master_id: int = 0,
    enabled: bool = True,
    timeout_s: float = 1.5,
) -> dict:
    frame = build_cfg_command(
        op,
        session.next_seq(),
        slot=slot,
        bus=bus,
        protocol=protocol,
        motor_id=motor_id,
        master_id=master_id,
        enabled=enabled,
    )
    session.write_command(frame)
    resp = _wait_cfg_response(session, timeout_s, expect_op=op)
    if resp is None:
        raise TimeoutError("no CFG response from MCU within timeout")
    if resp["status"] != CFG_STATUS_OK:
        raise RuntimeError(f"CFG op={op} failed: {resp['status_name']}")
    return resp


def fetch_table(session: "PcbSession", *, timeout_s: float = 1.5) -> List[dict]:
    return exchange_cfg(session, CFG_OP_GET, timeout_s=timeout_s)["slots"]


def apply_slot(
    session: "PcbSession",
    cfg: "ActuatorSlotConfig",
    *,
    timeout_s: float = 1.5,
) -> dict:
    master = cfg.master_id & 0xFF
    if int(cfg.protocol) == 3 and master == 0:
        master = 0xFF
    return exchange_cfg(
        session,
        CFG_OP_SET,
        slot=cfg.slot,
        bus=cfg.bus,
        protocol=int(cfg.protocol),
        motor_id=cfg.motor_id,
        master_id=master,
        enabled=cfg.enabled,
        timeout_s=timeout_s,
    )


def save_nvm(session: "PcbSession", *, timeout_s: float = 8.0, retries: int = 3) -> dict:
    last_err: Optional[Exception] = None
    for attempt in range(retries):
        try:
            return exchange_cfg(session, CFG_OP_SAVE, timeout_s=timeout_s)
        except (RuntimeError, TimeoutError) as exc:
            last_err = exc
            if attempt + 1 < retries:
                time.sleep(0.25)
    assert last_err is not None
    raise last_err
