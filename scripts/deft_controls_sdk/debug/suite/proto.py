"""Protocol name ↔ int for CFG CLI.

Canonical constants: ``deft_controls_sdk.config`` (PROTO_*).
Programmatic parse: ``config.typed_profiles.parse_protocol``.
"""
from __future__ import annotations

from typing import Dict

from deft_controls_sdk.config import (
    PROTO_CUBEMARS,
    PROTO_DAMIAO,
    PROTO_NONE,
    PROTO_ROBSTRIDE,
    PROTO_ZEROERR,
)

PROTO_BY_NAME: Dict[str, int] = {
    "none": PROTO_NONE,
    "off": PROTO_NONE,
    "robstride": PROTO_ROBSTRIDE,
    "rs": PROTO_ROBSTRIDE,
    "cubemars": PROTO_CUBEMARS,
    "cm": PROTO_CUBEMARS,
    "damiao": PROTO_DAMIAO,
    "dm": PROTO_DAMIAO,
    "zeroerr": PROTO_ZEROERR,
    "ze": PROTO_ZEROERR,
}

PROTO_NAMES: Dict[int, str] = {
    PROTO_NONE: "none",
    PROTO_ROBSTRIDE: "robstride",
    PROTO_CUBEMARS: "cubemars",
    PROTO_DAMIAO: "damiao",
    PROTO_ZEROERR: "zeroerr",
}


def parse_protocol(value: str) -> int:
    key = (value or "").strip().lower()
    if key in PROTO_BY_NAME:
        return PROTO_BY_NAME[key]
    if key.startswith("0x") or key.isdigit():
        return int(key, 0)
    known = ", ".join(sorted(PROTO_BY_NAME))
    raise ValueError(f"unknown protocol {value!r}; use int or one of: {known}")


def protocol_name(protocol: int) -> str:
    return PROTO_NAMES.get(int(protocol), f"proto({protocol})")
