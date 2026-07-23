"""Byte-stream framer for the 64 B PDB link — mirrors the controls-side
accumulate/resync shape in pdb_service_rx()/pdb_rx_resync() (App/Src/host/pdb_link.c)
so a host-side reader dropped mid-frame recovers the same way the MCU does.
"""
from __future__ import annotations

from typing import List, Optional

from .frame import FRAME_BYTES, is_frame_valid


class PdbFrameReader:
    """Feed arbitrary byte chunks; get back complete 64 B frames.

    Resync behavior: if the accumulated 64 B block doesn't validate against
    `expect_magic`, scan forward one byte at a time for the magic and slide
    it to the front (matching pdb_rx_resync()) instead of dropping the whole
    window — a single corrupt byte should cost one frame, not a full resync
    stall.
    """

    def __init__(self, expect_magic: int) -> None:
        self._expect_magic = expect_magic
        self._accum = bytearray()

    def feed(self, chunk: bytes) -> List[bytes]:
        if not chunk:
            return []
        self._accum.extend(chunk)
        out: List[bytes] = []
        while len(self._accum) >= FRAME_BYTES:
            candidate = bytes(self._accum[:FRAME_BYTES])
            if is_frame_valid(candidate, self._expect_magic):
                out.append(candidate)
                del self._accum[:FRAME_BYTES]
                continue
            shift = self._find_resync_offset()
            if shift is None:
                self._accum.clear()
                break
            del self._accum[:shift]
        return out

    def _find_resync_offset(self) -> Optional[int]:
        magic_bytes = self._expect_magic.to_bytes(4, "little")
        idx = self._accum.find(magic_bytes, 1)
        return idx if idx > 0 else None
