"""PDU / kill-link host identity (policy flags; wire helpers stay in pdb/)."""
from __future__ import annotations

# Host listen_pdu default for bench (ignore stale kill) vs product peer present.
DEFAULT_BENCH_LISTEN_PDU = False
DEFAULT_PRODUCT_LISTEN_PDU = True

__all__ = [
    "DEFAULT_BENCH_LISTEN_PDU",
    "DEFAULT_PRODUCT_LISTEN_PDU",
]
