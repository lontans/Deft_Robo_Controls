"""pytest path setup for legacy tests — add scripts/legacy and scripts/ to sys.path."""
from __future__ import annotations

import os
import sys

_LEGACY = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SCRIPTS = os.path.dirname(_LEGACY)
for p in (_LEGACY, _SCRIPTS):
    if p not in sys.path:
        sys.path.insert(0, p)
