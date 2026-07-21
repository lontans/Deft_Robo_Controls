"""Shared telemetry contract — hub publishes, scripts and debug_dashboard read.

Three different jobs (do not conflate them):

  1. **Plant TX** — send→sleep at stream_hz (no JSON).
  2. **Live UI mirror** — latest ``SessionState`` in RAM; optional ``state.json``
     when ``persist=True`` (dashboard). Latest-wins, lossy, for operators.
  3. **Accurate log (opt-in)** — ``start_recording()`` / ``log_feedback()`` writes
     compact ``deft_fb_v1`` NDJSON (+ optional raw frame) on the disk thread.

Default session dir: ``<cwd>/.deft_session/`` (override with session_dir=).
"""
from .cache import SessionState, TelemetryCache, default_session_dir
from .fb_log import FB_LOG_SCHEMA, build_fb_log_record

__all__ = [
    "SessionState",
    "TelemetryCache",
    "default_session_dir",
    "FB_LOG_SCHEMA",
    "build_fb_log_record",
]
