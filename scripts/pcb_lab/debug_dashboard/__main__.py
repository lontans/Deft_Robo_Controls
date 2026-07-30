"""python -m pcb_lab.debug_dashboard [--port COM5]

Thin lab entry for ``deft_controls_sdk.debug_dashboard`` with 200 Hz plant /
telemetry defaults (matches HostProxy / pcb_lab.debug).
"""
from __future__ import annotations

import argparse
import os
import sys
import time
import webbrowser

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from deft_controls_sdk.debug_dashboard.__main__ import _status_block  # noqa: E402
from deft_controls_sdk.debug_dashboard.app import AppState, serve  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        prog="pcb_lab.debug_dashboard",
        description="Serve the localhost controller + telemetry UI (lab entry)",
    )
    p.add_argument(
        "--port",
        default=None,
        help="Serial port (e.g. COM5) — omit to connect from the browser",
    )
    p.add_argument(
        "--http-port",
        type=int,
        default=8766,
        help="UI bind port (default 8766)",
    )
    p.add_argument(
        "--hz",
        type=float,
        default=200.0,
        help="Plant stream TX rate (default 200)",
    )
    p.add_argument(
        "--telemetry-hz",
        type=float,
        default=200.0,
        help="UI/state.json publish rate (default 200)",
    )
    p.add_argument(
        "--session-dir",
        default=None,
        help="Directory for state.json (default: scripts/.deft_session)",
    )
    p.add_argument("--no-browser", action="store_true")
    args = p.parse_args(argv)

    url = f"http://127.0.0.1:{args.http_port}/"
    state = AppState(
        session_dir=args.session_dir,
        stream_hz=float(args.hz),
        telemetry_hz=float(args.telemetry_hz),
        persist_telemetry=True,
    )
    print(f"state.json -> {state.telemetry.state_path}")
    print(
        f"plant TX {args.hz:.0f} Hz · telemetry publish {args.telemetry_hz:.0f} Hz "
        f"(persist_telemetry=on)"
    )

    if args.port:
        print(f"Connecting {args.port} in observe mode (DIAG_ONLY, no auto soft-kill)...")
        state.connect(args.port, mode="observe")
        print("Connected (observe). Use Enable control in the UI for NORMAL plant apply.")
    else:
        sp = state.telemetry.state_path
        print(
            "Not connected to COM — UI follows state.json when present:\n"
            f"  {sp}\n"
            "If continuous is writing that file, leave Connect alone.\n"
            "Connect (observe) is safe telemetry only; Enable control arms motors.",
            flush=True,
        )

    httpd = serve(state, http_port=args.http_port)
    print(f"UI: {url}")
    if not args.no_browser:
        try:
            webbrowser.open(url)
        except Exception:
            pass
    print("Ctrl+C to stop. Status lines below are paste-friendly (~1 Hz).\n")
    try:
        while True:
            time.sleep(1.0)
            if state.connected:
                snap = state.telemetry.snapshot()
                print(_status_block(snap), flush=True)
            else:
                try:
                    import json
                    from pathlib import Path

                    sp = Path(state.telemetry.state_path)
                    if sp.is_file():
                        d = json.loads(sp.read_text(encoding="utf-8"))
                        print(
                            f"FOLLOW  {d.get('grade', '?')}  "
                            f"fb_hz={d.get('fb_hz') or 0:.1f}  "
                            f"age={d.get('age_s') or 0:.2f}s  "
                            f"tick={d.get('tick')}  "
                            f"block={d.get('plant_block_name')}\n"
                            f"  file   {sp}\n",
                            flush=True,
                        )
                    else:
                        print(f"FOLLOW  waiting for {sp}\n", flush=True)
                except Exception as exc:
                    print(f"FOLLOW  read error: {exc}\n", flush=True)
    except KeyboardInterrupt:
        print("\nStopping…")
    finally:
        state.disconnect()
        httpd.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
