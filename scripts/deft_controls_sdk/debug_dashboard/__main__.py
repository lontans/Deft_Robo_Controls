"""python -m deft_controls_sdk.debug_dashboard [--port COM5]

--port is optional: omit it to start disconnected and pick a port + hit
Connect in the browser; pass it to auto-connect at launch (old workflow).
"""
from __future__ import annotations

import argparse
import os
import sys
import time
import webbrowser

# Ensure scripts/ is on path so deft_controls_sdk resolves
_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from deft_controls_sdk.debug_dashboard.app import AppState, serve  # noqa: E402


def _status_block(snap) -> str:
    """Paste-friendly multi-line status (legacy teleop style) — not \\r spam."""
    return (
        f"{snap.grade}  {snap.summary}\n"
        f"  link   fb_hz={snap.fb_hz or 0:.1f}  age={snap.age_s or 0:.2f}s  "
        f"ack={snap.ack_seq}  ack_lag={snap.stream_ack_lag}  "
        f"tick={snap.tick}  block={snap.plant_block_name}\n"
        f"  host   tx_hz={snap.stream_tx_hz or 0:.1f}  "
        f"tx_gap_p95={snap.stream_tx_gap_p95_ms or 0:.1f}ms  "
        f"tx_gap_max={snap.stream_tx_gap_max_ms or 0:.1f}ms  "
        f"loop={snap.stream_loop_ms or 0:.1f}ms  "
        f"send={snap.stream_send_ms or 0:.1f}  "
        f"poll={snap.stream_poll_ms or 0:.1f}  "
        f"credit_wait={snap.stream_credit_wait_ms or 0:.1f}  "
        f"pub={snap.stream_publish_ms or 0:.1f}  "
        f"(idle: fb flood + ack_lag~0. MCP Apply should stay near that after plant MCP non-block fix.)\n"
        f"  mcu    act_lap={snap.lap_ms}  act_peak={snap.lap_max_ms}  "
        f"periph_lap={snap.periph_lap_ms}  periph_peak={snap.periph_lap_max_ms}  "
        f"pend={snap.ticks_pending}  svd={snap.svd_present}\n"
    )


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Serve the localhost controller + telemetry UI")
    p.add_argument("--port", default=None, help="Serial port (e.g. COM5) — omit to connect from the browser")
    p.add_argument(
        "--http-port",
        type=int,
        default=8766,
        help="UI bind port (default 8766 — avoids pdb_uart_sim --control-port 8765)",
    )
    p.add_argument(
        "--hz",
        type=float,
        default=100.0,
        help="Plant stream TX rate (default 100; continuous mouse uses ~125)",
    )
    p.add_argument(
        "--telemetry-hz",
        type=float,
        default=10.0,
        help="UI/state.json publish rate (coalesced latest frame; keep << --hz)",
    )
    p.add_argument("--session-dir", default=None, help="Directory for state.json (default: ./.deft_session)")
    p.add_argument("--no-browser", action="store_true")
    args = p.parse_args(argv)

    url = f"http://127.0.0.1:{args.http_port}/"
    state = AppState(
        session_dir=args.session_dir,
        stream_hz=args.hz,
        telemetry_hz=args.telemetry_hz,
    )
    print(f"state.json → {state.telemetry.state_path}")
    print(f"plant TX {args.hz:.0f} Hz · telemetry publish {args.telemetry_hz:.0f} Hz (latest-wins)")

    if args.port:
        peer = state.peer_com_owner()
        if peer is not None:
            print(
                f"Not opening {args.port}: peer already owns COM via {peer['path']} "
                f"(port={peer.get('port') or '?'}, age={peer['age_s']:.1f}s).\n"
                "UI will follow state.json — stop the peer before Connect.",
                flush=True,
            )
        else:
            print(
                f"Connecting {args.port} (mode=debug) — entering Active frozen "
                "(soft_kill ON: hold-at-torque, NORMAL + plant_apply=1)..."
            )
            state.connect(args.port)
            print("Connected, Active, frozen. Use Release soft-kill in the UI before commanding motion.")
    else:
        sp = state.telemetry.state_path
        print(
            "Not connected to COM — UI follows state.json when present:\n"
            f"  {sp}\n"
            "If yam_continuous_all is writing that file, leave Connect alone.\n"
            "Connect enters Active frozen (soft_kill ON) by default — safe; Release soft-kill arms motion.\n"
            "Note: pdb_uart_sim control panel is :8765 — this UI defaults to :8766.",
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
                # Mirror /api/state follow path so the terminal isn't stuck on
                # an empty in-memory cache while a peer writes state.json.
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
