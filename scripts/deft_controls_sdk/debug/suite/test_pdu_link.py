"""PDU / kill-link prove — ``mode=debug`` pdb wire + listen_pdu policy."""
from __future__ import annotations

import argparse
import json
import time
from typing import Any, Dict, Optional

from deft_controls_sdk.host_proxy import HostProxy


def run_pdu_link_test(args: argparse.Namespace) -> int:
    stream_hz = float(getattr(args, "stream_hz", 200.0))
    tel = getattr(args, "telemetry_hz", None)
    telemetry_hz = float(tel) if tel is not None else stream_hz
    require_peer = bool(getattr(args, "peer", False))
    # Host listen_pdu for this prove: CLI --listen-pdu or force when --peer.
    listen = bool(getattr(args, "listen_pdu", False) or require_peer)

    print(
        f"test --pdu-link  mode=debug  host_listen_pdu={listen}  "
        f"require_peer={require_peer}  (no timing gates)"
    )

    with HostProxy.connect(
        getattr(args, "port", None),
        stream_hz=stream_hz,
        telemetry_hz=telemetry_hz,
        armed=False,
        listen_pdu=listen,
        mode="debug",
    ) as proxy:
        hub = proxy.hub
        # Brief stream so plant FB (kill bytes) can arrive.
        time.sleep(0.35)

        nvm_listen: Optional[bool] = None
        nvm_error: Optional[str] = None
        try:
            periph = dict(hub.debug.cfg_get_periph())
            nvm_listen = bool(periph.get("listen_pdu", False))
        except Exception as exc:  # noqa: BLE001
            nvm_error = str(exc)

        wire: Optional[Dict[str, Any]] = None
        wire_error: Optional[str] = None
        try:
            pdb = hub.pdb_status()
            if pdb is not None:
                wire = {
                    "ok": True,
                    "kill_state": int(pdb.kill_state),
                    "kill_state_name": pdb.kill_state_name,
                    "kill_reason": int(pdb.kill_reason),
                    "kill_reason_name": pdb.kill_reason_name,
                    "estop_sense": int(pdb.estop_sense),
                    "stale_failsafe": bool(pdb.stale_failsafe),
                }
        except Exception as exc:  # noqa: BLE001
            wire_error = str(exc)

        doctor = proxy.doctor()
        pdb_doc = doctor.get("pdb") or {}

        ok = True
        errors = []
        if nvm_error:
            ok = False
            errors.append(f"cfg_get_periph: {nvm_error}")
        if wire_error:
            ok = False
            errors.append(f"pdb_status: {wire_error}")
        if require_peer:
            if wire is None:
                ok = False
                errors.append("no pdb wire bytes yet (--peer requires peer)")
            elif wire.get("stale_failsafe"):
                ok = False
                errors.append("pdb wire stale_failsafe=True (--peer)")

        out = {
            "ok": ok,
            "host_listen_pdu": bool(hub.listen_pdu),
            "nvm_listen_pdu": nvm_listen,
            "wire": wire,
            "doctor_pdb": pdb_doc,
            "require_peer": require_peer,
            "errors": errors,
            "note": (
                "wire vs policy report — timing floors not applied. "
                "host listen_pdu gates soft-kill/LED; MCU may still publish kill bytes."
            ),
        }
        print(json.dumps(out, indent=2, default=str))
        return 0 if ok else 1
