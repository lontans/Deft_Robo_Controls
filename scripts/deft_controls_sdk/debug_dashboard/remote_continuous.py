"""Launch/stop ``legacy/yam_continuous_all.py`` on the Jetson from the dashboard.

Mirrors ``scripts/pcb_lab/legacy/launch_continuous.py``'s proven sync-then-launch pattern
(sftp the driver + the SDK modules it needs, clear stale process/flag state,
start in the background) but as a *persistent* run the operator starts and
stops from the GUI, rather than that script's fixed-50s prove-and-autokill
harness. Connection details (host/user/password) match that script's env-var
convention so both point at the same bench without extra config.

Local copies live under ``scripts/pcb_lab/legacy/``; remote layout stays flat under
``REMOTE_DIR`` (basename only) so existing Jetson paths keep working until
PlantProxy / pcb_lab replaces this path.

This module never runs on import or dashboard startup — only an explicit
POST to ``/api/continuous/launch`` or ``/api/continuous/stop`` calls into it,
and ``AppState`` holds the launcher/stopper as swappable callables
(``continuous_launcher`` / ``continuous_stopper``) specifically so the HTTP
contract can be tested end-to-end offline without importing paramiko or
touching a real host — see test_deft_controls_sdk_dashboard.py.
"""
from __future__ import annotations

import os
from pathlib import Path

HOST = os.environ.get("JETSON_HOST", "192.168.50.48")
USER = os.environ.get("JETSON_USER", "deft-robotics")
PASSWORD_ENV = "JETSON_PASS"
REMOTE_DIR = "/home/deft-robotics/controls_pcb/scripts"
# .../scripts/deft_controls_sdk/debug_dashboard/remote_continuous.py -> .../scripts
LOCAL_SCRIPTS_DIR = Path(__file__).resolve().parents[2]

# Fixed list, not a full repo push — keeps the sync fast and matches exactly
# what yam_continuous_all.py needs at import/run time (same core set as
# legacy/launch_continuous.py). Does not overwrite dashboard GUI sources.
# Each entry: (local path under scripts/, remote path under REMOTE_DIR).
SYNC_FILES = (
    ("pcb_lab/legacy/yam_continuous_all.py", "yam_continuous_all.py"),
    ("pcb_lab/legacy/pdb_uart_sim.py", "pdb_uart_sim.py"),
    ("pcb_lab/legacy/stop_can.py", "stop_can.py"),
    ("deft_controls_sdk/debug/robstride.py", "deft_controls_sdk/debug/robstride.py"),
    ("deft_controls_sdk/debug/robstride_motion.py", "deft_controls_sdk/debug/robstride_motion.py"),
    ("deft_controls_sdk/debug/rs02_motion.py", "deft_controls_sdk/debug/rs02_motion.py"),
    ("deft_controls_sdk/config/actuator.py", "deft_controls_sdk/config/actuator.py"),
    (
        "deft_controls_sdk/config/yam_bench_clear_left.py",
        "deft_controls_sdk/config/yam_bench_clear_left.py",
    ),
    (
        "deft_controls_sdk/config/yam_limits.py",
        "deft_controls_sdk/config/yam_limits.py",
    ),
    ("deft_controls_sdk/telemetry/cache.py", "deft_controls_sdk/telemetry/cache.py"),
)


class ContinuousLaunchError(RuntimeError):
    """Raised for any failure to reach/launch on the Jetson — never for a
    problem with the run itself once started (that shows up in telemetry,
    not here)."""


def _connect():
    try:
        import paramiko
    except ImportError as exc:
        raise ContinuousLaunchError(
            "paramiko not installed on this host — pip install paramiko"
        ) from exc
    password = os.environ.get(PASSWORD_ENV)
    if not password:
        raise ContinuousLaunchError(
            f"{PASSWORD_ENV} not set — export it before Launch/Stop continuous"
        )
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(HOST, username=USER, password=password, timeout=15)
    return client


def default_launcher(*, duration_s: float = 0.0, extra_args: str = "") -> dict:
    """Sync files, clear stale process/flag state, start
    ``yam_continuous_all.py`` in the background over SSH.

    ``duration_s <= 0`` (default) omits ``--duration`` — the run continues
    until the operator stops it (Soft-kill Park in follow mode, or
    Stop continuous), not a timer. Pass a positive value to bound it like the
    original prove harness does.
    """
    client = _connect()
    try:
        _, out, _ = client.exec_command(
            "pkill -9 -f yam_continuous_all.py || true; "
            f"rm -f {REMOTE_DIR}/.deft_session/soft_kill_request; sleep 0.5"
        )
        out.read()

        sftp = client.open_sftp()
        try:
            for local_rel, remote_rel in SYNC_FILES:
                local = LOCAL_SCRIPTS_DIR / local_rel.replace("/", os.sep)
                if local.is_file():
                    sftp.put(str(local), f"{REMOTE_DIR}/{remote_rel}")
        finally:
            sftp.close()

        args = "--record"
        if duration_s > 0:
            args += f" --duration {duration_s}"
        if extra_args:
            args += f" {extra_args}"

        transport = client.get_transport()
        assert transport is not None
        chan = transport.open_session()
        chan.exec_command(
            f"cd {REMOTE_DIR} && nohup python3 -u yam_continuous_all.py {args} "
            f">/tmp/yam_cont.log 2>&1 </dev/null &"
        )
        chan.close()
        return {"ok": True, "host": HOST, "args": args, "log": "/tmp/yam_cont.log"}
    finally:
        client.close()


def default_stopper() -> dict:
    """Hard-stop fallback for when the follow-mode ``soft_kill_request`` flag
    isn't being polled (e.g. process wedged) — pkill + blank CAN state,
    mirroring ``stop_can.py``. Prefer Soft-kill Park (writes the flag
    continuous already watches every tick) when the process is responsive;
    this is the belt-and-suspenders path for when it isn't."""
    client = _connect()
    try:
        _, out, _ = client.exec_command(
            "pkill -9 -f yam_continuous_all.py || true; sleep 0.5; "
            f"cd {REMOTE_DIR} && python3 -u stop_can.py"
        )
        text = out.read().decode("utf-8", "replace")
        return {"ok": True, "output": text}
    finally:
        client.close()
