# scripts/legacy/tmp_runners — archived Jetson SSH deploy-and-run wrappers

Five near-duplicate Windows-host Paramiko scripts that each: kill stale
`_tmp_base_bus56_lab.py` / `yam_continuous_all.py` on the Jetson, (re)start
`pdb_uart_sim.py`, SFTP-deploy a lab script, run one specific remote check,
and pull back the log. Archived here (not deleted, not folded) per
[`docs/scripts-hygiene.md`](../../../docs/scripts-hygiene.md) — fold into one
parametrized remote-runner helper when someone next needs this path.

| File | What it ran remotely |
|------|----------------------|
| `_tmp_run_prove360.py` | `_tmp_base_bus56_lab.py --prove-360` |
| `_tmp_poll_prove360.py` | polls `/tmp/prove360.log` for `_tmp_run_prove360.py` |
| `_tmp_run_tx_smoke.py` | `_tmp_base_bus56_lab.py --tx-smoke` |
| `_tmp_run_fix74.py` | `_tmp_base_bus56_lab.py --fix-74` |
| `_tmp_check_recording.py` | inspects `.deft_session/recordings/*.ndjson` histogram |

None of these are imported by other scripts — safe to read individually if
reviving one path, or to delete outright once the fold-in happens.
