# Deprecated DEBUG mailbox overlays + leftover smoke probes

## Status (ADR-004)

**Canonical host path:** debug lanes (`DL\x01` + fixed 32 B lanes) on
`DBGC`/`DBGF`. Host builders (`build_rs2_*`, `build_dm_*`, `build_cfg_*`)
**always** emit lanes — the legacy offset-630 TX path is removed from the SDK.

Firmware may still accept the legacy mailbox when the debug-lanes header is
absent. Soft-DFU prefers `stm32_mode=2` on a plant frame; mailbox tag `DFU!`
remains a deprecated alias.

Parsers accept either debug lanes or the legacy mailbox on inbound frames
(dual-path for older FW replies).

**Hardware inventory:** `python -m pcb_lab.debug test --inventory` (TUI or `--preset bench`)
— actuators (DEBUG discover; **ID ranges required**) + servos + PDU wire.
SDK: `hub.debug.inventory(preset=…)` / `run_inventory(proxy, …)`.

## Leftover smoke / out-of-lane probes (later cull)

These still exist in firmware (`diag.h` / `diag_*.c`) but are **not** first-class
debug lanes and should be deleted or re-homed after bring-up:

| Kind / tag | Notes |
|------------|--------|
| `PLANT_DIAG_PROBE_MCP_SMOKE` (20) | SPI rail smoke — CH4–6 bring-up only |
| `PLANT_DIAG_PROBE_MCP_WAKE` (21) | same |
| `PLANT_DIAG_PROBE_MCP_DISABLE` (22) | same |
| RS2 `PROBE_FULL` / `CTRL_*` / `CTRL_FAST` | Heavy control smoke; discover uses ENABLE_ONLY + PROMISC |
| Thermo / UART-bridge tags in mailbox | Optional; PDU lab lane 6 is the long-term home |
| CubeMars / ZeroErr lanes 1–2 | Reserved; no host packers yet |

Do not add new host callers for MCP smoke or FULL/CTRL kinds on the plant
mailbox. Prefer lane 0 (RS) / lane 3 (DM) / lane 7 (CFG) via `mode="debug"`.

## Bandwidth vs debug

```python
ControlsPcbHub.connect(port, mode="bandwidth")  # no debug-lanes frame; timing OK
ControlsPcbHub.connect(port, mode="debug")      # hub.debug.* allowed
```

`show --bandwidth` alone connects with `mode="bandwidth"`. Mixing CFG/PCB TUI
uses `mode="debug"`.
