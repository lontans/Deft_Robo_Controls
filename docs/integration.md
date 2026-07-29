# Integration — SDK, vbeta, pcb_lab

## Shape

```text
Desk / CFG / flash     →  debug_dashboard or hub.debug  → Hub → USB
Lab prove              →  pcb_lab                       → HostProxy → Hub → USB
YAM / teleop           →  vbeta (PcbArmDriver, …)      → HostProxy → Hub → USB
i2rt pcb:              →  bridge                        → HostProxy → Hub → USB
```

| Piece | Role |
|-------|------|
| `ControlsPcbHub` | Wire: slots, stream, `hub.debug`, telemetry |
| `HostProxy` | Platform demux: `left_arm` / `base` / … |
| `vbeta/` | YAM-shaped drivers on HostProxy |
| `debug/` | DEBUG toolkit behind `hub.debug` (CFG / discover / Soft-DFU) |
| `pcb_lab/` | Lab app + `tests/` + deprecated `legacy/` CLIs |
| `link/` | USB bytes + types |

## Stacks

**Lab:** `python -m pcb_lab doctor|hold|step|blank` → HostProxy  

**YAM:** `PcbArmDriver` → `PcbRobotSession` → HostProxy → Hub  

**i2rt:** bridge → HostProxy → Hub  
