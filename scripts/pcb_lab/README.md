# pcb_lab

Board toolkit for the Controls PCB. Prefer the interactive menu; CLI subcommands stay available.

```powershell
cd scripts
python -m pcb_lab              # interactive menu
python -m pcb_lab -h

python -m pcb_lab scan
python -m pcb_lab status --port COM5
python -m pcb_lab leave
python -m pcb_lab flash
python -m pcb_lab images
python -m pcb_lab build
python -m pcb_lab show defaults

# HostProxy snapshot (mode=debug)
python -m pcb_lab doctor --port COM5

# CFG / PCB TUI (always mode=debug)
python -m pcb_lab.debug --port COM5 show --pcb
python -m pcb_lab.debug --port COM5 show          # CFG table
python -m pcb_lab.debug --port COM5 show --bandwidth

# Mode-disciplined proves (suite-owned; no vbeta imports)
python -m pcb_lab.debug test                     # Assembly workshop (default)
python -m pcb_lab.debug test --assembly bench
python -m pcb_lab.debug test --bandwidth
python -m pcb_lab.debug test --actuators         # narrower discover/CFG/motion
python -m pcb_lab.debug test --led --preset idle
python -m pcb_lab.debug test --servo
python -m pcb_lab.debug test --pdu-link
```

`HostProxy.connect` / `ControlsPcbHub.connect` default to **`mode="bandwidth"`**. Pass `mode="debug"` (or use `pcb_lab.debug`) for CFG / discover / debug-lanes RPC.

`status` proves USB duplex at ~200 Hz host TX and checks FB `stm32_mode` echo is **bandwidth** (not debug / soft_dfu).

## `pcb_lab.debug test` — mode rules

All `test` code lives under `deft_controls_sdk.debug.suite` (`pcb_lab.debug` is a thin alias). Suite tests must not import `vbeta.*`.

| Prove | Link mode | Pass gates |
|-------|-----------|------------|
| `--bandwidth` | **bandwidth** (dedicated connect) | `debug.metrics.measure_hold` (ack_lag / fb_hz / stm32_mode) |
| bare `test` | **debug** + plant CMDH | **Assembly workshop**: edit profiles, CFG apply/persist, nudge+NVM gate, operate (`spin` / `move_arm`) |
| `--actuators` / `--led` / `--servo` / `--pdu-link` | **debug** | functional / observe only — **no** timing floors (`--actuators` = discover/CFG/motion menu) |

```powershell
python -m pcb_lab.debug --port COM5 test
python -m pcb_lab.debug test --assembly bench --cfg-map bench
```

**Plant motion** (programmatic): prefer `LabRobot.connect(assembly=yam_product_assembly())` then `lab.actuators("left_arm")` → `ActuatorAction` from a typed `ActuatorProfile`. Cruise/jog: `actions.make_teleop_engine` / `spin_jog` / `move_arm_cruise` (dashboard can import the same later). Assemblies compose actuator/servo profiles separately (no mixed peripheral maps). HostProxy still uses the actuator demux `Profile` shim (`assembly.to_demux_profile()`).

**Inventory** (what’s plugged in) is a top-level command — not under bandwidth:

```bash
python -m pcb_lab inventory                         # TUI: pick buses + ID ranges
python -m pcb_lab inventory --preset bench --buses 5,6
python -m pcb_lab inventory --rs-range 0x70-0x75 --dm-range 1-8 --json
```

Actuator discover **requires** an ID range (`--preset` / `--rs-range` / TUI) — wide defaults are intentionally not used. Also samples neck servos and PDU kill-link wire.

`test --bandwidth` TUI nests under **virtual** (`rx_sim` ON) vs **hardware** (`rx_sim` OFF). Scenarios: `idle` / `ch1`…`ch6` / `fdcan` / `mcp` / `arms` / `all`. Non-interactive: `--virtual|--hardware --matrix`, `--scenario mcp`, `--hz-list 40,200,500`.

Debug sessions interleave `DBGC`/`DBGF` with plant frames and can inflate `ack_lag` / depress `fb_hz`. Timing proves always reconnect in bandwidth mode.

Bare `show` prints the CFG table. Live board view: `show --pcb`. Doctor JSON: `pcb_lab doctor` (programmatic `collect_status` remains for scripts).

## Layout

```text
pcb_lab/
  lab.py          # CLI + LabRobot (programmatic hold/step still available)
  board.py        # scan / status / flash / images / menu
  debug/          # alias → deft_controls_sdk.debug.suite
  tests/          # offline SDK + lab tests
```
