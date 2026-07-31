# Integration — SDK, vbeta, pcb_lab



## Shape



```text

Desk / CFG / flash     →  debug_dashboard or hub.debug  → Hub → USB

Lab board (USB/DFU)    →  pcb_lab                       → Soft-DFU / bandwidth hub

Lab peripherals CLI    →  pcb_lab.debug {show|set|test} → HostProxy → Hub → USB

Notebooks / scripts    →  deft_controls_sdk.*           → HostProxy.actions → Hub → USB

Product teleop         →  deft_vbeta (parent)           → HostProxy.set_section → Hub → USB

ROS teleop             →  ControlsPcbHostNode           → HostProxy.set_section → Hub → USB

```



| Piece | Role |

|-------|------|

| `ControlsPcbHub` | Wire: slots, stream, `hub.debug`, telemetry |

| `HostProxy` | Session + **section demux** into held 694B CMDH; `arm_plant` / `disarm_plant` |

| `config/` | Static product assembly + YAM CFG preset (`yam_product_rows`) |

| `actions/` | **Lab/notebooks only** — mount/apply/clear helpers |

| `debug/` | CFG / discover / cal / Soft-DFU |

| `ros/` | Optional ROS 2 adapter (`set_section` + MIT `5*n` commands) |

| `pcb_lab/` | **CLI only** — board USB/flash + thin alias of `debug.suite` |

| `link/` | USB bytes + types |



**Rule:** notebooks and scripts import `deft_controls_sdk`. `pcb_lab` is for `python -m` only.



**Ownership:** `controls_pcb` is a submodule of **deft_vbeta**. Product YAM drivers live in the parent repo; this SDK exposes `HostProxy` / `config` / `actions` / `debug` only.



## Product demux (fixed sections)



HostProxy default assembly = `yam_product_assembly()`. Section → slots is platform truth:



| Section | Slots (order) | Notes |

|---------|---------------|--------|

| `left_arm` | 0–6 | Damiao CH1 |

| `right_arm` | 7–13 | Damiao CH2 |

| `base_wheel_1` | 14, 17 | Center steer + drive |

| `base_wheel_2` | 15, 18 | Right steer + drive |

| `base_wheel_3` | 16, 19 | Left steer + drive |

| `torso` | 20 | CFG disabled in preset |

| `neck` (servo) | 0, 1 | DXL pitch, yaw |



Product path:



```text

deft_vbeta authors ActuatorDesire (p/v/kp/kd/τ)

        → HostProxy.set_section(name, desires)

        → merge into held CMDH → stream

```



Do **not** use `proxy.actions` on the product path. Lab notebooks may.



CFG: `config.ensure_yam_product_cfg` / `HostProxy.connect(..., apply_yam_cfg=True, mode="debug")`. Teleop runs `mode="bandwidth"` with the preset already on the board.



## ROS command wire



`python -m deft_controls_sdk.ros --port COMx --profile product`



- Command: `actuators/<section>/command` — `Float64MultiArray` length `5 * n`, interleaved `[p, v, kp, kd, τ] * n`

- State: `actuators/<section>/state` — `JointState` (position/velocity/effort)

- Product default `listen_pdu=True`



## pcb_lab charter



| Entry | Role |

|-------|------|

| `python -m pcb_lab` | Board: scan / status / leave / flash / images / build |

| `python -m pcb_lab.debug show\|set\|test` | CLI alias of `deft_controls_sdk.debug.suite` |



## Stacks



**Lab board:** `python -m pcb_lab` → Soft-DFU / bandwidth status



**Lab peripherals CLI:** `python -m pcb_lab.debug {show|set|test}`



**Notebooks:** `HostProxy` + `actions` + `config` + `hub.debug`



**Product (deft_vbeta):** section desires → `HostProxy.set_section` → Hub



**ROS:** `ControlsPcbHostNode` → same demux



## deft_vbeta follow-ups (parent repo)



1. Depend on submodule SDK; one `HostProxy` or one ROS node owns COM.

2. Arms/base/torso/neck call `set_section` (or publish MIT topics); no per-peripheral COM.

3. Keep product drivers in parent deft_vbeta; do not use `proxy.actions` on the product path.

4. Bringup may apply YAM CFG in `mode=debug`; teleop is `bandwidth`.

5. Keep YAMAIMobile above HostProxy/ROS — no plant slot packing there.


