# Architecture

## Overview

Two execution contexts share data through **staging buffers** — no malloc, no RTOS.

| Context | Rate | Entry | Job |
|---------|------|-------|-----|
| **Main loop** | As fast as `app_run()` spins | `app_run()` | Host RX/TX only |
| **Plant loop** | 500 Hz (TIM6) | `control_loop_tick()` | CAN apply/capture |

The host publishes **desire** commands at its own rate (hold-last-command). The plant runs at 500 Hz independently.

## Naming (command / feedback)

| Tier | Command (ingress) | Feedback (egress) |
|------|-------------------|-------------------|
| Host | `host_command_image_dispatch` | `host_feedback_image_fetch` |
| Plant | `plant_command_image_dispatch` | `plant_feedback_image_fetch` |
| Actuator | `actuator_command_mount` | `actuator_feedback_snapshot` |
| TIM6 | `actuator_apply_desire` | `actuator_capture_state` |

## Data flow

```mermaid
flowchart LR
  subgraph host["Host (Jetson)"]
    CMD["562 B command image"]
    FB["562 B feedback image"]
  end

  subgraph main["Main loop"]
    HL_RX["host_link_poll_rx"]
    HL_TX["host_link_poll_tx"]
    DISPATCH["host_command_image_dispatch"]
    FETCH["host_feedback_image_fetch"]
  end

  subgraph staging["Staging RAM"]
    DS["actuator_desire_stage[]"]
    SS["actuator_state_stage[]"]
  end

  subgraph tim6["TIM6 500 Hz"]
    APPLY["actuator_apply_desire"]
    CAP["actuator_capture_state"]
  end

  subgraph plant["Plant live RAM"]
    AD["actuator_desire_live[]"]
    AS["actuator_state_live[]"]
  end

  subgraph can["CAN"]
    MOTOR["RobStride drive"]
  end

  CMD --> HL_RX --> DISPATCH --> DS
  DS --> APPLY --> AD --> MOTOR
  MOTOR --> APPLY --> AS --> CAP --> SS
  SS --> FETCH --> HL_TX --> FB
```

## Buffer handoffs

| Buffer | Size / type | Writer | Reader | Notes |
|--------|-------------|--------|--------|-------|
| Wire command image | 562 B | Host | `host_link` | Magic + layout v1 |
| `actuator_desire_stage[]` | `ACTUATOR_COUNT` × command | Main | TIM6 `actuator_apply_desire` | `actuator_desire_pending` |
| `actuator_desire_live[]` | Plant RAM | TIM6 | Plugins / CAN pack | Hold-last between host updates |
| `actuator_state_live[]` | Plant RAM | Plugins / CAN parse | TIM6 `actuator_capture_state` | Per-motor feedback |
| `actuator_state_stage[]` | `ACTUATOR_COUNT` × feedback | TIM6 | `host_feedback_image_fetch` | Snapshot for host |
| Wire feedback image | 562 B | `host_link` | Host | Magic + tick + ack seq |
| Transport RX rings | 2048 B | ISR / CDC callback | Main `read()` | Drop-on-full |

**Wire vs plant:** Exchange structs define **25 actuator slots** on the wire. Firmware uses `ACTUATOR_COUNT` (currently **1**) ≤ `HOST_EXCHANGE_ACTUATOR_SLOTS`.

## Module map

```
App/
  Inc/app.h
  Src/app.c
  host/          wire schema, link, transport
  plant/         config, actuator, control loop, can/, plugin_schema/, plugins/
```

| Module | Role | Key files |
|--------|------|-----------|
| `app` | Init order, main loop | `App/Src/app.c` |
| `host_link` | RX reassembly, dispatch; TX fetch | `App/Src/host/host_link.c` |
| `host_transport` | USB vs UART vtable | `App/Src/host/host_transport*.c` |
| `host_exchange_schema` | Wire struct layout + static asserts | `App/Inc/host/host_exchange_schema.h` |
| `plant_command` | Dispatch wire command to subsystems | `App/Src/plant/plant_command.c` |
| `plant_feedback` | Aggregate feedback payload | `App/Src/plant/plant_feedback.c` |
| `actuator` | Mount, apply_desire, capture, snapshot | `App/Src/plant/actuator.c` |
| `control_loop` | TIM6 tick, heartbeat, motor enable on boot | `App/Src/plant/control_loop.c` |
| `can_router` | Per-bus TX queue, RX ring, FDCAN1 backend | `App/Src/plant/can/can_router.c` |
| `plugin_table` | Dispatch pack/parse by protocol | `App/Src/plant/plugin_schema/plugin_table.c` |
| `robstride` | RS-02 private extended-frame protocol | `App/Src/plant/plugins/robstride.c` |
| `plant_config` | Actuator table (RAM stub today) | `App/Src/plant/plant_config.c` |

## Host transport selection

Compile-time toggle in `App/Inc/host/host_transport.h`:

- `HOST_TRANSPORT_UART 1` → UART4, `host_transport_uart_ops`
- `HOST_TRANSPORT_UART 0` → USB CDC, `host_transport_usb_ops`

USB hooks live in **Cube USER CODE** only: `USB_Device/App/usbd_cdc_if.c` → `host_transport_usb_rx_push` / `host_transport_usb_tx_complete`.

UART path: `UART4_IRQHandler` → HAL → `HAL_UART_RxCpltCallback` in `App/Src/host/host_transport_uart.c`.

## Control loop (500 Hz)

On each TIM6 period:

1. `g_control_tick_count++` (12-bit field in feedback)
2. Heartbeat toggle PC3 every 250 ticks (~2 Hz)
3. `actuator_apply_desire()` — promote `actuator_desire_stage` → `actuator_desire_live`, pack TX, `can_router_poll`, parse RX
4. `actuator_capture_state()` — copy `actuator_state_live[]` → `actuator_state_stage[]`

Boot: `control_loop_init()` sends RobStride enable (type 3) once, then starts TIM6.

## Main loop

```c
void app_run(void) {
    for (;;) {
        host_link_poll_rx();
        host_link_poll_tx();
    }
}
```

No CAN or actuator logic here — transport only.

## Invariants

- Plant rate is **500 Hz** regardless of host command/feedback rate.
- Host command images use **layout v1**, **562 bytes**, little-endian.
- RobStride drives use the **private extended-frame protocol** (not MIT/CANopen unless changed on the motor via type 25).
- `plant_command_image_dispatch()` only mounts **actuator** desires today; system/servo/LED/PDU fields are defined on the wire but not consumed yet.

## Related docs

- [host-exchange-v1.md](host-exchange-v1.md) — byte layout
- [bringup.md](bringup.md) — flash and bench
- [known-issues.md](known-issues.md) — races, resync, upgrade targets
