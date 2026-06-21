# Known issues and upgrade backlog

As-built gaps identified during architecture review. Safe for bench learning; address before production or multi-actuator scale.

## High priority (upgrade package)

| Issue | Where | Impact |
|-------|-------|--------|
| **No MCU magic resync on RX** | `host_link_rx_resync` | ~~One misaligned byte → commands never apply~~ **Fixed:** hunts `CMDH` magic in partial buffer. |
| **Main ↔ TIM6 staging races** | `actuator_stage_desires`, `actuator_consume`, snapshot | ~~Torn float reads/writes~~ **Fixed:** `__disable_irq()` around staging copies (short critical sections). |
| **Stale feedback while TX pending** | `host_link_poll_tx` | ~~Feedback frozen until prior send completes~~ **Fixed:** rebuild when `tx_ready()` each poll. |

## Medium priority

| Issue | Where | Impact |
|-------|-------|--------|
| **UART TX blocks main loop** | `host_transport_uart.c` `HAL_UART_Transmit` | `host_link_poll_rx` not called during blocking TX; RX ring can overflow under fast host. |
| **Silent CAN TX drop** | `actuator_apply` ignores `can_tx_enqueue` status | Full TX queue → command frame skipped with no flag. |
| **`float_to_uint(…, 16)` UB** | `robstride.c` | ~~`1<<16` on signed int~~ **Fixed:** unsigned shift. |
| **Out-of-range `protocol` index** | `plugin_table.c` | ~~OOB handler lookup~~ **Fixed:** `protocol >= PROTO_COUNT` guard. |
| **Enable marked OK when enqueue fails** | `control_loop_init` | ~~`motor_enabled` set without checking enqueue~~ **Fixed.** |
| **Both transports always linked** | `host_transport_usb.c` + `_uart.c` in project | Extra ~4 KiB BSS for duplicate RX rings even when only one mode is active. |

## Low priority / not implemented yet

| Item | Notes |
|------|-------|
| System / servo / LED / PDU staging | Wire layout defined; `command_image_apply` only stages actuators |
| Feedback `header.seq` | Not incremented |
| ROTS / apply-history ring | Noted in `host_link_poll_tx` TODO |
| NVM config loader | `config_loader` is RAM stub |
| FDCAN2/3 backends | Router only drives CH1 |
| RobStride model limits table | RS-02 hardcoded; RS-03/04 need limit profiles only (same protocol) |

## Intentional bench shortcuts

- `ACTUATOR_COUNT = 1` with 25 wire slots
- Magic resync and framing recovery deferred to host on Jetson side
- Blocking UART TX (simple bring-up; IT TX later)

When fixing an item, move it to a closed section with commit reference or delete from this file.
