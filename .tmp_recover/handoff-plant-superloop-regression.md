# Handoff: plant superloop regression ΓÇö RESOLVED (Jul 2026)

**Status:** Closed on bench. CH2 FDCAN (`teleop --slot 1`) and CH4 MCP (`teleop --slot 3`) plant teleop verified with `lapΓëê0ΓÇô1 ms`, `fb_ageΓëê0` during motion. CH2 cali after teleop verified.

**Authoritative bringup:** [bringup.md](bringup.md) ┬º7ΓÇô┬º8.

---

## Summary for future agents

Regression landed in **`d9ce9e6`** / **`c700c78`** after **`5df1f04`**. Restoring burst/superloop knobs alone was insufficient. Fixes spanned firmware (actuator scope, DXL skip, MCP init/TX) and host teleop (`plant.py`).

Do **not** reintroduce:

- Blocking `robstride_probe_tx` / `mcp2518_send` on the 500 Hz plant path (use fire-and-forget `try_send` for MIT; blocking OK for probes/recovery only).
- `FB_STALE` gating on flat position (use `ACK_STALE`).
- Servicing Dynamixel UART without `g_servo_host_session`.
- Lazy-only MCP init without eager boot init.

## Session log

| Date | Result |
|------|--------|
| Jul 2026 | CH4 MCP + CH2 FDCAN chunky; `lap` 200ΓÇô370 ms, `pend` maxed |
| Jul 2026 | Partial burst revert ΓÇö insufficient |
| Jul 2026 | DXL 50 ms skip ΓåÆ CH2 `lapΓëê0` |
| Jul 2026 | MCP init + host gating + TX path ΓåÆ CH4 motion + LED |
| Jul 2026 | Fire-and-forget MCP try_send ΓåÆ `lapΓëê0` during MCP teleop |
| Jul 2026 | Host cmd slew + fb_age metric fixes |
| Jul 2026 | CH2 cali: FDCAN reset before 0x05 |
