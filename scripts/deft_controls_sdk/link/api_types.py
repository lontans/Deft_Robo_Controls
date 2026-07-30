"""Typed host objects — desires out, feedback in (not raw wire helpers)."""
from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, List, Mapping, Optional, Union

from .exceptions import InvalidFrameError, InvalidSlotError
from .exchange import (
    ACTUATOR_COUNT,
    HOST_COMMAND_MAGIC,
    HOST_EXCHANGE_ACTUATOR_SLOTS,
    HOST_LAYOUT_VERSION,
    IMAGE_BYTES,
    parse_actuator_feedback,
    parse_feedback_header,
    patch_actuator_desire,
    patch_led_command,
    patch_servo_command,
    patch_system_mcu_state,
    patch_system_rx_sim,
    patch_system_rx_sim_mask,
)
from .exchange.pack import patch_system_plant_apply, patch_system_stm32_mode


class McuState(IntEnum):
    """system.mcu_state (host_system_command_t bits 1-3) — safety/lifecycle.

    Observe vs control is ``plant_apply`` (wire bit 11), not mcu_state.
    ``DIAG_ONLY`` remains as a deprecated int value (2); prefer
    ``Connection.set_plant_apply(False)``.
    """

    NORMAL = 0
    RECOVERY = 1
    DIAG_ONLY = 2  # deprecated — FW maps to plant_apply=0
    ESTOP = 3


@dataclass(frozen=True)
class ActuatorDesire:
    """Raw MIT desire for one actuator slot — position (rad), velocity (rad/s), kp, kd,
    torque (Nm). An all-zero desire is a valid idle/no-torque command.
    """

    position: float = 0.0
    velocity: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    torque: float = 0.0


IDLE = ActuatorDesire()


@dataclass(frozen=True)
class ServoDesire:
    """One DXL host command (6 B). servo_id==0 clears the slot / ends session when both clear."""

    servo_id: int = 0
    native_step_position: int = 0
    native_speed_unit: int = 0
    torque_enable: bool = True
    led_control: bool = False
    operating_mode: int = 3  # position control


# host_led_command_t.mode (5-bit). See docs/legacy/rfc/rfc-led-factory-patterns.md.
LED_MODE_OFF = 0
LED_MODE_TEST = 1  # single-pixel chase (snake)
LED_MODE_FLASH = 2  # ~2 Hz full-strip red blink (bringup)
LED_MODE_SOLID_GREEN = 3
LED_MODE_SOLID_YELLOW = 4
LED_MODE_SOLID_RED = 5
LED_MODE_BLINK_YELLOW_SLOW = 6  # caution, 1 Hz 50%
LED_MODE_BLINK_RED_FAST = 7  # estop/fault, 5 Hz 50%
# Idle: cornflower #6495ED (100,149,237); 500 on / 500 off (1 Hz 50%).
LED_MODE_IDLE_CORNFLOWER = 8

LED_MODE_NAMES = {
    LED_MODE_OFF: "off",
    LED_MODE_TEST: "test",
    LED_MODE_FLASH: "flash",
    LED_MODE_SOLID_GREEN: "solid_green",
    LED_MODE_SOLID_YELLOW: "solid_yellow",
    LED_MODE_SOLID_RED: "solid_red",
    LED_MODE_BLINK_YELLOW_SLOW: "blink_yellow_slow",
    LED_MODE_BLINK_RED_FAST: "blink_red_fast",
    LED_MODE_IDLE_CORNFLOWER: "idle_cornflower",
}


def led_mode_name(mode: int) -> str:
    return LED_MODE_NAMES.get(int(mode), f"unknown({mode})")


def led_mode_from_pdb_kill(*, kill_state: int, estop_sense: int) -> int:
    """Mirror ``led_mode_from_pdb()`` in App/Src/plant/led.c.

    USB plant FB already folds stale PDB UART into HARD_ESTOP + COMMS_LOSS,
    so ``NORMAL`` here means fresh+normal on the wire.
    """
    # Local ints avoid importing pdb.frame at module load (link ↔ pdb cycle).
    kill_hard = 3
    kill_soft_ready = 2
    kill_soft_req = 1
    kill_normal = 0
    if int(kill_state) == kill_hard or int(estop_sense) == 0:
        return LED_MODE_BLINK_RED_FAST
    if int(kill_state) == kill_soft_ready:
        return LED_MODE_SOLID_RED
    if int(kill_state) == kill_soft_req:
        return LED_MODE_BLINK_YELLOW_SLOW
    if int(kill_state) == kill_normal:
        return LED_MODE_IDLE_CORNFLOWER
    return LED_MODE_BLINK_RED_FAST


LED_DESIRE_MODES = ("debug", "pdu", "follow")


def infer_effective_led(
    *,
    host_led_mode: Optional[object] = None,
    host_led: Optional["LedDesire"] = None,
    listen_pdu: bool = False,
    kill_state: Optional[int] = None,
    estop_sense: Optional[int] = None,
) -> Dict[str, object]:
    """Host LedDesire policy vs PDU traffic-light (mirrors led.c).

    ``debug`` → forced numeric pattern. ``pdu`` / ``follow`` → wire mode 0
    (no host override); PDU map when listening, else unknown/NVM default.
    """
    desire = host_led
    if desire is None and host_led_mode is not None:
        if isinstance(host_led_mode, LedDesire):
            desire = host_led_mode
        elif isinstance(host_led_mode, str):
            desire = LedDesire(mode=host_led_mode)
        else:
            desire = LedDesire(mode=int(host_led_mode))

    policy = desire.mode if desire is not None else "follow"
    wire = desire.wire_mode if desire is not None else LED_MODE_OFF
    listening = True if policy == "pdu" else bool(listen_pdu)

    out: Dict[str, object] = {
        "host_mode": wire,
        "host_mode_name": led_mode_name(wire),
        "host_policy": policy,
        "listen_pdu": listening,
    }
    if policy == "debug" and wire != LED_MODE_OFF:
        out["source"] = "host_debug"
        out["effective_mode"] = wire
        out["effective_mode_name"] = led_mode_name(wire)
        out["note"] = "LedDesire debug — numeric pattern overrides PDU/NVM"
        return out
    if not listening:
        out["source"] = "nvm_or_follow"
        out["effective_mode"] = LED_MODE_IDLE_CORNFLOWER
        out["effective_mode_name"] = led_mode_name(LED_MODE_IDLE_CORNFLOWER)
        out["note"] = "follow with listen_pdu=0 — MCU uses NVM default_mode"
        return out
    if kill_state is None:
        out["source"] = "unknown"
        out["effective_mode"] = LED_MODE_OFF
        out["effective_mode_name"] = "off"
        out["note"] = "listen_pdu set but no PDU kill bytes in FB yet"
        return out
    sense = 1 if estop_sense is None else int(estop_sense)
    eff = led_mode_from_pdb_kill(kill_state=int(kill_state), estop_sense=sense)
    out["source"] = "pdu_traffic_light"
    out["effective_mode"] = eff
    out["effective_mode_name"] = led_mode_name(eff)
    out["note"] = (
        "PDU: NORMAL→cornflower, SOFT_REQ→blink yellow, "
        "SOFT_READY→solid red, HARD/estop_sense=0/stale→blink red"
    )
    return out


@dataclass(frozen=True)
class LedDesire:
    """Host LED policy packed into the 2 B SK9822 command word.

    ``mode`` is a policy string:
      - ``debug``  — force ``pattern`` (LED_MODE_* 1..8) on the wire
      - ``pdu``    — wire mode 0 + ensure listen_pdu (traffic-light)
      - ``follow`` — wire mode 0; MCU follows NVM listen_pdu bit

    Legacy ``LedDesire(mode=8)`` (int) still works → ``debug`` + that pattern.
    ``led_count`` 0 ⇒ firmware max (300).
    """

    mode: object = "follow"
    pattern: int = 0
    master_brightness: int = 8
    led_count: int = 0

    def __post_init__(self) -> None:
        raw = self.mode
        if isinstance(raw, int):
            mid = int(raw) & 0x1F
            if mid == 0:
                object.__setattr__(self, "mode", "follow")
                object.__setattr__(self, "pattern", 0)
            else:
                object.__setattr__(self, "mode", "debug")
                object.__setattr__(self, "pattern", mid)
        else:
            m = str(raw).strip().lower()
            if m not in LED_DESIRE_MODES:
                raise ValueError(
                    f"LedDesire.mode must be one of {LED_DESIRE_MODES}, got {raw!r}"
                )
            object.__setattr__(self, "mode", m)
            object.__setattr__(self, "pattern", int(self.pattern) & 0x1F)
        object.__setattr__(
            self, "master_brightness", max(0, min(31, int(self.master_brightness)))
        )
        object.__setattr__(self, "led_count", max(0, int(self.led_count)))

    @property
    def wire_mode(self) -> int:
        """Numeric mode byte for the plant command word."""
        if self.mode == "debug":
            return int(self.pattern) & 0x1F
        return LED_MODE_OFF


def validate_slot(slot: int) -> None:
    if not (0 <= slot < ACTUATOR_COUNT):
        raise InvalidSlotError(
            f"slot must be 0..{ACTUATOR_COUNT - 1} "
            f"(ACTUATOR_COUNT={ACTUATOR_COUNT}, wire={HOST_EXCHANGE_ACTUATOR_SLOTS}); "
            f"got {slot}."
        )


class CommandImage:
    """Mutable builder for one 694 B command frame."""

    def __init__(
        self,
        seq: int = 0,
        mcu_state: McuState = McuState.NORMAL,
        *,
        plant_apply: bool = True,
    ) -> None:
        self._buf = bytearray(IMAGE_BYTES)
        struct.pack_into(
            "<IHHI", self._buf, 0, HOST_COMMAND_MAGIC, HOST_LAYOUT_VERSION, IMAGE_BYTES, seq & 0xFFFFFFFF
        )
        patch_system_mcu_state(self._buf, int(mcu_state))
        patch_system_plant_apply(self._buf, bool(plant_apply))
        self._desires: Dict[int, ActuatorDesire] = {}
        self._servos: Dict[int, ServoDesire] = {}
        self._led: Optional[LedDesire] = None
        self._rx_sim_mask = 0
        self._plant_apply = bool(plant_apply)

    @property
    def seq(self) -> int:
        seq, = struct.unpack_from("<I", self._buf, 8)
        return seq

    def set_seq(self, seq: int) -> "CommandImage":
        struct.pack_into("<I", self._buf, 8, seq & 0xFFFFFFFF)
        return self

    def set_mcu_state(self, state: McuState) -> "CommandImage":
        patch_system_mcu_state(self._buf, int(state))
        return self

    def set_plant_apply(self, enable: bool) -> "CommandImage":
        """Wire bit11: True = apply desires; False = observe (no plant mount)."""
        self._plant_apply = bool(enable)
        patch_system_plant_apply(self._buf, self._plant_apply)
        return self

    def set_rx_sim(self, enable: bool) -> "CommandImage":
        """True → ACTUATOR rx_sim only (bit0). Prefer set_rx_sim_mask for children."""
        return self.set_rx_sim_mask(0x1 if enable else 0)

    def set_rx_sim_mask(self, mask: int) -> "CommandImage":
        """system.reserved bits0..3: ACTUATOR|SERVO|LED|PDU."""
        self._rx_sim_mask = int(mask) & 0xF
        patch_system_rx_sim_mask(self._buf, self._rx_sim_mask)
        return self

    def set_stm32_mode(self, mode: int) -> "CommandImage":
        """system wire bits9..10: plant/debug/soft_dfu (ADR-004)."""
        patch_system_stm32_mode(self._buf, int(mode) & 0x3)
        return self

    def set_actuator(self, slot: int, desire: ActuatorDesire) -> "CommandImage":
        validate_slot(slot)
        patch_actuator_desire(
            self._buf, desire.position, desire.velocity, desire.kp, desire.kd, desire.torque, slot=slot
        )
        self._desires[slot] = desire
        return self

    def set_actuators(self, desires: Mapping[int, ActuatorDesire]) -> "CommandImage":
        for slot, desire in desires.items():
            self.set_actuator(slot, desire)
        return self

    def set_servo(self, slot: int, desire: ServoDesire) -> "CommandImage":
        if slot not in (0, 1):
            raise InvalidSlotError(f"servo slot must be 0..1, got {slot}")
        patch_servo_command(
            self._buf,
            slot,
            servo_id=desire.servo_id,
            native_step_position=desire.native_step_position,
            native_speed_unit=desire.native_speed_unit,
            torque_enable=desire.torque_enable,
            led_control=desire.led_control,
            operating_mode=desire.operating_mode,
        )
        self._servos[slot] = desire
        return self

    def set_servos(self, desires: Mapping[int, ServoDesire]) -> "CommandImage":
        for slot, desire in desires.items():
            self.set_servo(slot, desire)
        return self

    def set_led(self, desire: LedDesire) -> "CommandImage":
        patch_led_command(
            self._buf,
            mode=desire.wire_mode,
            master_brightness=desire.master_brightness,
            led_count=desire.led_count,
        )
        self._led = desire
        return self

    def desire(self, slot: int) -> ActuatorDesire:
        validate_slot(slot)
        return self._desires.get(slot, IDLE)

    def to_bytes(self) -> bytes:
        return bytes(self._buf)

    def __bytes__(self) -> bytes:
        return self.to_bytes()

    def __len__(self) -> int:
        return len(self._buf)


class PlantBlockReason(IntEnum):
    """diag.h plant_block_reason_t — why actuator_apply_desire() is not driving CAN."""

    NONE = 0
    BENCH_SESSION = 1
    PROBE_BUSY = 2
    QUIET_PERIOD = 3
    APPLY_OFF = 4  # plant_apply=0 (legacy name DIAG_ONLY)
    HOST_STALE = 5
    SERVO_SESSION = 6

    # Back-compat alias
    DIAG_ONLY = 4


@dataclass(frozen=True)
class FeedbackState:
    """One actuator_feedback[] slot."""

    position: float
    velocity: float
    torque: float
    temperature: float
    fault: int


class FeedbackImage:
    """Parsed 694 B feedback frame — raises InvalidFrameError on bad magic/size."""

    __slots__ = ("_raw", "_header", "_slots")

    def __init__(self, raw: bytes) -> None:
        if len(raw) != IMAGE_BYTES:
            raise InvalidFrameError(f"expected {IMAGE_BYTES} B frame, got {len(raw)}")
        header = parse_feedback_header(raw)
        if header is None:
            raise InvalidFrameError("bad magic/layout_version/byte_size in feedback frame")
        self._raw = raw
        self._header = header
        slots: List[Optional[FeedbackState]] = []
        for slot in range(ACTUATOR_COUNT):
            act = parse_actuator_feedback(raw, slot)
            slots.append(
                FeedbackState(act["position"], act["velocity"], act["torque"], act["temperature"], act["fault"])
                if act is not None
                else None
            )
        self._slots = slots

    @property
    def raw(self) -> bytes:
        return self._raw

    @property
    def tick(self) -> int:
        """TIM6 control_tick_count, 12-bit, wraps at 4096."""
        return self._header["tick"]

    @property
    def ack_seq(self) -> int:
        """8-bit echo of the mounted command's header.seq (seq & 0xFF)."""
        return self._header["last_cmd_seq"]

    @property
    def mcu_state(self) -> int:
        return self._header["mcu_state"]

    @property
    def plant_block(self) -> Union[PlantBlockReason, int]:
        """PlantBlockReason, or the raw int if firmware reports an unmapped code."""
        raw = self._header["plant_block"]
        try:
            return PlantBlockReason(raw)
        except ValueError:
            return raw

    def actuator(self, slot: int) -> Optional[FeedbackState]:
        if not (0 <= slot < ACTUATOR_COUNT):
            raise ValueError(f"slot must be 0..{ACTUATOR_COUNT - 1}, got {slot}")
        return self._slots[slot]

    @property
    def actuators(self) -> List[Optional[FeedbackState]]:
        return list(self._slots)
