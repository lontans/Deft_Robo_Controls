"""HubSession — PlantSession + servos[2] + leds[1] compose, for controls_hub_dashboard.

Adds the two remaining plant-path wire regions (servos[] @ offset 516, leds[] @ offset
528) on top of PlantSession's actuator_commands[]/mcu_state surface. `pdu` (offset 530)
is never written — see docs/controls_hub_dashboard.md hard constraint #1.

Same held-state pattern as PlantSession._desires: set_servo()/set_led() update a
host-side mirror that build_command() resends in full on every frame, because firmware
mounts the whole command image unconditionally (design doc §3 row 1).
"""
from __future__ import annotations

import struct
import threading
from dataclasses import dataclass
from typing import Dict, Mapping, Optional

from controls_pcb_host.commands import pack_led_word, patch_servo_command
from controls_pcb_host.protocol import HOST_PDU_PAYLOAD_BYTES, LED_CMD_OFF, PDU_OFF

from .command import ActuatorDesire, McuState
from .session import PlantSession

SERVO_SLOTS = (0, 1)


@dataclass(frozen=True)
class ServoCommand:
    """One servos[] wire slot — host_servo_command_t."""

    position: int  # native_step_position
    servo_id: int
    speed: int = 0  # native_speed_unit
    torque_enable: int = 1
    operating_mode: int = 3


@dataclass(frozen=True)
class LedCommand:
    """The single leds[] wire slot — host_led_command_t, packed at LED_CMD_OFF."""

    mode: int = 1  # 0 = test scan, 1 = off (firmware-defined)
    brightness: int = 0  # 0..31 (SK9822 global brightness)
    led_count: int = 0  # 0 = firmware default (LED_STRIP_MAX)


class HubSession(PlantSession):
    def __init__(self, port: str, **kwargs: object) -> None:
        super().__init__(port, **kwargs)  # type: ignore[arg-type]
        self._hub_lock = threading.RLock()
        self._io_lock = threading.Lock()
        self._servo_cmds: Dict[int, ServoCommand] = {}
        self._led_cmd: Optional[LedCommand] = None

    # -- servo / LED held state -----------------------------------------------------

    def set_servo(
        self,
        slot: int,
        position: int,
        *,
        servo_id: int,
        speed: int = 0,
        torque_enable: int = 1,
        operating_mode: int = 3,
        send: bool = True,
    ) -> None:
        if slot not in SERVO_SLOTS:
            raise ValueError(f"servo slot must be one of {SERVO_SLOTS}, got {slot}")
        with self._hub_lock:
            self._servo_cmds[slot] = ServoCommand(
                position=position,
                servo_id=servo_id,
                speed=speed,
                torque_enable=torque_enable,
                operating_mode=operating_mode,
            )
        if send:
            self.send_once()

    def set_led(self, mode: int, brightness: int, led_count: int, *, send: bool = True) -> None:
        with self._hub_lock:
            self._led_cmd = LedCommand(mode=mode, brightness=brightness, led_count=led_count)
        if send:
            self.send_once()

    def held_servo(self, slot: int) -> Optional[ServoCommand]:
        return self._servo_cmds.get(slot)

    @property
    def held_led(self) -> Optional[LedCommand]:
        return self._led_cmd

    # -- overrides: route actuator/mcu_state mutation + send through the hub lock ----

    def set_actuator(self, slot: int, desire: ActuatorDesire, *, send: bool = True) -> None:
        with self._hub_lock:
            super().set_actuator(slot, desire, send=False)
        if send:
            self.send_once()

    def set_actuators(self, desires: Mapping[int, ActuatorDesire], *, send: bool = True) -> None:
        with self._hub_lock:
            super().set_actuators(desires, send=False)
        if send:
            self.send_once()

    def set_mcu_state(self, state: McuState, *, send: bool = True) -> None:
        with self._hub_lock:
            super().set_mcu_state(state, send=False)
        if send and self._ser is not None:
            self.send_once()

    # -- compose ----------------------------------------------------------------------

    def build_command(self) -> bytes:  # type: ignore[override]
        """actuator_commands[] + mcu_state (from PlantSession) + servos[] + leds[].
        pdu is left untouched (all-zero) — asserted below."""
        with self._hub_lock:
            base = PlantSession.build_command(self).to_bytes()
            buf = bytearray(base)
            for slot, servo in self._servo_cmds.items():
                patch_servo_command(
                    buf,
                    slot,
                    servo.position,
                    servo.servo_id,
                    torque_enable=servo.torque_enable,
                    speed=servo.speed,
                    operating_mode=servo.operating_mode,
                )
            if self._led_cmd is not None:
                word = pack_led_word(self._led_cmd.mode, self._led_cmd.brightness, self._led_cmd.led_count)
                struct.pack_into("<H", buf, LED_CMD_OFF, word)
        assert buf[PDU_OFF : PDU_OFF + HOST_PDU_PAYLOAD_BYTES] == b"\x00" * HOST_PDU_PAYLOAD_BYTES, (
            "HubSession must never write pdu — see docs/controls_hub_dashboard.md constraint #1"
        )
        return bytes(buf)

    def send_once(self) -> None:
        """Send one composed frame (actuators + servos + leds). Overrides PlantSession
        because build_command() here returns bytes, not CommandImage."""
        self.write_command(self.build_command())

    def write_command(self, frame: bytes) -> None:  # type: ignore[override]
        ser = self._require_serial()
        with self._io_lock:
            ser.write(frame)
            ser.flush()
