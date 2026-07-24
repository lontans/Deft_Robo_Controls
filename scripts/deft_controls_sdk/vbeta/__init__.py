"""deft_vbeta-shaped drivers on ControlsPcbHub.

See docs/vbeta-pcb-adapter.md.
"""
from __future__ import annotations

from deft_controls_sdk.vbeta.arm import (
    PcbArmDriver,
    RobotDeviceAlreadyConnectedError,
    RobotDeviceNotConnectedError,
)
from deft_controls_sdk.vbeta.cfg import ensure_yam_product_cfg, table_matches_yam
from deft_controls_sdk.vbeta.leds import (
    LED_MODE_BLINK_RED_FAST,
    LED_MODE_BLINK_YELLOW_SLOW,
    LED_MODE_FLASH,
    LED_MODE_IDLE_CORNFLOWER,
    LED_MODE_OFF,
    LED_MODE_SOLID_GREEN,
    LED_MODE_SOLID_RED,
    LED_MODE_SOLID_YELLOW,
    LED_MODE_TEST,
    led_caution,
    led_fault,
    led_flash,
    led_idle,
    led_off,
    led_solid_green,
    led_solid_red,
    led_solid_yellow,
    led_test,
    set_led,
)
from deft_controls_sdk.vbeta.neck import PcbNeckDriver, deg_to_steps, steps_to_deg
from deft_controls_sdk.vbeta.platform import PcbPlatformClient
from deft_controls_sdk.vbeta.session import PcbRobotSession
from deft_controls_sdk.vbeta.slots import (
    BASE_DRIVE_SLOTS,
    BASE_SLOTS,
    BASE_STEER_SLOTS,
    LEFT_ARM_SLOTS,
    LIFT_SLOT,
    RIGHT_ARM_SLOTS,
    arm_slots,
    yam_product_rows,
)

__all__ = [
    "BASE_DRIVE_SLOTS",
    "BASE_SLOTS",
    "BASE_STEER_SLOTS",
    "LEFT_ARM_SLOTS",
    "LIFT_SLOT",
    "LED_MODE_BLINK_RED_FAST",
    "LED_MODE_BLINK_YELLOW_SLOW",
    "LED_MODE_FLASH",
    "LED_MODE_IDLE_CORNFLOWER",
    "LED_MODE_OFF",
    "LED_MODE_SOLID_GREEN",
    "LED_MODE_SOLID_RED",
    "LED_MODE_SOLID_YELLOW",
    "LED_MODE_TEST",
    "PcbArmDriver",
    "PcbNeckDriver",
    "PcbPlatformClient",
    "PcbRobotSession",
    "RIGHT_ARM_SLOTS",
    "RobotDeviceAlreadyConnectedError",
    "RobotDeviceNotConnectedError",
    "arm_slots",
    "deg_to_steps",
    "ensure_yam_product_cfg",
    "led_caution",
    "led_fault",
    "led_flash",
    "led_idle",
    "led_off",
    "led_solid_green",
    "led_solid_red",
    "led_solid_yellow",
    "led_test",
    "set_led",
    "steps_to_deg",
    "table_matches_yam",
    "yam_product_rows",
]
