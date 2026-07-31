"""ros — optional ROS 2 adapter: construct HostProxy as a node.

    ROS topics/services -> ros/ node adapter -> HostProxy.set_section -> Hub

``ControlsPcbHostNode`` (``node.py``) owns exactly one ``HostProxy`` (one COM
owner). Actuator commands are full MIT section payloads
(``Float64MultiArray`` length ``5 * n`` interleaved ``[p, v, kp, kd, τ] * n``)
demuxed via ``HostProxy.set_section`` — same contract as in-process deft_vbeta.
This package does not invent gains or a parallel motion engine.

Default connect ``mode="bandwidth"`` (timing-safe teleop). CFG / discover /
cal stay off this node: use ``mode="debug"`` via ``pcb_lab`` / ``hub.debug``
for that, same as everywhere else in this SDK.

Nothing in this package is imported by ``deft_controls_sdk``'s top-level
``__init__.py`` and this module itself imports nothing — ``rclpy`` is only
required once a submodule here (``node``, ``topics``, ``__main__``) is
actually imported. ``import deft_controls_sdk`` / ``HostProxy`` must keep
working with no ROS install present.

Run: ``python -m deft_controls_sdk.ros --help``
"""
from __future__ import annotations
