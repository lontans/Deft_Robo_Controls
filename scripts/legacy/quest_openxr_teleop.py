#!/usr/bin/env python3
"""In-person Quest controller reader via OpenXR (no Deft app, no WebRTC rooms).

Headset = tracker / bridge. Controllers = sticks + buttons + 6DOF grips.
Emits the same TeleopSample schema as quest_udp_sniff.py.

Prereqs (Windows):
  1. Install Meta Quest Link (PC app) — sets the OpenXR runtime.
  2. Headset on same account, Enable Link / Air Link to THIS PC.
  3. In Quest Link app: Settings → General → OpenXR Runtime → Set Meta Quest Link.
  4. Wear headset so the session reaches FOCUSED (system UI dismissed).
  5. pip install pyopenxr  (and a working GLFW/OpenGL stack)

Usage:
  python scripts/quest_openxr_teleop.py
  python scripts/quest_openxr_teleop.py --ndjson
"""

from __future__ import annotations

import argparse
import ctypes
import json
import math
import os
import sys
import time
from dataclasses import asdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import xr
    from xr.utils.gl import ContextObject
    from xr.utils.gl.glfw_util import GLFWOffscreenContextProvider
except ImportError as e:  # pragma: no cover
    print("Need pyopenxr:  python -m pip install pyopenxr", file=sys.stderr)
    raise SystemExit(2) from e

from quest_udp_sniff import TeleopSample, print_console

TOUCH_PROFILE = "/interaction_profiles/oculus/touch_controller"


def _quat_to_list(q) -> list[float]:
    return [float(q.x), float(q.y), float(q.z), float(q.w)]


def _pos_to_list(p) -> list[float]:
    return [float(p.x), float(p.y), float(p.z)]


def _euler_xyz_from_quat(x: float, y: float, z: float, w: float) -> list[float]:
    """Match VrState head_rot as Euler XYZ (radians), rough OpenXR→teleop note."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [roll, pitch, yaw]


def _float_action(session, action, subpath) -> float:
    st = xr.get_action_state_float(
        session,
        xr.ActionStateGetInfo(action=action, subaction_path=subpath),
    )
    return float(st.current_state) if st.is_active else 0.0


def _bool_action(session, action, subpath) -> bool:
    st = xr.get_action_state_boolean(
        session,
        xr.ActionStateGetInfo(action=action, subaction_path=subpath),
    )
    return bool(st.current_state) if st.is_active else False


def _vec2_action(session, action, subpath) -> tuple[float, float]:
    st = xr.get_action_state_vector2f(
        session,
        xr.ActionStateGetInfo(action=action, subaction_path=subpath),
    )
    if not st.is_active:
        return 0.0, 0.0
    return float(st.current_state.x), float(st.current_state.y)


def _pose_or_default(session, space, base, t) -> tuple[list[float], list[float], bool]:
    loc = xr.locate_space(space=space, base_space=base, time=t)
    ok = bool(loc.location_flags & xr.SPACE_LOCATION_POSITION_VALID_BIT)
    if not ok:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], False
    return _pos_to_list(loc.pose.position), _quat_to_list(loc.pose.orientation), True


def run(*, ndjson: bool, hz: float) -> int:
    # Fail early with a clear message if Meta Quest Link / SteamVR isn't the runtime.
    try:
        xr.enumerate_instance_extension_properties()
    except xr.exception.RuntimeUnavailableError:
        print(
            "No OpenXR runtime registered.\n"
            "Install Meta Quest Link → Settings → General → OpenXR Runtime → Set as active,\n"
            "then Link/Air Link the headset to this PC and re-run.",
            file=sys.stderr,
        )
        return 3

    with ContextObject(
        context_provider=GLFWOffscreenContextProvider(),
        instance_create_info=xr.InstanceCreateInfo(
            application_info=xr.ApplicationInfo(
                application_name="deft_inperson_teleop",
                application_version=1,
                engine_name="controls_pcb",
                engine_version=1,
                api_version=xr.XR_CURRENT_API_VERSION,
            ),
            enabled_extension_names=[xr.KHR_OPENGL_ENABLE_EXTENSION_NAME],
        ),
    ) as context:
        inst = context.instance
        left = xr.string_to_path(inst, "/user/hand/left")
        right = xr.string_to_path(inst, "/user/hand/right")
        hand_paths = (xr.Path * 2)(left, right)
        aset = context.default_action_set

        pose_action = xr.create_action(
            action_set=aset,
            create_info=xr.ActionCreateInfo(
                action_type=xr.ActionType.POSE_INPUT,
                action_name="grip_pose",
                localized_action_name="Grip Pose",
                count_subaction_paths=2,
                subaction_paths=hand_paths,
            ),
        )
        stick_action = xr.create_action(
            action_set=aset,
            create_info=xr.ActionCreateInfo(
                action_type=xr.ActionType.VECTOR2F_INPUT,
                action_name="thumbstick",
                localized_action_name="Thumbstick",
                count_subaction_paths=2,
                subaction_paths=hand_paths,
            ),
        )
        trigger_action = xr.create_action(
            action_set=aset,
            create_info=xr.ActionCreateInfo(
                action_type=xr.ActionType.FLOAT_INPUT,
                action_name="trigger",
                localized_action_name="Trigger",
                count_subaction_paths=2,
                subaction_paths=hand_paths,
            ),
        )
        squeeze_action = xr.create_action(
            action_set=aset,
            create_info=xr.ActionCreateInfo(
                action_type=xr.ActionType.FLOAT_INPUT,
                action_name="squeeze",
                localized_action_name="Squeeze",
                count_subaction_paths=2,
                subaction_paths=hand_paths,
            ),
        )
        a_action = xr.create_action(
            action_set=aset,
            create_info=xr.ActionCreateInfo(
                action_type=xr.ActionType.BOOLEAN_INPUT,
                action_name="a_button",
                localized_action_name="A",
                count_subaction_paths=1,
                subaction_paths=(xr.Path * 1)(right),
            ),
        )
        b_action = xr.create_action(
            action_set=aset,
            create_info=xr.ActionCreateInfo(
                action_type=xr.ActionType.BOOLEAN_INPUT,
                action_name="b_button",
                localized_action_name="B",
                count_subaction_paths=1,
                subaction_paths=(xr.Path * 1)(right),
            ),
        )
        x_action = xr.create_action(
            action_set=aset,
            create_info=xr.ActionCreateInfo(
                action_type=xr.ActionType.BOOLEAN_INPUT,
                action_name="x_button",
                localized_action_name="X",
                count_subaction_paths=1,
                subaction_paths=(xr.Path * 1)(left),
            ),
        )
        y_action = xr.create_action(
            action_set=aset,
            create_info=xr.ActionCreateInfo(
                action_type=xr.ActionType.BOOLEAN_INPUT,
                action_name="y_button",
                localized_action_name="Y",
                count_subaction_paths=1,
                subaction_paths=(xr.Path * 1)(left),
            ),
        )

        bindings = [
            (pose_action, "/user/hand/left/input/grip/pose"),
            (pose_action, "/user/hand/right/input/grip/pose"),
            (stick_action, "/user/hand/left/input/thumbstick"),
            (stick_action, "/user/hand/right/input/thumbstick"),
            (trigger_action, "/user/hand/left/input/trigger/value"),
            (trigger_action, "/user/hand/right/input/trigger/value"),
            (squeeze_action, "/user/hand/left/input/squeeze/value"),
            (squeeze_action, "/user/hand/right/input/squeeze/value"),
            (a_action, "/user/hand/right/input/a/click"),
            (b_action, "/user/hand/right/input/b/click"),
            (x_action, "/user/hand/left/input/x/click"),
            (y_action, "/user/hand/left/input/y/click"),
        ]
        suggested = (xr.ActionSuggestedBinding * len(bindings))(
            *[
                xr.ActionSuggestedBinding(
                    action=act,
                    binding=xr.string_to_path(inst, path),
                )
                for act, path in bindings
            ]
        )
        xr.suggest_interaction_profile_bindings(
            instance=inst,
            suggested_bindings=xr.InteractionProfileSuggestedBinding(
                interaction_profile=xr.string_to_path(inst, TOUCH_PROFILE),
                count_suggested_bindings=len(bindings),
                suggested_bindings=suggested,
            ),
        )

        left_space = xr.create_action_space(
            session=context.session,
            create_info=xr.ActionSpaceCreateInfo(action=pose_action, subaction_path=left),
        )
        right_space = xr.create_action_space(
            session=context.session,
            create_info=xr.ActionSpaceCreateInfo(action=pose_action, subaction_path=right),
        )

        print(
            "OpenXR session up. Wear headset (FOCUSED), move controllers. "
            "Ctrl+C to stop. No Deft app / no shared teleop rooms.",
            flush=True,
        )
        n = 0
        t0 = time.time()
        focused_once = False
        period = 1.0 / max(hz, 1.0)
        try:
            for frame_state in context.frame_loop():
                if context.session_state != xr.SessionState.FOCUSED:
                    continue
                focused_once = True
                active = xr.ActiveActionSet(
                    action_set=aset,
                    subaction_path=xr.NULL_PATH,
                )
                xr.sync_actions(
                    session=context.session,
                    sync_info=xr.ActionsSyncInfo(
                        count_active_action_sets=1,
                        active_action_sets=ctypes.pointer(active),
                    ),
                )
                t = frame_state.predicted_display_time
                lc_pos, lc_quat, lc_ok = _pose_or_default(
                    context.session, left_space, context.space, t
                )
                rc_pos, rc_quat, rc_ok = _pose_or_default(
                    context.session, right_space, context.space, t
                )
                # Head from view/reference space already used as context.space base;
                # locate IDENTITY offset = headset origin in that space is not exposed
                # the same way — leave head at zeros unless VIEW space is available.
                head_pos = [0.0, 0.0, 0.0]
                head_euler = [0.0, 0.0, 0.0]
                try:
                    view_space = getattr(context, "view_space", None) or getattr(
                        context, "space", None
                    )
                    if view_space is not None and hasattr(context, "view_space"):
                        hp, hq, hok = _pose_or_default(
                            context.session, context.view_space, context.space, t
                        )
                        if hok:
                            head_pos = hp
                            head_euler = _euler_xyz_from_quat(*hq)
                except Exception:
                    pass

                ltx, lty = _vec2_action(context.session, stick_action, left)
                rtx, rty = _vec2_action(context.session, stick_action, right)
                sample = TeleopSample(
                    t_unix=time.time(),
                    src="openxr",
                    head_pos=head_pos,
                    head_rot_euler=head_euler,
                    lc_pos=lc_pos,
                    lc_rot_quat=lc_quat,
                    rc_pos=rc_pos,
                    rc_rot_quat=rc_quat,
                    l_stick=[ltx, lty],
                    r_stick=[rtx, rty],
                    a=_bool_action(context.session, a_action, right),
                    b=_bool_action(context.session, b_action, right),
                    x=_bool_action(context.session, x_action, left),
                    y=_bool_action(context.session, y_action, left),
                    left_index=_float_action(context.session, trigger_action, left),
                    right_index=_float_action(context.session, trigger_action, right),
                    left_middle=_float_action(context.session, squeeze_action, left),
                    right_middle=_float_action(context.session, squeeze_action, right),
                    packet_bytes=0,
                )
                n += 1
                elapsed = max(time.time() - t0, 1e-6)
                if ndjson:
                    print(json.dumps(asdict(sample), separators=(",", ":")), flush=True)
                else:
                    tag = ("L" if lc_ok else "l") + ("R" if rc_ok else "r")
                    print_console(sample, n, n / elapsed)
                    if n % 30 == 1:
                        print(f"  (tracking={tag} session=FOCUSED)", flush=True)
                time.sleep(period)
        except KeyboardInterrupt:
            print(f"\nStopped. samples={n} focused_once={focused_once}", flush=True)
            return 0 if n > 0 else 2

        if not focused_once:
            print(
                "Session never reached FOCUSED — wear the headset and dismiss the dash.",
                file=sys.stderr,
            )
            return 4
        return 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Quest OpenXR → TeleopSample (in-person)")
    p.add_argument("--ndjson", action="store_true")
    p.add_argument("--hz", type=float, default=30.0, help="print rate cap")
    args = p.parse_args(argv)
    return run(ndjson=args.ndjson, hz=args.hz)


if __name__ == "__main__":
    raise SystemExit(main())
