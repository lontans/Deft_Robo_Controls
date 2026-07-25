"""Static gravity-torque feedforward for the YAM arm (i2rt-style).

Optional and lazily-imported: constructing `PcbArmDriver` / running
`yam_continuous_all.py` without a `GravityComp` instance has zero behavior
change (torque stays 0.0, exactly as before this module existed). Nothing
here has been bench-validated on real hardware — see
docs/i2rt-vs-ours-arm-compare.md P2 before enabling it on a live arm.

Mirrors i2rt_cpp's `KDLHelper::compute_inverse_dynamics` +
`MotorChainRobot::gravity_comp_factor_` (docs/deft_vbeta_ref/i2rt_cpp/src/utils/kdl_helper.cpp,
src/robots/motor_chain_robot.cpp): recompute the model's static gravity
torque (`mj_inverse` with qvel=qacc=0) every call from the live joint
angles, then scale by an adjustable per-joint factor refined from real
measured hold-torque, the same way i2rt's `calibrate_gravity_comp()` does.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional, Sequence

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[3]
_DEFAULT_XML = (
    _REPO_ROOT
    / "docs"
    / "deft_vbeta_ref"
    / "i2rt_cpp"
    / "robot_models"
    / "yam"
    / "yam.xml"
)

# i2rt_cpp starts `gravity_comp_factor_` at a uniform 1.45 (empirically tuned
# on their rig) and only refines it online. We have no bench data yet, so
# start at 0.0 (no feedforward torque sent) rather than guessing a nonzero
# scale for hardware this hasn't been tested on. Set `.scale` directly or
# call `calibrate()` after a real bench pass.
_UNCALIBRATED_SCALE = 0.0
_SCALE_CLAMP = (0.0, 3.0)  # matches i2rt's calibrate_gravity_comp() clamp


class GravityComp:
    """Live MuJoCo static-gravity feedforward torque for the 6 arm joints."""

    def __init__(self, xml_path: Optional[Path] = None) -> None:
        try:
            import mujoco
        except ImportError as exc:  # pragma: no cover - environment dependent
            raise RuntimeError(
                "GravityComp requires the 'mujoco' package "
                "(pip install mujoco) — not installed."
            ) from exc
        path = Path(xml_path) if xml_path is not None else _DEFAULT_XML
        if not path.is_file():
            raise FileNotFoundError(f"YAM MuJoCo model not found: {path}")
        self._mujoco = mujoco
        self._model = mujoco.MjModel.from_xml_path(str(path))
        # Matches i2rt_cpp's KDLHelper ctor exactly (kdl_helper.cpp): disable
        # geom collisions and joint limits so mj_inverse only ever returns
        # gravity torque, never a spurious contact/limit constraint force.
        # Confirmed load-bearing on this exact model: leaving collisions on
        # produced a ~-8571 Nm "gravity" torque on J2 at a real bench pose
        # (vs. ~-5.8 Nm with collisions/limits off, in line with the
        # ~-10 Nm measured on the bench) — do not remove this.
        self._model.geom_contype[:] = 0
        self._model.geom_conaffinity[:] = 0
        self._model.jnt_limited[:] = 0
        self._data = mujoco.MjData(self._model)
        if self._model.nq < 6 or self._model.nv < 6:
            raise ValueError(
                f"yam.xml has fewer than 6 DoF (nq={self._model.nq}, nv={self._model.nv})"
            )
        # Per-joint scale, arm joints only (J1..J6) — gripper/J7 is always 0.
        self.scale = np.full(6, _UNCALIBRATED_SCALE, dtype=np.float64)

    def _raw_gravity(self, q6: Sequence[float]) -> np.ndarray:
        """Unscaled model gravity torque (Nm) at position `q6` (6,)."""
        q = np.asarray(q6, dtype=np.float64).reshape(6)
        self._data.qpos[:6] = q
        self._data.qvel[:6] = 0.0
        self._data.qacc[:6] = 0.0
        self._mujoco.mj_inverse(self._model, self._data)
        return np.array(self._data.qfrc_inverse[:6], dtype=np.float64)

    def compute(self, q6: Sequence[float]) -> np.ndarray:
        """Scaled gravity feedforward torque (Nm) for the 6 arm joints."""
        return (self._raw_gravity(q6) * self.scale).astype(np.float32)

    def calibrate(
        self,
        measured_torque: Sequence[float],
        q6: Sequence[float],
        joints: Sequence[int] = (1, 2, 3),
    ) -> None:
        """Refine `.scale` from a measured hold torque at a known pose.

        `joints` defaults to the shoulder/elbow chain (index 1-3, i.e. J2-J4)
        — same restriction as i2rt's `calibrate_gravity_comp()`, since those
        are the joints that see the most gravity load and the base/wrist
        joints are noisier to calibrate this way. Skips a joint if the model
        predicts ~0 torque there (avoids a divide-by-near-zero blowup).
        """
        g = self._raw_gravity(q6)
        measured = np.asarray(measured_torque, dtype=np.float64).reshape(6)
        lo, hi = _SCALE_CLAMP
        for j in joints:
            if abs(g[j]) < 1e-6:
                continue
            ratio = float(measured[j] / g[j])
            if lo <= ratio <= hi:
                self.scale[j] = ratio
