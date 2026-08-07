"""Static gravity-torque feedforward for the YAM arm (i2rt-style).

Optional: constructing ``TeleopEngine`` / continuous without a ``GravityComp``
leaves torque at 0.0. Requires ``pip install mujoco`` and ``yam.xml`` under
``docs/deft_vbeta_ref/…``.

Promoted from legacy ``pcb_lab/legacy/_gravity_comp.py`` so dashboard teleop
and continuous share one implementation.
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

# Continuous / mouse path uses 1.45 (i2rt starting factor). Default construct
# stays 0.0 until the caller sets scale — safer for uncalibrated hosts.
_UNCALIBRATED_SCALE = 0.0
_SCALE_CLAMP = (0.0, 3.0)
I2RT_GRAVITY_SCALE = 1.45


class GravityComp:
    """Live MuJoCo static-gravity feedforward torque for the 6 arm joints."""

    def __init__(self, xml_path: Optional[Path] = None) -> None:
        try:
            import mujoco
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError(
                "GravityComp requires the 'mujoco' package (pip install mujoco)"
            ) from exc
        path = Path(xml_path) if xml_path is not None else _DEFAULT_XML
        if not path.is_file():
            raise FileNotFoundError(f"YAM MuJoCo model not found: {path}")
        self._mujoco = mujoco
        self._model = mujoco.MjModel.from_xml_path(str(path))
        # Disable geom collisions / joint limits so mj_inverse returns gravity
        # only (load-bearing — collisions on produced absurd torques on J2).
        self._model.geom_contype[:] = 0
        self._model.geom_conaffinity[:] = 0
        self._model.jnt_limited[:] = 0
        self._data = mujoco.MjData(self._model)
        if self._model.nq < 6 or self._model.nv < 6:
            raise ValueError(
                f"yam.xml has fewer than 6 DoF (nq={self._model.nq}, nv={self._model.nv})"
            )
        self.scale = np.full(6, _UNCALIBRATED_SCALE, dtype=np.float64)

    def _raw_gravity(self, q6: Sequence[float]) -> np.ndarray:
        q = np.asarray(q6, dtype=np.float64).reshape(6)
        self._data.qpos[:6] = q
        self._data.qvel[:6] = 0.0
        self._data.qacc[:6] = 0.0
        self._mujoco.mj_inverse(self._model, self._data)
        return np.array(self._data.qfrc_inverse[:6], dtype=np.float64)

    def compute(self, q6: Sequence[float]) -> np.ndarray:
        return (self._raw_gravity(q6) * self.scale).astype(np.float32)

    def calibrate(
        self,
        measured_torque: Sequence[float],
        q6: Sequence[float],
        joints: Sequence[int] = (1, 2, 3),
    ) -> None:
        g = self._raw_gravity(q6)
        measured = np.asarray(measured_torque, dtype=np.float64).reshape(6)
        lo, hi = _SCALE_CLAMP
        for j in joints:
            if abs(g[j]) < 1e-6:
                continue
            ratio = float(measured[j] / g[j])
            if lo <= ratio <= hi:
                self.scale[j] = ratio


def try_gravity_comp(
    *,
    scale: float = I2RT_GRAVITY_SCALE,
    xml_path: Optional[Path] = None,
) -> Optional[GravityComp]:
    """Build ``GravityComp`` or return None if mujoco/xml unavailable."""
    try:
        gc = GravityComp(xml_path=xml_path)
        gc.scale[:] = float(scale)
        return gc
    except Exception:
        return None


__all__ = ["GravityComp", "I2RT_GRAVITY_SCALE", "try_gravity_comp"]
