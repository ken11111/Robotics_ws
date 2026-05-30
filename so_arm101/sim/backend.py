"""SimBackend — MuJoCo implementation of RobotBase.

Composes the PTZ-tracking scene via mujoco.MjSpec (see scene.py for why)
and exposes the same apply_targets / read_state / get_frame contract as
the future RealBackend.
"""

from __future__ import annotations

import mujoco
import numpy as np

from .. import RobotBase, load_joint_limits
from .scene import build_ptz_tracking_model


class SimBackend(RobotBase):
    """MuJoCo-backed SO-ARM101.

    The first 6 qpos entries are the arm joints; remaining entries belong
    to the freejoint target body. apply_targets / read_state operate on the
    arm slice only.
    """

    ARM_DOF = 6

    def __init__(
        self,
        camera: str = "external_cam",
        render_size: tuple[int, int] = (640, 480),
    ) -> None:
        self.model = build_ptz_tracking_model()
        self.data = mujoco.MjData(self.model)

        self.joint_names, self.qpos_lower, self.qpos_upper = load_joint_limits()
        assert len(self.joint_names) == self.ARM_DOF

        # Renderer is lazy — only built when get_frame() is called, so that
        # headless smoke tests don't require a GL context.
        self._renderer: mujoco.Renderer | None = None
        self._camera = camera
        self._render_h, self._render_w = render_size[1], render_size[0]

        # qpos0 of the compiled spec already encodes the home pose, so a
        # plain reset is enough. See scene.py for why we don't use a keyframe.
        mujoco.mj_resetData(self.model, self.data)

    def apply_targets(self, q: np.ndarray) -> None:
        q = np.asarray(q, dtype=np.float64).reshape(-1)
        if q.size != self.ARM_DOF:
            raise ValueError(f"expected {self.ARM_DOF} joint targets, got {q.size}")
        q_clipped = np.clip(q, self.qpos_lower, self.qpos_upper)
        self.data.ctrl[: self.ARM_DOF] = q_clipped

    def read_state(self) -> np.ndarray:
        return self.data.qpos[: self.ARM_DOF].copy()

    def get_frame(self) -> np.ndarray | None:
        if self._renderer is None:
            self._renderer = mujoco.Renderer(
                self.model, height=self._render_h, width=self._render_w
            )
        self._renderer.update_scene(self.data, camera=self._camera)
        rgb = self._renderer.render()
        # MuJoCo Renderer returns RGB; downstream CV code typically wants BGR.
        return rgb[:, :, ::-1].copy()

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
