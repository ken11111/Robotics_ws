"""SO-ARM101 robot package.

Backend 抽象 (sim ↔ real swap) のエントリーポイント。
上位の ptz_integration/* はこの make_robot() のみを呼び、
MuJoCo / pyserial への直接依存を持たない。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path

import numpy as np
import yaml

PACKAGE_DIR = Path(__file__).resolve().parent
CONFIG_DIR = PACKAGE_DIR / "config"
JOINT_LIMITS_PATH = CONFIG_DIR / "joint_limits.yaml"


def load_joint_limits() -> tuple[list[str], np.ndarray, np.ndarray]:
    """Return (joint_names, lower_bounds, upper_bounds) in qpos order."""
    with open(JOINT_LIMITS_PATH) as f:
        spec = yaml.safe_load(f)
    names = [j["name"] for j in spec["joints"]]
    lo = np.array([j["min"] for j in spec["joints"]], dtype=np.float64)
    hi = np.array([j["max"] for j in spec["joints"]], dtype=np.float64)
    return names, lo, hi


class RobotBase(ABC):
    """SO-ARM101 backend interface.

    Implementations: SimBackend (MuJoCo), RealBackend (USB-RS485 + Feetech).
    Upper layers (PTZ tracking, E-Stop, soft limit) program against this only.
    """

    joint_names: list[str]
    qpos_lower: np.ndarray
    qpos_upper: np.ndarray

    @abstractmethod
    def apply_targets(self, q: np.ndarray) -> None:
        """Send target joint angles (rad, qpos order). Length == 6."""

    @abstractmethod
    def read_state(self) -> np.ndarray:
        """Return current joint angles (rad, qpos order). Length == 6."""

    @abstractmethod
    def get_frame(self) -> np.ndarray | None:
        """Return BGR camera frame, or None if camera comes from outside."""

    def step(self) -> None:
        """Advance one simulation/control tick. Real backend may no-op."""

    def close(self) -> None:
        """Release resources. Real backend disconnects serial."""


def make_robot(backend: str = "sim", **kwargs) -> RobotBase:
    """Factory. Use this from upper layers, not the Backend classes directly."""
    if backend == "sim":
        from .sim.backend import SimBackend
        return SimBackend(**kwargs)
    if backend == "real":
        from .real.backend import RealBackend  # implemented after hardware arrives
        return RealBackend(**kwargs)
    raise ValueError(f"unknown backend: {backend!r}")
