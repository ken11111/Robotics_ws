"""Standalone MuJoCo viewer for the PTZ tracking scene.

Usage:
    python -m so_arm101.sim.run_sim
    python so_arm101/sim/run_sim.py

Holds the arm at the home (zeros) pose. The tracker control loop lives
in ptz_integration/ and will reuse the same SimBackend via make_robot().
"""

from __future__ import annotations

import time

import mujoco
import mujoco.viewer
import numpy as np

from so_arm101 import make_robot


def main() -> None:
    robot = make_robot("sim")
    home = np.zeros(robot.ARM_DOF)

    with mujoco.viewer.launch_passive(robot.model, robot.data) as viewer:
        while viewer.is_running():
            robot.apply_targets(home)
            robot.step()
            viewer.sync()
            time.sleep(robot.model.opt.timestep)


if __name__ == "__main__":
    main()
