"""Sim backend smoke test (headless, CI-safe).

Verifies:
  1. SimBackend constructs and loads the Menagerie SO-101 model.
  2. 200 simulation steps run without NaN.
  3. joint_limits.yaml agrees with the MuJoCo model's joint ranges
     (single source of truth — drift here means real-hw soft limits
     would diverge from sim).

Renderer is NOT exercised here; GUI viewer is run_sim.py's job.
"""

from __future__ import annotations

import sys

import mujoco
import numpy as np

from so_arm101 import make_robot, load_joint_limits


def main() -> int:
    robot = make_robot("sim")

    # (1) Model loaded
    assert robot.model.nq >= robot.ARM_DOF, "model has fewer DOF than expected"
    assert robot.data.qpos.shape[0] == robot.model.nq

    # (2) 200 steps, no NaN
    home = np.zeros(robot.ARM_DOF)
    for _ in range(200):
        robot.apply_targets(home)
        robot.step()
    if not np.isfinite(robot.data.qpos).all():
        print("FAIL: NaN/Inf in qpos after 200 steps", file=sys.stderr)
        return 1

    # (3) joint_limits.yaml == MuJoCo joint ranges
    yaml_names, yaml_lo, yaml_hi = load_joint_limits()
    for i, name in enumerate(yaml_names):
        jid = mujoco.mj_name2id(robot.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            print(f"FAIL: joint {name!r} not present in MuJoCo model", file=sys.stderr)
            return 1
        lo, hi = robot.model.jnt_range[jid]
        if not (np.isclose(lo, yaml_lo[i], atol=1e-4) and np.isclose(hi, yaml_hi[i], atol=1e-4)):
            print(
                f"FAIL: {name} limits mismatch — yaml=[{yaml_lo[i]:.5f}, {yaml_hi[i]:.5f}] "
                f"mjcf=[{lo:.5f}, {hi:.5f}]",
                file=sys.stderr,
            )
            return 1

    robot.close()
    print(f"OK: sim backend loaded ({robot.model.nq} qpos), 200 steps no NaN, "
          f"{len(yaml_names)} joint limits consistent")
    return 0


if __name__ == "__main__":
    sys.exit(main())
