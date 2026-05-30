"""PTZ-tracking scene builder.

We compose the scene in Python via mujoco.MjSpec instead of using an XML
<include> wrapper because MuJoCo's textual XML include does not propagate
the included file's compiler.meshdir, which makes the Menagerie meshes fail
to resolve. MjSpec.from_file loads each file with its correct directory
context, so it side-steps the issue cleanly.

The resulting scene = Menagerie's scene.xml (so101 + floor + lights + skybox)
plus an external fixed camera and a freejoint target box.
"""

from __future__ import annotations

from pathlib import Path

import mujoco

SIM_DIR = Path(__file__).resolve().parent
MENAGERIE_DIR = SIM_DIR / "menagerie" / "robotstudio_so101"
BASE_SCENE_XML = MENAGERIE_DIR / "scene.xml"


def build_ptz_tracking_spec() -> mujoco.MjSpec:
    """Return a compiled-ready MjSpec for the PTZ tracking scene."""
    spec = mujoco.MjSpec.from_file(str(BASE_SCENE_XML))
    wb = spec.worldbody

    # External fixed camera — stand-in for the Spresense PTZ camera.
    # pos ~0.55 m in front of the arm, 0.35 m up; xyaxes oriented so the
    # camera looks back toward the origin with world +Z roughly as image up.
    wb.add_camera(
        name="external_cam",
        pos=[0.55, 0.0, 0.35],
        xyaxes=[0.0, 1.0, 0.0, -0.537, 0.0, 0.844],
    )

    # Moving target — red box the tracker should follow.
    target = wb.add_body(name="target", pos=[0.30, 0.05, 0.05])
    target.add_freejoint(name="target_free")
    target.add_geom(
        name="target_box",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[0.025, 0.025, 0.025],
        rgba=[0.9, 0.1, 0.1, 1.0],
        mass=0.05,
    )

    # NOTE: we deliberately do NOT add a <keyframe> here. The compiled
    # model's qpos0 already encodes the desired home pose (arm zeros +
    # target at its spawn body pos), so mj_resetData alone reaches it.
    # In mujoco 3.9.0 the passive viewer's keyframe UI can call
    # mj_setKeyframe with index -1 at startup, which segfaults — avoiding
    # keyframes side-steps the bug while losing nothing here.

    return spec


def build_ptz_tracking_model() -> mujoco.MjModel:
    return build_ptz_tracking_spec().compile()
