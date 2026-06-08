"""RealBackend — LeRobot `SOFollower` を `RobotBase` に橋渡し。

単位系: 本 ws 内部は **rad** で統一 (Menagerie / `joint_limits.yaml` 準拠)。
SOFollower は use_degrees=True で **deg** を返す。境界 (この backend)
で deg ↔ rad を変換する。

順序: qpos 順 = [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
wrist_roll, gripper]。SOFollower の dict キーから明示的に並べ替える。

実装ステータス:
- ✅ read_state         (Phase X.2: 実機 → sim ミラー)
- ✅ apply_targets      (Phase X.3: sim → 実機 ミラー、要 torque=True)
- ⏳ get_frame          (カメラ未接続、None を返す)
"""

from __future__ import annotations

import numpy as np

from .. import RobotBase, load_joint_limits

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_ROBOT_ID = "so101_follower_v1"


class RealBackend(RobotBase):
    """SO-ARM101 follower 実機 backend (LeRobot SOFollower ラッパー)。

    Args:
        port: USB-RS485 ポート (`lerobot-find-port` で特定済の値)
        robot_id: キャリブ JSON の識別子 (`lerobot-calibrate --robot.id`)
        torque: True なら通常制御モード (位置保持・apply_targets で動かす)、
            False なら脱力 (read-only / 人が手で動かす)。既定 False。
        max_relative_target: 1 回の apply_targets で許す最大角度差 (deg)。
            None なら無制限。安全のためミラーデモでは 2〜5 度を推奨。
    """

    ARM_DOF = 6

    def __init__(
        self,
        port: str = DEFAULT_PORT,
        robot_id: str = DEFAULT_ROBOT_ID,
        torque: bool = False,
        max_relative_target: float | None = None,
    ) -> None:
        # 遅延 import: sim-only ユーザーが lerobot[feetech] の起動コストを
        # 払わないで済むよう、ここで初めて触る。
        from lerobot.robots.so_follower import SOFollower
        from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig

        self.joint_names, self.qpos_lower, self.qpos_upper = load_joint_limits()
        assert len(self.joint_names) == self.ARM_DOF

        cfg = SOFollowerRobotConfig(
            port=port,
            id=robot_id,
            use_degrees=True,
            max_relative_target=max_relative_target,
        )
        self._robot = SOFollower(cfg)
        self._robot.connect(calibrate=False)

        if not torque:
            self._robot.bus.disable_torque()

    def apply_targets(self, q: np.ndarray) -> None:
        """目標関節角を実機サーボに送る (rad → deg → dict 変換)。

        - `qpos_lower/upper` (joint_limits.yaml) で先にクリップ
        - LeRobot 側でも `max_relative_target` による delta クリップが入る
          (本クラス __init__ で設定した場合)
        """
        q = np.asarray(q, dtype=np.float64).reshape(-1)
        if q.size != self.ARM_DOF:
            raise ValueError(
                f"expected {self.ARM_DOF} joint targets, got {q.size}"
            )
        q_clipped = np.clip(q, self.qpos_lower, self.qpos_upper)
        q_deg = np.rad2deg(q_clipped)
        action = {
            f"{name}.pos": float(q_deg[i])
            for i, name in enumerate(self.joint_names)
        }
        self._robot.send_action(action)

    def read_state(self) -> np.ndarray:
        obs = self._robot.get_observation()
        deg = np.array(
            [obs[f"{n}.pos"] for n in self.joint_names], dtype=np.float64
        )
        return np.deg2rad(deg)

    def get_frame(self) -> np.ndarray | None:
        return None

    def close(self) -> None:
        try:
            self._robot.disconnect()
        except Exception:
            pass
