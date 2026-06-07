"""実機 SO-ARM101 (follower) との通信スモークテスト。

前提:
- 12V 電源 ON、USB-RS485 (CH343) アダプタが /dev/ttyACM0 にあり
- `lerobot-calibrate` 済 (ID=so101_follower_v1)
- 6 サーボ全てがバスにチェーン接続済 (ID 1〜6)

何をするか:
- SOFollower で接続 (calibrate=False で既存キャリブを再利用)
- 1 回 get_observation を呼んで全関節角度を読み出す
- 切断
- **動かさない** (send_action は呼ばない、torque も触らない)

期待出力:
    --- 現在角 (度) ---
      gripper.pos        +12.34
      shoulder_pan.pos    -1.23
      ...
    OK
"""

from __future__ import annotations

import argparse
import sys


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--robot-id", default="so101_follower_v1")
    args = parser.parse_args()

    # SOFollowerRobotConfig は CLI 用 (RobotConfig 継承で id / calibration_dir 持ち)。
    # SOFollowerConfig (同じ module) は id を持たない別クラスなので使わない。
    from lerobot.robots.so_follower import SOFollower
    from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig

    cfg = SOFollowerRobotConfig(port=args.port, id=args.robot_id, use_degrees=True)
    robot = SOFollower(cfg)
    robot.connect(calibrate=False)
    try:
        obs = robot.get_observation()
        print("--- 現在角 (度) ---")
        for k, v in sorted(obs.items()):
            if k.endswith(".pos"):
                print(f"  {k:30s} {v:+7.2f}")
    finally:
        robot.disconnect()
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
