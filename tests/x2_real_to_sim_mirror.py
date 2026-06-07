"""実機→sim ライブミラー (Phase X.2 デモ)。

実機の現在角を読み続け、sim にそのまま流す。MuJoCo viewer で同じ姿勢が
リアルタイム表示される。**実機を手で動かす → sim が追従** が見える。

前提:
- 12V 電源 ON、`/dev/ttyACM0` に CH343 (`lerobot-calibrate` 済)
- WSLg で MuJoCo viewer が出ること (出ない場合は `MUJOCO_GL=egl` でも可、
  ただし画面表示にはならない)

実行:
    python tests/x2_real_to_sim_mirror.py
    python tests/x2_real_to_sim_mirror.py --port /dev/ttyACM1
    python tests/x2_real_to_sim_mirror.py --rate-hz 30

torque は RealBackend のデフォルトで OFF。アームを手で動かしてください。
Ctrl+C で安全に終了。
"""

from __future__ import annotations

import argparse
import time

import mujoco
import mujoco.viewer

from so_arm101 import make_robot


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--robot-id", default="so101_follower_v1")
    parser.add_argument("--rate-hz", type=float, default=50.0,
                        help="ループ周期 (Hz). 既定 50Hz = 20ms 周期")
    args = parser.parse_args()

    sim = make_robot("sim")
    real = make_robot("real", port=args.port, robot_id=args.robot_id, torque=False)
    dt = 1.0 / args.rate_hz

    print(
        f"[mirror] 起動。実機 ({args.port}) を手で動かすと sim が追従。\n"
        f"         loop rate = {args.rate_hz:g} Hz, Ctrl+C で終了"
    )
    try:
        with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
            while viewer.is_running():
                t0 = time.perf_counter()
                q_rad = real.read_state()
                sim.apply_targets(q_rad)
                sim.step()
                viewer.sync()
                # rate limit
                slack = dt - (time.perf_counter() - t0)
                if slack > 0:
                    time.sleep(slack)
    except KeyboardInterrupt:
        print("\n[mirror] Ctrl+C 検知、終了処理中...")
    finally:
        real.close()
        sim.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
