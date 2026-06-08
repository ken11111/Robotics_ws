"""sim → 実機 ライブミラー (Phase X.3 デモ)。

sim 内で動かしている SO-101 (sin スイープ) の関節角を、リアルタイムで
実機にも流す。**実機が物理的に動きます。**

前提:
- 12V 電源 ON、`/dev/ttyACM0` に CH343 (`lerobot-calibrate` 済)
- アーム周囲のクリアランスを必ず確認
- 電源 SW を手元に (異常時すぐ落とせるように)
- できれば 1 回目は安全のため手でアームを軽く支える

実行:
    python tests/x3_sim_to_real_mirror.py
    python tests/x3_sim_to_real_mirror.py --amplitude-deg 8 --period-s 8
    python tests/x3_sim_to_real_mirror.py --max-step-deg 1   # さらにゆっくり

安全装置 (二重):
- `RealBackend.apply_targets` 内: `joint_limits.yaml` で関節範囲クリップ
- LeRobot 内: `max_relative_target` で 1 step あたりの delta クリップ (deg)

Ctrl+C で終了 (実機は disable_torque されて脱力する。
脱力で重力に負けて垂れる姿勢ではないよう、終了前にアームを
安全姿勢に手で寄せておくと安心)。
"""

from __future__ import annotations

import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np

from so_arm101 import make_robot


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--robot-id", default="so101_follower_v1")
    parser.add_argument("--rate-hz", type=float, default=30.0,
                        help="ループ周期 (Hz)。既定 30Hz")
    parser.add_argument("--amplitude-deg", type=float, default=10.0,
                        help="各関節 sin スイープの振幅 (deg)。既定 10°")
    parser.add_argument("--period-s", type=float, default=6.0,
                        help="sin の周期 (s)。既定 6s = 0.17Hz")
    parser.add_argument("--max-step-deg", type=float, default=2.0,
                        help="LeRobot 側の 1 step あたり最大 delta (deg)。"
                             "既定 2° × 30Hz = 60°/s 上限")
    parser.add_argument("--countdown-s", type=float, default=3.0,
                        help="起動前カウントダウン秒数")
    args = parser.parse_args()

    print("=" * 60)
    print("⚠️  sim → 実機 WRITE モード — 実機が物理的に動きます")
    print("=" * 60)
    print(f"  port              : {args.port}")
    print(f"  amplitude         : ±{args.amplitude_deg}°  (各関節 sin)")
    print(f"  period            : {args.period_s}s  ({1/args.period_s:.2f} Hz)")
    print(f"  loop rate         : {args.rate_hz} Hz")
    print(f"  max step (LeRobot): {args.max_step_deg}° = "
          f"{args.max_step_deg * args.rate_hz:.0f}°/s 角速度上限")
    print()
    print("  □ アーム周囲のクリアランス OK か?")
    print("  □ 電源 SW が手元にあるか?")
    print("  □ 1 回目は手でアームを軽く支える準備したか?")
    print()
    print(f"{args.countdown_s:.0f} 秒後に開始 (Ctrl+C で中止)...")
    for i in range(int(args.countdown_s), 0, -1):
        print(f"  {i}...", end="\r", flush=True)
        time.sleep(1)
    print("  開始!  ")

    sim = make_robot("sim")
    real = make_robot(
        "real",
        port=args.port,
        robot_id=args.robot_id,
        torque=True,
        max_relative_target=args.max_step_deg,
    )

    amp = np.deg2rad(args.amplitude_deg)
    omega = 2 * np.pi / args.period_s
    dt = 1.0 / args.rate_hz

    print(f"  実機初期角 (deg): {np.round(np.rad2deg(real.read_state()), 1)}")
    print("  Ctrl+C / viewer の ✕ で終了")

    try:
        with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
            t0 = time.perf_counter()
            while viewer.is_running():
                tloop = time.perf_counter()
                t = tloop - t0
                # 各関節を位相ずらしの sin で揺らす (rad)
                q_target = np.array([
                    amp * np.sin(omega * t + i * np.pi / 3)
                    for i in range(6)
                ])
                sim.apply_targets(q_target)
                sim.step()
                # sim の qpos 実値 (物理ステップ後) を real に流す
                real.apply_targets(sim.read_state())
                viewer.sync()
                slack = dt - (time.perf_counter() - tloop)
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
