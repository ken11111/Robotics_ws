"""保持トルク (Pres_Load) を 6 個比較する診断ツール。

「ID3 が他より弱いのか」を Pres_Load の数値で確認する。

手順:
  1. torque OFF (脱力) のままアームを手で支え、
     elbow_flex (ID3) に重力負荷がかかる姿勢にする
     (例: 上腕水平, 前腕も水平 — 前腕が elbow から外側に伸びる姿勢)
  2. スクリプトが Goal_Position に現在位置を書く (動かない保証)
  3. torque ON
  4. ゆっくり手を離す (持てない場合に備えて支えは継続できる体勢)
  5. 10 サンプル取って平均 Pres_Load を表示

読み方:
  - Pres_Load は 0..1023 の符号付き (% of max torque 相当)
  - 重力に抗してる時は ±100〜500 程度になるのが典型
  - **±1000 近くで張り付く** = 最大トルクで踏ん張ってるが負け気味 → 個体劣化
  - **ほぼ 0** = モーターが踏ん張ってない → 電気/制御の問題

姿勢ドリフトも併記:
  - PosDrift = (測定中の Pres_Pos) - (Goal_Pos)
  - 0 = 完全に保持できている
  - 大きく (±50 以上) ずれる = 保持できていない = 落ちる

実行:
    python tests/check_holding_load.py
    python tests/check_holding_load.py --port /dev/ttyUSB0
"""

from __future__ import annotations

import argparse
import time


REG_TORQUE_ENABLE = 0x28
REG_GOAL_POS      = 0x2A
REG_PRESENT_POS   = 0x38
REG_PRESENT_LOAD  = 0x3C

IDS = [1, 2, 3, 4, 5, 6]
ID_TO_JOINT = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}


def parse_load(raw: int) -> int:
    """STS3215 Present_Load: bit10 が符号、bit0..9 が大きさ。"""
    sign = -1 if (raw & 0x400) else 1
    mag = raw & 0x3FF
    return sign * mag


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--samples", type=int, default=10,
                        help="平均する Pres_Load サンプル数 (既定 10)")
    parser.add_argument("--sample-interval", type=float, default=0.1,
                        help="サンプル間隔 [s] (既定 0.1s)")
    args = parser.parse_args()

    from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

    port = PortHandler(args.port)
    if not port.openPort():
        print(f"open {args.port} failed"); return 1
    port.setBaudRate(1_000_000)
    pkt = PacketHandler(0)

    try:
        # Step 1: 全 motor の torque off を保証
        for mid in IDS:
            pkt.write1ByteTxRx(port, mid, REG_TORQUE_ENABLE, 0)
        time.sleep(0.2)

        print("=" * 60)
        print("[1/4] アームを手で支えてください。")
        print("      ID3 (elbow_flex) に重力負荷がかかる姿勢:")
        print("        例) 上腕水平 + 前腕も水平で前方に伸ばす")
        print("      アームを支えたまま Enter")
        print("=" * 60)
        input()

        # Step 2: 現在位置をスナップ → Goal にコピー
        positions = {}
        for mid in IDS:
            pos, _, _ = pkt.read2ByteTxRx(port, mid, REG_PRESENT_POS)
            positions[mid] = pos
            pkt.write2ByteTxRx(port, mid, REG_GOAL_POS, pos)

        # Step 3: torque ON
        for mid in IDS:
            pkt.write1ByteTxRx(port, mid, REG_TORQUE_ENABLE, 1)

        print("[2/4] torque ON しました (Goal = 現在位置)")
        print("      ゆっくり手を離してください (落ちそうなら支えに戻して OK)")
        print("      支え無しの状態で 1〜2 秒待ってから Enter")
        input()

        # Step 4: 複数回サンプル
        print(f"[3/4] {args.samples} サンプル取得中...")
        load_sum = {mid: 0 for mid in IDS}
        drift_sum = {mid: 0 for mid in IDS}
        for _ in range(args.samples):
            for mid in IDS:
                load_raw, _, _ = pkt.read2ByteTxRx(port, mid, REG_PRESENT_LOAD)
                pos_now, _, _ = pkt.read2ByteTxRx(port, mid, REG_PRESENT_POS)
                load_sum[mid] += parse_load(load_raw)
                drift_sum[mid] += pos_now - positions[mid]
            time.sleep(args.sample_interval)

        print()
        print(f"{'ID':>3} {'Joint':14} {'Goal':>6} {'avg Load':>10} "
              f"{'avg PosDrift':>14}")
        n = args.samples
        for mid in IDS:
            ld = load_sum[mid] / n
            dr = drift_sum[mid] / n
            marker = ""
            if abs(ld) > 800:
                marker += "  ⚠️ Load 飽和"
            if abs(dr) > 50:
                marker += "  ⚠️ 保持できてない"
            print(f"{mid:>3} {ID_TO_JOINT[mid]:14} {positions[mid]:>6} "
                  f"{ld:>+10.1f} {dr:>+14.1f}{marker}")

        print("\n[4/4] torque OFF します。アーム支えてください。")
        input("支えたら Enter...")
    finally:
        # 必ず torque off にして戻す
        for mid in IDS:
            try:
                pkt.write1ByteTxRx(port, mid, REG_TORQUE_ENABLE, 0)
            except Exception:
                pass
        port.closePort()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
