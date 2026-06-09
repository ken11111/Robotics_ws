"""指定 1 モーターだけを sin スイープして、target vs present を比較する診断。

スティクション (静摩擦で初動が出ない) や、追従の鈍さを定量化する。
他の関節は torque ON で **現在位置を保持** するため、アーム姿勢は維持される。

実行:
    # 既定: ID3 (elbow_flex) を ±100 raw (≈9°) 4s 周期で 10s
    python tests/check_dynamic_tracking.py

    # 同じテストを ID2 (shoulder_lift) で比較
    python tests/check_dynamic_tracking.py --motor-id 2

    # 振幅を上げる (より大きい delta を要求)
    python tests/check_dynamic_tracking.py --amplitude-raw 200

読み方:
    max_err / rms_err  ... 目標 vs 実際の位置差 (raw)。大きい = 追従できてない
    max_load           ... モーターが使った最大トルク (%)
    stuck_count        ... 「目標は >5 動いたのに present は <1」のサンプル数
                          → 多い = スティクションで動き出せない瞬間が頻発
    "stuck zones"      ... どの位置 (raw) で stuck したか分布。
                          中央 / 端 / どこか特定位置で集中するなら機械干渉
"""

from __future__ import annotations

import argparse
import math
import time


REG_TORQUE_ENABLE = 0x28
REG_GOAL_POS      = 0x2A
REG_PRESENT_POS   = 0x38
REG_PRESENT_LOAD  = 0x3C

IDS_ALL = [1, 2, 3, 4, 5, 6]
ID_TO_JOINT = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}


def parse_load(raw: int) -> int:
    sign = -1 if (raw & 0x400) else 1
    return sign * (raw & 0x3FF)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--motor-id", type=int, default=3, choices=IDS_ALL,
                        help="スイープするモーター ID (既定 3=elbow_flex)")
    parser.add_argument("--amplitude-raw", type=int, default=100,
                        help="sin 振幅 raw counts (既定 100 ≈ 9°)")
    parser.add_argument("--period-s", type=float, default=4.0,
                        help="sin 周期 s (既定 4)")
    parser.add_argument("--duration-s", type=float, default=10.0,
                        help="計測秒数 (既定 10 = 2.5 周期)")
    parser.add_argument("--rate-hz", type=float, default=30.0,
                        help="ループ Hz (既定 30)")
    args = parser.parse_args()

    from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

    mid = args.motor_id
    print(f"動的スイープ test: ID {mid} ({ID_TO_JOINT[mid]})")
    print(f"  amplitude={args.amplitude_raw} raw "
          f"(≈{args.amplitude_raw*360/4095:.1f}°)")
    print(f"  period   ={args.period_s} s")
    print(f"  duration ={args.duration_s} s "
          f"({args.duration_s/args.period_s:.1f} cycles)")
    print(f"  rate     ={args.rate_hz} Hz "
          f"({int(args.duration_s*args.rate_hz)} samples)")

    port = PortHandler(args.port)
    if not port.openPort():
        print(f"open {args.port} failed"); return 1
    port.setBaudRate(1_000_000)
    pkt = PacketHandler(0)

    try:
        # 全 motor torque off
        for m in IDS_ALL:
            pkt.write1ByteTxRx(port, m, REG_TORQUE_ENABLE, 0)
        time.sleep(0.2)

        print("\n[1/3] アームを手で支え、安全な姿勢に。")
        print(f"      ID {mid} ({ID_TO_JOINT[mid]}) の現在位置 ±{args.amplitude_raw}")
        print(f"      raw が端 (0 / 4095) に当たらない位置で。")
        print("      支えたまま Enter")
        input()

        # 現在位置をスナップ → Goal にコピー → torque ON (全 motor)
        bases = {}
        for m in IDS_ALL:
            pos, _, _ = pkt.read2ByteTxRx(port, m, REG_PRESENT_POS)
            bases[m] = pos
            pkt.write2ByteTxRx(port, m, REG_GOAL_POS, pos)
        for m in IDS_ALL:
            pkt.write1ByteTxRx(port, m, REG_TORQUE_ENABLE, 1)

        print(f"\n[2/3] torque ON、ID {mid} の base position = {bases[mid]} raw")
        print("      ゆっくり手を離す → 1 秒待つ → Enter")
        input()

        print(f"\n[3/3] {args.duration_s}s スイープ中 (Ctrl+C で中断可)...")

        omega = 2 * math.pi / args.period_s
        dt = 1.0 / args.rate_hz
        n_samples = int(args.duration_s * args.rate_hz)
        records: list[tuple[float, int, int, int]] = []  # (t, goal, present, load)
        t0 = time.perf_counter()

        for i in range(n_samples):
            ts = i * dt
            goal = bases[mid] + int(args.amplitude_raw * math.sin(omega * ts))
            pkt.write2ByteTxRx(port, mid, REG_GOAL_POS, goal)
            pos, _, _ = pkt.read2ByteTxRx(port, mid, REG_PRESENT_POS)
            load_raw, _, _ = pkt.read2ByteTxRx(port, mid, REG_PRESENT_LOAD)
            records.append((ts, goal, pos, parse_load(load_raw)))
            slack = (t0 + (i + 1) * dt) - time.perf_counter()
            if slack > 0:
                time.sleep(slack)

    except KeyboardInterrupt:
        print("\nCtrl+C 検知")
        records = records if 'records' in dir() else []
    finally:
        for m in IDS_ALL:
            try:
                pkt.write1ByteTxRx(port, m, REG_TORQUE_ENABLE, 0)
            except Exception:
                pass
        port.closePort()

    if not records:
        return 0

    # 統計
    errors = [g - p for (_, g, p, _) in records]
    loads = [l for (_, _, _, l) in records]
    max_err = max(abs(e) for e in errors)
    rms_err = (sum(e * e for e in errors) / len(errors)) ** 0.5
    max_load = max(abs(l) for l in loads)
    mean_abs_load = sum(abs(l) for l in loads) / len(loads)

    # スティクション: goal が >5 動いたのに present が <1 のサンプル数
    stuck = 0
    stuck_positions: list[int] = []
    for i in range(1, len(records)):
        dg = abs(records[i][1] - records[i - 1][1])
        dp = abs(records[i][2] - records[i - 1][2])
        if dg > 5 and dp < 1:
            stuck += 1
            stuck_positions.append(records[i][2])

    print()
    print(f"--- 計測結果 ({len(records)} samples) ---")
    print(f"  max tracking error : {max_err:>5} raw  "
          f"(≈{max_err*360/4095:>5.1f}°)")
    print(f"  rms tracking error : {rms_err:>5.1f} raw  "
          f"(≈{rms_err*360/4095:>5.1f}°)")
    print(f"  max abs load       : {max_load:>5}  "
          f"({max_load*100/1000:>5.1f}% of max)")
    print(f"  mean abs load      : {mean_abs_load:>5.1f}  "
          f"({mean_abs_load*100/1000:>5.1f}%)")
    print(f"  stuck samples      : {stuck:>5}  "
          f"({stuck*100/len(records):>5.1f}% of run)")
    if stuck_positions:
        bmin, bmax = min(stuck_positions), max(stuck_positions)
        print(f"  stuck position range: [{bmin}, {bmax}] raw  "
              f"(base={bases[mid]}, offset {bmin-bases[mid]}..{bmax-bases[mid]})")

    # 抽出表示 (30 行ぶん)
    step = max(1, len(records) // 30)
    print()
    print(f"{'t[s]':>6} {'goal':>5} {'pres':>5} {'err':>+5} {'load':>+5}")
    for i in range(0, len(records), step):
        t, g, p, l = records[i]
        print(f"{t:>6.2f} {g:>5} {p:>5} {g-p:>+5} {l:>+5}")

    # 解釈ヒント
    print()
    print("--- 判定ヒント ---")
    if max_err < 20 and stuck < 3:
        print("  ✅ 健全: 追従誤差が小さく、stuck もほぼ無し")
    elif max_err > 60 and max_load > 800:
        print("  ⚠️  Load が飽和して追従できず → トルク不足")
    elif max_err > 60 and max_load < 200:
        print("  ⚠️  追従できないのに Load も低い → 制御 or 通信の問題")
    elif stuck > len(records) * 0.1:
        print("  ⚠️  stuck > 10% → スティクション/ギア引っかかりの可能性")
    else:
        print("  △ 中間的な結果。ID2 等と比較して判断")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
