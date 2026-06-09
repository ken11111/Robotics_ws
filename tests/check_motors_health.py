"""6 サーボのトルク / PID / 温度 / 電圧 / 負荷を読み出して比較する診断ツール。

ID3 (elbow_flex) だけ保持トルクが弱い等の症状時、EEPROM 設定の個体差や
過熱保護を切り分けるのに使う。

STS3215 control table 抜粋 (Feetech SCS) :
  0x10 Max_Torque       (2 byte, EEPROM, 0..1000)
  0x15 P_Coefficient    (1 byte, EEPROM, 位置 P gain)
  0x16 D_Coefficient    (1 byte, EEPROM, 位置 D gain)
  0x17 I_Coefficient    (1 byte, EEPROM, 位置 I gain)
  0x28 Torque_Enable    (1 byte, RAM,    0=脱力)
  0x30 Torque_Limit     (2 byte, RAM,    0..1000  ← Max_Torque 範囲内)
  0x38 Present_Position (2 byte, RAM)
  0x3A Present_Speed    (2 byte, RAM, signed)
  0x3C Present_Load     (2 byte, RAM, signed, % of max)
  0x3E Present_Voltage  (1 byte, RAM, in 0.1V → / 10)
  0x3F Present_Temperature (1 byte, RAM, ℃)

実行:
    python tests/check_motors_health.py
    python tests/check_motors_health.py --port /dev/ttyUSB0
"""

from __future__ import annotations

import argparse


REG = {
    "Max_Torque":   (0x10, 2, "0..1000"),
    "P_gain":       (0x15, 1, "位置 P"),
    "D_gain":       (0x16, 1, "位置 D"),
    "I_gain":       (0x17, 1, "位置 I"),
    "Torque_Enable":(0x28, 1, "0=脱力"),
    "Torque_Limit": (0x30, 2, "0..1000"),
    "Pres_Pos":     (0x38, 2, "raw 0..4095"),
    "Pres_Speed":   (0x3A, 2, "raw signed"),
    "Pres_Load":    (0x3C, 2, "raw signed (% of max)"),
    "Pres_Volt":    (0x3E, 1, "× 0.1V"),
    "Pres_Temp":    (0x3F, 1, "℃"),
}

ID_TO_JOINT = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    args = parser.parse_args()

    from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

    port = PortHandler(args.port)
    if not port.openPort():
        print(f"open {args.port} failed"); return 1
    port.setBaudRate(1_000_000)
    pkt = PacketHandler(0)

    # ヘッダ
    print(f"{'ID':>3} {'Joint':14}", end="")
    for name in REG:
        print(f" {name:>14}", end="")
    print()

    rows = {}
    for mid in range(1, 7):
        row = {"ID": mid, "Joint": ID_TO_JOINT.get(mid, "?")}
        for name, (addr, size, _) in REG.items():
            if size == 1:
                v, comm, _ = pkt.read1ByteTxRx(port, mid, addr)
            else:
                v, comm, _ = pkt.read2ByteTxRx(port, mid, addr)
            if comm != COMM_SUCCESS:
                row[name] = None
            else:
                # 符号付きへの変換
                if size == 2 and v >= 0x8000:
                    v = v - 0x10000
                # 0x3E (電圧) は × 0.1V
                if addr == 0x3E:
                    v = v / 10.0
                row[name] = v
        rows[mid] = row
        print(f"{mid:>3} {row['Joint']:14}", end="")
        for name in REG:
            v = row[name]
            if v is None:
                print(f" {'N/A':>14}", end="")
            elif isinstance(v, float):
                print(f" {v:>14.1f}", end="")
            else:
                print(f" {v:>14}", end="")
        print()
    port.closePort()

    # 各列で「他と違う ID」を太字代わりに [diff] マークで指摘
    print()
    print("--- outlier 検出 (他と違う ID) ---")
    found_any = False
    for name in REG:
        vals = {mid: rows[mid][name] for mid in rows if rows[mid][name] is not None}
        if len(set(vals.values())) > 1:
            # 値ごとに ID を集計
            from collections import defaultdict
            by_val = defaultdict(list)
            for mid, v in vals.items():
                by_val[v].append(mid)
            # 「単独」の値があれば指摘
            for v, ids in by_val.items():
                if len(ids) == 1:
                    others = [m for m in vals if m not in ids]
                    other_vals = sorted(set(vals[m] for m in others))
                    print(f"  {name:14}: ID {ids[0]} ({ID_TO_JOINT[ids[0]]}) = {v}  ← "
                          f"他 5 個は {other_vals}")
                    found_any = True
    if not found_any:
        print("  (全 ID で全レジスタが一致)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
