# Robotics_ws 作業ログ

最終更新: 2026-06-09 (動的追従テスト完了、ID3 健全判定に更新)

このドキュメントは本 ws の **時系列の作業履歴 + 現状 + 中断状態からの再開手順** をまとめる。設計判断・アーキテクチャは [`README.md`](../README.md) を参照。

---

## 0. 一目で分かる現状

| カテゴリ | 状態 |
|---|---|
| sim 環境 (MuJoCo) | ✅ 完成 — `tests/x0_sim_smoke.py` グリーン |
| 実機セットアップ | ✅ ID 焼き + キャリブ完了 |
| 実機 → sim ミラー (read) | ✅ 動作確認済 — `tests/x1_real_smoke.py` / `x2_real_to_sim_mirror.py` |
| sim → 実機 ミラー (write) | ✅ 動作確認済 — `tests/x3_sim_to_real_mirror.py` |
| ID3 (elbow_flex) 動作 | ✅ **動的追従テストで健全と判定** (load 17%, stuck 0%) |
| Phase X.3+ (PTZ 統合) | ❌ 未着手 |

最新 commit: `325bc4a sim → 実機 ライブミラー (apply_targets 実装)`

---

## 1. マイルストーン履歴

### Phase X.1〜X.2: sim 環境構築 (2026-05-30 〜 05-31)

- uv venv (Python 3.11) + `lerobot[feetech]` + `mujoco` で依存固定
- `so_arm101/__init__.py` に `RobotBase` ABC + `make_robot("sim" | "real")` ファクトリ
- `SimBackend`: MuJoCo Menagerie の `robotstudio_so101` を **`MjSpec` API で組立** (XML `<include>` の `meshdir` 伝播問題回避)
- `so_arm101/config/joint_limits.yaml` を sim/real 共通の真実 source に
- `MuJoCo Menagerie` を sparse git submodule で取込 (`so_arm101/sim/menagerie/`)
- `TheRobotStudio/SO-ARM100` fork を実機ハード参照として別の sparse submodule で取込 (`so_arm101/hardware/`)
- Commits: `4ca1ad5`, `bea0843`

### 実機セットアップ (2026-06-06 〜 06-07)

| 段階 | コマンド | 結果 |
|---|---|---|
| ① WSL2 USB attach | (PowerShell) `usbipd bind --busid 1-1; attach --wsl --busid 1-1 --auto-attach` | CH343 が `/dev/ttyACM0` で見える |
| ② サーボ ID 焼き | `lerobot-setup-motors --robot.type=so101_follower --robot.port=/dev/ttyACM0` | 6 個に ID 1〜6 焼込 |
| ③ キャリブ | `lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=so101_follower_v1` | JSON 保存 (`~/.cache/huggingface/lerobot/.../so101_follower_v1.json`) |
| ④ 本 ws 連携 | シンボリックリンク `so_arm101/config/calibration.json` → cache、`.gitignore` で除外 | — |
| ⑤ 通信確認 | `python tests/x1_real_smoke.py` | 全 6 関節の現在角が degrees で読めることを確認 |

Commit: `6f310b2`

### sim ↔ real 双方向ミラー (2026-06-07 〜 06-08)

- `RealBackend(RobotBase)` 実装 (`so_arm101/real/backend.py`)
  - read 側: `get_observation()` の dict (deg) → qpos 順 ndarray (rad) 変換
  - write 側: `apply_targets(rad)` → joint_limits clip → np.rad2deg → dict → `SOFollower.send_action()`
  - 単位境界 (rad ↔ deg) はこのクラス内に閉じ込め、上位は rad 一本
  - `torque` パラメータで read-only / write モード切替 (read-only は手で動かせるよう disable_torque)
  - `max_relative_target` で LeRobot 内部の delta クリップを設定可能
- `tests/x2_real_to_sim_mirror.py`: 実機を手で動かす → sim viewer の SO-101 が追従 ✅
- `tests/x3_sim_to_real_mirror.py`: sim 内の sin スイープが実機を動かす ✅ (ただし ID3 で症状)
- Commits: `80cef18`, `325bc4a`

### ID3 (elbow_flex) 動作異常の調査 (2026-06-08 〜 06-09)

ID3 だけ動きが渋く、人が初動を与えると追従する症状を発見。原因切り分け中。

| 診断 | スクリプト | 結果 |
|---|---|---|
| EEPROM レジスタ比較 | `tests/check_motors_health.py` | 全 6 個でほぼ全項目一致 (gripper の Torque_Limit=500 は設計通り)。ID3 温度のみ +2〜3℃ |
| 静的保持トルク測定 | `tests/check_holding_load.py` | ID3 Load=+28/1000 (3%), drift=+4 raw → **静的には正常保持**。ただし測定姿勢 (ID3 が中央付近で負荷小) が問題を再現できていなかった可能性 |
| 動的追従テスト | `tests/check_dynamic_tracking.py` | **✅ ID3 健全**: max tracking error 41 raw (3.6°), max load 17.2% (飽和には程遠い), stuck samples 0 — スティクションも追従不足も検出されず |

詳細は [§3 現状の課題](#3-現状の課題) 参照。

---

## 2. 主要ファイルマップ

### 本番コード

| パス | 役割 |
|---|---|
| `so_arm101/__init__.py` | `RobotBase` ABC + `make_robot()` factory + `load_joint_limits()` |
| `so_arm101/config/joint_limits.yaml` | sim/real 共通の関節範囲 (rad, Menagerie 準拠) |
| `so_arm101/config/calibration.json` | 実機キャリブ JSON のシンボリックリンク (gitignore 済) |
| `so_arm101/sim/scene.py` | MjSpec でシーン組立 (Menagerie + 外部カメラ + ターゲット) |
| `so_arm101/sim/backend.py` | `SimBackend(RobotBase)` |
| `so_arm101/sim/run_sim.py` | 単体 viewer 起動 |
| `so_arm101/sim/menagerie/` | git submodule (sparse: `robotstudio_so101`) |
| `so_arm101/real/__init__.py` | docstring に「組立補助は LeRobot 公式 CLI 使う方針」 |
| `so_arm101/real/backend.py` | `RealBackend(RobotBase)` — SOFollower ラップ |
| `so_arm101/hardware/` | git submodule (sparse: SO101 関連、URDF/STL/STEP/Optional) |

### テスト・診断スクリプト

| パス | 種別 | 用途 |
|---|---|---|
| `tests/x0_sim_smoke.py` | smoke | sim ロード + 200 step + joint_limits 整合 |
| `tests/x1_real_smoke.py` | smoke | 実機 6 関節の現在角を 1 回読み出し |
| `tests/x2_real_to_sim_mirror.py` | デモ | 実機 → sim ライブミラー (read) |
| `tests/x3_sim_to_real_mirror.py` | デモ | sim → 実機 ライブミラー (write) |
| `tests/check_motors_health.py` | 診断 | 6 モーター EEPROM レジスタ比較 |
| `tests/check_holding_load.py` | 診断 | 静的保持トルク (Pres_Load) を 6 個比較 |
| `tests/check_dynamic_tracking.py` | 診断 | 単一モーターを sin スイープし target vs present を比較 |

---

## 3. 現状の課題

### ✅ ID3 (elbow_flex) 動作異常 → 解決 / 健全判定

**最初の症状** (2026-06-08 sim → 実機 ミラー初回実行時):
- ID3 だけ追従が遅れる、ログに `'elbow_flex': {'original goal_pos': -9.33, 'safe goal_pos': 8.33}` 連発
- 持ち上げて手で初動を与えると動き始める
- 静止状態で「少しだけ持ち上げて手を離すと落ちる」

**切り分け結果**:
| 診断 | 結果 |
|---|---|
| `check_motors_health.py` | レジスタ全項目で他 5 個と一致 (gripper の 500 は設計通り)。温度のみ +2〜3℃ (弱い傍証) |
| `check_holding_load.py` | Load=+28/1000 (3%), drift=+4 raw → 静的保持は正常 (ただし測定姿勢の負荷小) |
| `check_dynamic_tracking.py` | **max tracking error 41 raw (3.6°), max load 17.2%, stuck 0/300** — スティクションも追従不足も検出されず |

**最有力解釈**: コネクタの一時的接触不良
- 初回 sweep ログの "safe goal_pos = +8 固定" は、Pres_Pos のサンプリング失敗 (古い値を返す → delta 大きいまま → clamp 連発) で説明可能
- 12V 再投入や接続見直しで安定し、現在は dynamic にも正常

**「落ちる」現象の解釈**: torque ON モーターは Goal_Position に向かって動く。手で動かしてから手を離すと、Goal に戻る = 落ちて見える。診断スクリプトは「Goal = 現在位置」を先に書いてから torque ON するので保持できた (正常挙動)。

**残作業 (任意)**:
- ID2 や他関節で同じ dynamic テストを走らせて、ID3 が他と同等の健全さであることを再確認したい場合: `python tests/check_dynamic_tracking.py --motor-id 2`

### 🟡 viewer 終了時の segfault (低優先)

**症状**: `python tests/x2_*.py` を Ctrl+C で終了すると `GLXBadDrawable` → Segfault

**原因**: WSLg + MuJoCo 3.9 passive viewer の片付け不全。本 ws のコードバグではない。

**回避**: Ctrl+C ではなく **viewer ウィンドウの ✕ ボタン** で閉じれば `viewer.is_running()` が False になりループ自然終了 → 正常 close。

---

## 4. 作業再開ガイド

### 4.1 環境チェック (毎回)

```bash
cd ~/Robotics_ws
source .venv/bin/activate

# sim 健全性
python tests/x0_sim_smoke.py
# 期待: OK: sim backend loaded (13 qpos), 200 steps no NaN, 6 joint limits consistent
```

### 4.2 実機を触る場合の手順

1. **USB attach** (Windows 側 PowerShell):
   ```powershell
   usbipd attach --wsl --busid 1-1 --auto-attach
   ```
   `auto-attach` で抜き差し後も自動再接続。終了は Ctrl+C で auto-attach プロセスを止める。
2. **WSL で確認**:
   ```bash
   ls /dev/ttyACM*
   # /dev/ttyACM0 が出れば OK
   ```
3. **電源**: 12V 投入
4. **疎通**:
   ```bash
   python tests/x1_real_smoke.py
   ```
   全 6 関節の角度が表示されれば OK。

### 4.3 PTZ 統合 (Phase X.3+) に進む場合

未着手。順序は概ね:

1. `ptz_integration/soft_limit.py` — `joint_limits.yaml` を読んで動作領域制限
2. `ptz_integration/motion_to_servo.py` — bbox → サーボ目標角変換
3. `ptz_integration/tracking_controller.py` — Visual Servo PID + HTTP サーバ (PC viewer 連携)
4. `ptz_integration/e_stop_handler.py` — E-Stop ハード割込み

設計の真実 source は親プロジェクト docs:
- `~/Spr_ws/GH_wk_test/docs/security_camera/05_future_actions/phase_planned/PTZ_ARM_POC_PLAN.md`
- `~/Spr_ws/GH_wk_test/docs/security_camera/02_specifications/interface/PTZ_ARM_PROTOCOL_SPEC.md`

---

## 5. 学んだこと (memory にも保存済み)

### 公式の組み込み方法を実装前に確認する

`feedback_verify_integration_pattern_first.md` / `feedback_dont_truncate_investigation.md` を参照。本 ws の作業中に 2 回露呈した同じパターン:

1. **MuJoCo Menagerie の組込**: XML `<include>` で書いたが meshdir 伝播問題で失敗。Menagerie 同梱の `tutorial.ipynb` を見ていれば `MjSpec` API ですぐ正解にたどり着けた。
2. **`lerobot-setup-motors` 自作**: `entry_points.txt` を `head -10` で切ったため公式 CLI を見逃し、70 行の自作ラッパーを書いた (後で削除)。`head -10` で結論する前に **全件確認** すべきだった。

教訓:
- third-party の素材を使う時は「**どう組み込むのが公式の流儀か**」を先に確認する
- 「公式に無い」を根拠に自作する前に、**entry_points / scripts / docs / 同梱 tutorial の 4 箇所** を全件確認する
- リスト系の出力 (`entry_points.txt`, `pip show --files` 等) は head/grep で打ち切らない

---

## 6. コミット履歴 (主要のみ)

```
325bc4a sim → 実機 ライブミラー (apply_targets 実装)             2026-06-08
80cef18 RealBackend (read-only) と sim ライブミラーを追加        2026-06-07
6f310b2 実機セットアップ完了 (LeRobot 公式 CLI で ID 焼き + キャリブ)  2026-06-07
bea0843 SO-ARM100 fork を実機ハード参照として sparse submodule で追加  2026-05-31
4ca1ad5 LeRobot シミュレーション環境構築 (SimBackend + Menagerie SO-101)  2026-05-30
8e81ff6 Robotics_ws スケルトン作成 (README + .gitignore)         2026-05-27
```
