# Robotics_ws — ロボティクス統合ワークスペース

**ステータス**: 🟢 **シム環境構築済** (2026-05-30) / 実機 SO-ARM101 到着待ち
**親プロジェクト**: [Spresense 防犯カメラシステム](file:///home/ken/Spr_ws/GH_wk_test/docs/security_camera/)

---

## 📋 このワークスペースは何か

Spresense 防犯カメラへの **LeRobot SO-ARM101 Pro 統合 (PTZ アーム追尾)** の実装・PoC スクリプトを置く独立ワークスペース。`~/Spr_ws/GH_wk_test/` (C/NuttX) や `~/Rust_ws/` (PC viewer) とは **言語・依存系を分離** している。

**設計方針**: 実機到着前にシム上でロジックを先行検証し、到着後は **同じ上位コードがそのまま動く** よう、最初から sim/real を切替可能な抽象 (`make_robot()`) を入れている。

### 設計の真実 source は別

| 区分 | 場所 |
|---|---|
| **設計・要求・STAMP/STPA・計画** (真実 source) | `~/Spr_ws/GH_wk_test/docs/security_camera/` |
| **PC viewer 実装** | `~/Rust_ws/security_camera_viewer/` |
| **本ws (LeRobot 実装)** | `~/Robotics_ws/` ← **ここ** |

本 ws の役割は **「実装と PoC 試験スクリプト」のみ**。設計判断は親プロジェクト docs/ に集約する。

---

## 🏗️ アーキテクチャ — 実機/シム切替

```
┌─────────────────────────────────────────────────┐
│ PTZ Logic Layer       (backend-agnostic)        │  ← ptz_integration/
│  ・tracking_controller.py (Visual Servo PID)    │     実機・シム共通
│  ・motion_to_servo.py     (bbox → joint angle)  │
│  ・e_stop_handler.py                            │
│  ・soft_limit.py                                │
└─────────────────┬───────────────────────────────┘
                  │ apply_targets / read_state / get_frame
┌─────────────────┴───────────────────────────────┐
│ Robot Abstraction     (so_arm101/__init__.py)   │  ← so_arm101/
│  RobotBase + make_robot("sim" | "real")         │
└────────┬────────────────────────┬───────────────┘
         │                        │
┌────────┴───────────┐  ┌─────────┴────────────────┐
│ SimBackend         │  │ RealBackend              │
│ so_arm101/sim/     │  │ so_arm101/real/          │
│  MuJoCo Menagerie  │  │  USB-RS485 + Feetech     │
│  ✅ 実装済         │  │  ⏳ 実機到着後           │
└────────────────────┘  └──────────────────────────┘
```

- `so_arm101/config/joint_limits.yaml` が **sim XML と real driver の共通真実 source**
- 上位 `ptz_integration/*` は MuJoCo にも Feetech SDK にも import 依存しない
- 実機側は LeRobot 0.4.4 の `lerobot.robots.so_follower.SOFollower` をラップする想定 (詳細は後述「実機統合の足がかり」)
- テストは backend をパラメータ化 (`--backend=sim` がデフォルト、実機到着後 `--backend=real` を追加)

---

## 📂 ディレクトリ構成

凡例: ✅ = 実装済 / ⏳ = 実機到着後 / 📁 = 空ディレクトリ (進行と共に埋まる)

| パス | 役割 | 状態 |
|---|---|---|
| `pyproject.toml` / `uv.lock` | 依存定義 (uv 管理) | ✅ |
| `.venv/` | Python 3.11 仮想環境 (gitignore 済) | ✅ |
| `so_arm101/__init__.py` | `RobotBase` 抽象 + `make_robot()` factory | ✅ |
| `so_arm101/config/joint_limits.yaml` | 可動範囲 (sim/real 共通) | ✅ |
| `so_arm101/config/calibration.yaml` | 実機ゼロ点オフセット | ⏳ |
| `so_arm101/sim/scene.py` | MuJoCo シーン組立 (MjSpec) | ✅ |
| `so_arm101/sim/backend.py` | `SimBackend(RobotBase)` | ✅ |
| `so_arm101/sim/run_sim.py` | viewer 起動スクリプト | ✅ |
| `so_arm101/sim/menagerie/` | sim 用 — git submodule (sparse: `robotstudio_so101` のみ、`MuJoCo Menagerie` から) | ✅ |
| `so_arm101/hardware/` | 実機ハード参照 — git submodule (sparse: SO101 関連のみ、`TheRobotStudio/SO-ARM100` fork から) | ✅ |
| `so_arm101/real/backend.py` | `RealBackend(RobotBase)` — LeRobot `SOFollower` + `scservo-sdk` をラップ | ⏳ |
| (find_port は CLI `lerobot-find-port` を使う方針、自作不要) | LeRobot 同梱 | ✅ (CLI として) |
| `ptz_integration/tracking_controller.py` | Visual Servo PID + HTTP サーバ (Phase X.3-X.4) | ⏳ |
| `ptz_integration/motion_to_servo.py` | bbox → サーボ角度変換 (Phase X.3) | ⏳ |
| `ptz_integration/e_stop_handler.py` | E-Stop ハード割込み (Phase X.4) | ⏳ |
| `ptz_integration/soft_limit.py` | 動作領域制限 — `joint_limits.yaml` を読む (Phase X.4) | ⏳ |
| `tests/x0_sim_smoke.py` | モデルロード + step + 限界整合チェック | ✅ |
| `tests/x1_servo_unit_test.py` | backend パラメータ化テスト | ⏳ |
| `docs/` | 本 ws 固有の運用ノート | 📁 |
| `data/` | 計測ログ / 試験データ (gitignore 済) | 📁 |

---

## 🚀 セットアップ

```bash
cd ~/Robotics_ws

# 1. Python 仮想環境 + 依存
uv venv .venv --python 3.11
source .venv/bin/activate
uv pip install -e .          # lerobot[feetech] (= scservo-sdk 同梱), mujoco, etc.

# 2. submodules を取得
#    - so_arm101/sim/menagerie/  (MuJoCo Menagerie sparse: robotstudio_so101)
#    - so_arm101/hardware/       (SO-ARM100 fork sparse: SO101/Optional 関連)
git submodule update --init --recursive

# 3. 受け入れ確認
python tests/x0_sim_smoke.py
# 期待出力: OK: sim backend loaded (13 qpos), 200 steps no NaN, 6 joint limits consistent
```

---

## 🎮 シミュレーション

### GUI viewer

```bash
source .venv/bin/activate
python -m so_arm101.sim.run_sim
```

MuJoCo の Passive viewer が立ち上がり、SO-101 と移動ターゲット (赤箱) が表示される。カメラは `[` / `]` で切替 — `external_cam` (Spresense 模擬) と `wrist_cam` (アーム手先) が選べる。

WSL2 で GUI が出ない場合: `MUJOCO_GL=egl python -m so_arm101.sim.run_sim` でヘッドレス描画にフォールバック (録画など用)。

### Python API から

```python
from so_arm101 import make_robot
import numpy as np

robot = make_robot("sim")           # 後で make_robot("real") に切替可能
robot.apply_targets(np.zeros(6))    # 関節角指令 (rad, qpos 順)
robot.step()                        # 1 tick 進める
q = robot.read_state()              # 現在角
frame = robot.get_frame()           # external_cam の BGR 画像
```

### シーン構成

`so_arm101/sim/scene.py` の `build_ptz_tracking_spec()` で:
- Menagerie の SO-101 (6 DOF) + 標準シーン (床・ライト・skybox)
- 固定外部カメラ `external_cam` (Spresense 模擬、アーム正面)
- 移動ターゲット (赤箱、freejoint で物理)
- ホーム姿勢 = 各 body の `pos` から決まる `qpos0` で表現 (keyframe は使わない)

二つの実装メモ:
- XML `<include>` の `meshdir` 伝播問題を避けるため、シーン拡張は `MjSpec` API で Python から組み立てている
- mujoco 3.9.0 の passive viewer が keyframe UI 初期化時に `mj_setKeyframe(-1)` を呼ぶ既知の不具合があるため、`<keyframe>` は意図的に追加していない (qpos0 で同等のホーム姿勢に到達できる)

---

## 🔌 実機統合の足がかり (Phase X.3 以降に着手)

実機 SO-ARM101 が届いた後の作業前提を整理。**いずれも未着手** (本ステップではコードを書かない、調査結果のみ記録)。

### LeRobot 0.4.4 の実 API (`pip install lerobot[feetech]` で取得済)

| 区分 | 実体 |
|---|---|
| ロボットクラス | `lerobot.robots.so_follower.SOFollower` |
| 設定 | `lerobot.robots.so_follower.SOFollowerConfig(port=..., use_degrees=True, ...)` |
| 主な API | `.connect() / .disconnect() / .calibrate() / .configure() / .setup_motors() / .send_action() / .get_observation()` |
| サーボ SDK | `scservo_sdk` (Feetech STS3215) — `feetech-servo-sdk` パッケージ |

### LeRobot CLI ツール (venv 有効化後そのまま実行可)

| コマンド | 用途 |
|---|---|
| `lerobot-find-port` | USB-RS485 アダプタのポート検出 |
| `lerobot-calibrate` | サーボのゼロ点・可動域キャリブレーション |
| `lerobot-find-joint-limits` | 実機の物理可動域を測定 (→ `joint_limits.yaml` 更新) |
| `lerobot-find-cameras` | 接続済みカメラ列挙 |
| `lerobot-info` / `lerobot-record` / `lerobot-eval` | 情報表示 / データ収集 / 評価 |

### ハード参照 (`so_arm101/hardware/`)

`TheRobotStudio/SO-ARM100` fork (`ken11111/SO-ARM100`) を sparse submodule で取得。**SO-101 関連のみ** に限定して 251 MB → working tree 157 MB に。

| パス | 用途 |
|---|---|
| `Simulation/SO101/so101_new_calib.urdf` | ROS2 / MoveIt / pybullet 等で URDF が必要になった場合 |
| `Simulation/SO101/so101_new_calib.xml` | Menagerie 派生元の **原典 MJCF** (シム調整内容を辿る際の比較用) |
| `Simulation/SO101/joints_properties.xml` | サーボ ID・ホーム位置などの実機側パラメータ参照 |
| `STL/SO101/` | 3D 印刷用モデル (実機部品を再印刷する場合) |
| `STEP/SO101/` | CAD ソース (マウント穴位置の改造などが必要な場合) |
| `Optional/` | グリッパー替え / カメラマウント (特に `Wrist_Cam_Mount_*`, `SO101_Wrist_Cam_Hex-Nut_Mount_32x32_UVC_Module`) |
| `README.md`, `3DPRINT.md`, `SO100.md`, `CHANGELOG.md` | 組立手順・部品表 |

スコープ外 (将来必要になれば `git sparse-checkout add` で足す): `Mini/`, `media/`, `STL/SO100`, `STL/Gauges`, `STEP/SO100`

### sim と hardware の住み分け

| 用途 | 参照先 |
|---|---|
| シミュレーション実行 | `so_arm101/sim/menagerie/robotstudio_so101/` (Menagerie 精製版) |
| URDF / ROS 系 | `so_arm101/hardware/Simulation/SO101/*.urdf` |
| 実機組立・3D 印刷 | `so_arm101/hardware/STL/SO101/` |
| シム調整の出典確認 | `so_arm101/hardware/Simulation/SO101/so101_new_calib.xml` ↔ Menagerie 派生版 |

### 設計上の整合タスク (実機到着前に解決推奨)

| 項目 | 内容 |
|---|---|
| 単位系 | `SOFollower` は `use_degrees=True` がデフォルト (度数法)。本 ws の `joint_limits.yaml` はラジアン (Menagerie 準拠)。`RealBackend` で deg ↔ rad 変換を担う |
| API シグネチャ | `SOFollower.send_action(dict)` vs 本 ws `RobotBase.apply_targets(np.ndarray)` のアダプタ層が `RealBackend` 内に必要 |
| 観測の単位 | `SOFollower.get_observation()` の戻り値構造を確認し、`RobotBase.read_state()` の np.ndarray (6,) に変換 |
| キャリブ運用 | `lerobot-find-joint-limits` の出力を `joint_limits.yaml` に統合するスクリプト (手動転記でも可) |

### 既知の未検証項目 (要追加検証)

| # | 項目 | 検証手段 |
|---|---|---|
| 1 | ホーム姿勢 = 全関節 0 の物理妥当性 (自己衝突しないか) | viewer で目視 / smoke test に collision count assertion |
| 2 | ターゲット位置 `(0.30, 0.05, 0.05)` がアーム可動域内か | IK で逆算 / SO-101 リーチ仕様確認 |
| 3 | `external_cam` の `xyaxes` が意図した方向を向いているか | viewer の Camera パネルで目視 |
| 4 | ターゲット質量 0.05 kg の妥当性 | 用途 (PoC) には影響軽微、後で要なら調整 |

---

## 🔗 親プロジェクト docs への参照

| 文書 | 用途 |
|---|---|
| [`safety_analysis/STAMP_STPA_PTZ_ARM_REFERENCE.md`](file:///home/ken/Spr_ws/GH_wk_test/docs/security_camera/02_specifications/quality/safety_analysis/STAMP_STPA_PTZ_ARM_REFERENCE.md) | 安全分析 draft (L-8/9, H-19〜22, CTRL-ARM, M-41〜44) — `joint_limits.yaml` 値設定根拠 |
| [`PTZ_ARM_POC_PLAN.md`](file:///home/ken/Spr_ws/GH_wk_test/docs/security_camera/05_future_actions/phase_planned/PTZ_ARM_POC_PLAN.md) | PoC 実施計画 (マイルストーン X.1〜X.5, 60-80 人日) |
| [`PTZ_ARM_PROTOCOL_SPEC.md`](file:///home/ken/Spr_ws/GH_wk_test/docs/security_camera/02_specifications/interface/PTZ_ARM_PROTOCOL_SPEC.md) | PC viewer ↔ Robotics_ws 通信プロトコル (HTTP/JSON) |
| [`PTZ_ARM_HW_DESIGN.md`](file:///home/ken/Spr_ws/GH_wk_test/docs/security_camera/02_specifications/architecture/PTZ_ARM_HW_DESIGN.md) | E-Stop ハード / Soft Limit / 配線設計 |

---

## 🧭 拡張ポリシー

| ケース | 対応 |
|---|---|
| 同 SO-101 で別シーン (pick-and-place 等) | `so_arm101/sim/scene.py` に builder 追加 / 新 module 作成 |
| 別ロボット同スタック (UR5 等) | `<robot>/` を top-level に追加、`sim/` `real/` の二段構成踏襲、venv 共有 |
| 別スタック (Isaac Sim / Gazebo / ROS2) | `<stack>/.venv/` + `<stack>/pyproject.toml` で完全分離 |
| LeRobot 公式 gym 環境 (aloha/pusht/xarm) 追加 | `pyproject.toml` の `lerobot` を `lerobot[aloha,pusht]` に変更し再 lock |

---

## 📝 改訂履歴

| Date | 変更内容 |
|---|---|
| 2026-05-27 | 初版 — スケルトン作成 (mkdir + README + .gitignore) |
| 2026-05-30 | シム環境構築 — uv venv + LeRobot + MuJoCo Menagerie (robotstudio_so101)。`RobotBase` 抽象 + `SimBackend` 実装。`joint_limits.yaml` を実機・シム共通 source に設定。smoke test グリーン |
| 2026-05-30 | 追加調査反映 — `lerobot[feetech]` extras 導入 (scservo-sdk 同梱)。LeRobot 0.4.4 の実 API (`SOFollower`, `lerobot-*` CLI) を README に記載。未検証項目を可視化 |
| 2026-05-31 | 実機ハード参照を準備 — `so_arm101/hardware/` に `TheRobotStudio/SO-ARM100` fork を sparse submodule で追加 (SO101 関連のみ)。URDF / STL / STEP / Optional マウント類が参照可能に |
