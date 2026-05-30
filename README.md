# Robotics_ws — ロボティクス統合ワークスペース

**ステータス**: 🟡 **スケルトン作成済** (2026-05-27) / Phase 12 PoC 着手待ち
**親プロジェクト**: [Spresense 防犯カメラシステム](file:///home/ken/Spr_ws/GH_wk_test/docs/security_camera/)

---

## 📋 このワークスペースは何か

Spresense 防犯カメラへの **LeRobot SO-ARM101 Pro 統合 (PTZ アーム追尾)** の実装・PoC スクリプトを置く独立ワークスペース。`~/Spr_ws/GH_wk_test/` (C/NuttX) や `~/Rust_ws/` (PC viewer) とは **言語・依存系を分離** している。

### 設計の真実 source は別

| 区分 | 場所 |
|---|---|
| **設計・要求・STAMP/STPA・計画** (真実 source) | `~/Spr_ws/GH_wk_test/docs/security_camera/` |
| **PC viewer 実装** | `~/Rust_ws/security_camera_viewer/` |
| **本ws (LeRobot 実装)** | `~/Robotics_ws/` ← **ここ** |

本 ws の役割は **「実装と PoC 試験スクリプト」のみ**。設計判断は親プロジェクト docs/ に集約する。

---

## 📂 ディレクトリ構成

| ディレクトリ | 役割 | 着手 Phase |
|---|---|---|
| `lerobot/` | LeRobot SDK (pip install or submodule で取得) | X.1 |
| `so-arm101/config/` | サーボ可動範囲のキャリブレーションデータ | X.1 |
| `ptz_integration/` | Spresense 防犯カメラ連携スクリプト | X.3-X.4 |
| ├── `tracking_controller.py` | Visual Servoing PID (M-42) + HTTP サーバ | X.3-X.4 |
| ├── `motion_to_servo.py` | bbox → サーボ角度変換 | X.3 |
| ├── `e_stop_handler.py` | E-Stop ハード割込み (M-41) | X.4 |
| └── `soft_limit.py` | 動作領域制限 (M-41) | X.4 |
| `tests/` | PoC マイルストーン別テストスクリプト | 各 Phase |
| `docs/` | 本 ws 固有の運用ノート (設計は親 docs/ 参照) | as-needed |
| `data/` | 計測ログ / 試験データ (`.gitignore` 推奨) | X.4-X.5 |
| `.venv/` | Python 環境 (uv venv で作成、`.gitignore` 済) | X.1 |

---

## 🚀 セットアップ (Phase X.1 で実施予定)

```bash
cd ~/Robotics_ws

# 1. Python 仮想環境
uv venv .venv --python python3.11
source .venv/bin/activate

# 2. LeRobot SDK + 依存
uv pip install lerobot fastapi uvicorn pyserial opencv-python numpy

# 3. SO-ARM101 接続確認
python -m lerobot.scripts.find_port  # USB-RS485 ポート検出
python -c "from lerobot.common.robot_devices.robots.so_arm101 import SOArm101; ..."

# 4. PoC X.1 スクリプト
python tests/x1_servo_unit_test.py
```

---

## 🔗 親プロジェクト docs への参照

| 文書 | 用途 |
|---|---|
| [`safety_analysis/STAMP_STPA_PTZ_ARM_REFERENCE.md`](file:///home/ken/Spr_ws/GH_wk_test/docs/security_camera/02_specifications/quality/safety_analysis/STAMP_STPA_PTZ_ARM_REFERENCE.md) | 安全分析 draft (L-8/9, H-19〜22, CTRL-ARM, M-41〜44) |
| [`PTZ_ARM_POC_PLAN.md`](file:///home/ken/Spr_ws/GH_wk_test/docs/security_camera/05_future_actions/phase_planned/PTZ_ARM_POC_PLAN.md) | PoC 実施計画 (マイルストーン X.1〜X.5, 60-80 人日) |
| [`PTZ_ARM_PROTOCOL_SPEC.md`](file:///home/ken/Spr_ws/GH_wk_test/docs/security_camera/02_specifications/interface/PTZ_ARM_PROTOCOL_SPEC.md) | PC viewer ↔ Robotics_ws 通信プロトコル (HTTP/JSON) |
| [`PTZ_ARM_HW_DESIGN.md`](file:///home/ken/Spr_ws/GH_wk_test/docs/security_camera/02_specifications/architecture/PTZ_ARM_HW_DESIGN.md) | E-Stop ハード / Soft Limit / 配線設計 |

---

## 📝 改訂履歴

| Date | 変更内容 |
|---|---|
| 2026-05-27 | 初版 — スケルトン作成 (mkdir + README + .gitignore) |
