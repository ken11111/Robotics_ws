"""Real-hardware backend for SO-ARM101.

Phase X.3+ で `RealBackend(RobotBase)` を実装する場所。

組立時の補助は LeRobot 公式 CLI をそのまま使う方針 (自作しない):
- `lerobot-setup-motors --robot.type=so101_follower --robot.port=...`
- `lerobot-calibrate     --robot.type=so101_follower --robot.port=...`
- `lerobot-find-joint-limits --robot.type=so101_follower --robot.port=...`
"""
