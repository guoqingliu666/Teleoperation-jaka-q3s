# JAKAS5-Quest3s 伺服遥操作 V2.0.0

这是 2026-10-05 发布的可复现 Windows 影子/数字孪生版本。唯一入口是 `START_V2.cmd`。

已包含：Quest 3S 输入、Grip deadman、连续笛卡尔目标、阻尼雅可比、默认开启的低通与 Ruckig jerk 约束、125 Hz 最新槽状态发布、Unity V2Twin、TCP1/TCP2 目标标记、视角自由操作和日志。TCP1 按 `TCP2 local +Z 200 mm` 建模；`Home` 回正，`Tab` 展开详情。

本包不包含 JAKA 或 DH116 厂家 SDK，不连接或运动真机。DH116 Trigger 抓握仅包含模拟/接口原型，尚未接入日常闭环，也没有 EtherCAT、触觉或力控实机通过证据。公开包没有 CAD、原始现场日志、相机图像或用户私有配置。

## 解压与启动

1. 下载本 Release 的 `JAKAS5-Quest3s-servo-V2.0.0-windows.zip` 和 `SHA256SUMS.txt`；不要只下载 GitHub 自动生成的 Source code ZIP。
2. 用 `certutil -hashfile JAKAS5-Quest3s-servo-V2.0.0-windows.zip SHA256` 对照校验文件。
3. 解压到新的 D 盘英文路径，例如 `D:/ChatGPT/Reproductions/JAKA_Q3S_V2.0.0`，不要覆盖旧项目。
4. 在 Anaconda Prompt 进入解压后的 `V2_servo`：

```bat
powershell -NoProfile -ExecutionPolicy Bypass -File development/tools/restore_frozen_environment.ps1 -VerifyOnly
powershell -NoProfile -ExecutionPolicy Bypass -File development/tools/restore_frozen_environment.ps1 -EnvironmentPath D:/ChatGPT/Environments/jaka_q3s_v2_0_0
conda activate D:/ChatGPT/Environments/jaka_q3s_v2_0_0
python run_v2_app.py --check
START_V2.cmd
```

环境从包内 33 个精确 Conda 包离线创建，需要电脑已有 Conda。Quest Link、Meta/OpenXR 和显卡驱动需自行安装。点击“连续遥操作影子”，先松开 Grip，握住后移动；松开减速并解绑。它只驱动模拟反馈。

## 验证

```bat
set PYTHONPATH=%CD%/src;%CD%/.deps;%CD%;%CD%/tests
set OMP_NUM_THREADS=1
set OPENBLAS_NUM_THREADS=1
python -m unittest test_tool_frames test_tracking_observation test_continuous_pose test_shadow_telemetry test_global_kinematics test_input_gate test_runtime_paths -v
python -m v2_servo.teleop.continuous_shadow --synthetic --duration 60
```

开发证据与边界见 `docs/tcp_camera_fix_20261004.md`、`docs/runtime_user_guide.md`。真机连续伺服、全臂碰撞、真实停止距离、DH116/O30、触觉力控和 ACT 正式采集不属于本 Release 的通过范围。
