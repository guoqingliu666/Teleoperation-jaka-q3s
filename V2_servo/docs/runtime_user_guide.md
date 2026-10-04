# V2 流畅影子基线 · v2.0.0-shadow-smooth.20261004

用户确认流畅的 Quest → 连续规划 → Unity 模拟反馈基线。125Hz发布、实际桌面约72Hz应用；滤波开启。
**本包不包含JAKA/DH116厂家SDK，不允许真机运动；台面碰撞、Trigger手部、A/B独立腕部旋转尚未实现。**
最新用户会话最终触发接近奇异位形保护，不能称为全部动作/全空间通过。

## 下载复现

下载Release的 `-windows-public.zip` 和 `SHA256SUMS.txt`，不要只下载GitHub自动Source code ZIP。
解压至新的D盘目录，不覆盖旧项目。在Anaconda Prompt进入解压后的V2_servo目录：

```bat
powershell -NoProfile -ExecutionPolicy Bypass -File development/tools/restore_frozen_environment.ps1 -VerifyOnly
powershell -NoProfile -ExecutionPolicy Bypass -File development/tools/restore_frozen_environment.ps1 -EnvironmentPath D:\ChatGPT\Environments\v2_shadow_public
conda activate D:\ChatGPT\Environments\v2_shadow_public
python run_v2_app.py --check
START_V2.cmd
```

安装器从包内33个精确Conda包离线创建环境，不改旧jaka。若同名环境存在，换一个新D盘路径。Python 3.12.14、numpy 2.5.2、Ruckig 0.12.2，Ruckig二进制及Pillow截图依赖在.deps。需要电脑已有Conda；Quest Link/OpenXR与显卡驱动需另行安装配置。

检查预期：`SDK_connection_attempts=0`，`Player_present=true`、`ruckig_import=true`；SDK未随公开包提供是正常情况。
点击“连续遥操作影子”，先松Grip再握持自然移动/转动，松Grip减速，摇杆点击停止；不会运动真机。

离线验证：
```bat
set "PYTHONPATH=%CD%\src;%CD%\.deps;%CD%;%CD%\tests"
set OMP_NUM_THREADS=1
set OPENBLAS_NUM_THREADS=1
python -m unittest test_continuous_pose test_global_kinematics test_shadow_telemetry test_runtime_paths -v
python -m v2_servo.teleop.continuous_shadow --synthetic --duration 60
```

Unity工程在digital_twin/unity，版本2022.3.62f3；预编译Player已包含，不必重新导入CAD。开发在新分支进行；回滚请重新解压同一Release，不覆盖当前工作目录。

## 已知限制

旧数学模型对应旧Tool1。用户2026-10-04已把真实Tool1改为[0,0,200]mm、RX/RY/RZ=0；本冻结影子保留旧模型以复现体验，不可用于新的真实TCP。台下模型穿透尚无整臂碰撞限制。旋转第六轴即使TCP位于轴线上，也会改变姿态，只可能保持位置。

真实连续伺服、全臂/工具/手/线缆碰撞边界、实际停止距离、手部触觉与力控均未在本版本验收。公开版启动器禁用历史真机按钮；没有上传原始现场会话或厂家SDK。

运动核/滤波/参数与冻结本机版本相同；公开分发只修改启动器SDK诊断与真机按钮、SDK路径和数学证据脱敏引用。差异及哈希见PUBLIC_EXPORT.json、RELEASE_MANIFEST.json。

本项目自有代码沿用仓库许可；第三方二进制、依赖、Unity资源保留各自来源及权利，不因此自动变为MIT。请遵守THIRD_PARTY_NOTICE.md，不将本包当成厂家授权的真机系统。
