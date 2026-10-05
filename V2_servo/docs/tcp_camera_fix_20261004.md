# TCP1 与视角修复（2026-10-04，本地开发版）

2026-10-05 完成收尾；保留原验证文件的 `20261004` 命名。当前运行文件 SHA256、汇总检查和冻结包完整性见 `data/tcp_view_fix_20261004/final_verification.json`。

本轮修复了统一入口中的工具模型、TCP 标记、启动视角和遮挡面板。前一轮只补了标记，影子控制仍沿用旧工具数学模型；这与用户现在的 TCP1 定义不一致。本轮同时改计算与显示，不只挪动小球。已发布的冻结 Release 和历史官方 FK 原始报告保持不变。

## 正确的坐标关系

用户确认控制器 TCP1 为 `[0,0,200] mm, RX=RY=RZ=0`，TCP2 为第六轴末端传感器中心，Z 轴为第六轴转轴。本轮统一使用 **TCP1 / TCP2** 的用户命名，不把名字 TCP2 当成已读取的控制器工具槽 2。

关系为 `T_base_TCP1 = T_base_TCP2 × TransZ(0.200 m)`：

- TCP1 位置 = TCP2 位置 + TCP2 的局部 Z 单位向量 × 0.2 m。
- 两个坐标系的姿态相同。不是沿世界 Z 轴加高度。
- 只改变 J6、其余关节保持不变时，TCP1 位置不变，但 X/Y 轴随腕部旋转；因此 **TCP 位置不变，不等于完整位姿不变**。

运行默认模型由历史官方 FK 模型去掉原工具 `[-23.703,-60.146,200.963] mm, RZ=45°`，恢复法兰参考系，再增加新偏移。空间关节轴不变，末端参考变换与雅可比的线速度部分同步更新。来源配置见 `config/tcp_frames.json`，实现见 `src/v2_servo/teleop/tool_frames.py`。启动报告包含历史模型 SHA256、新工具配置 SHA256、旧参数和推导关系。

**标定边界：**恢复的法兰参考系作为 TCP2 的数学参考，Z 轴与识别的 J6 轴一致；实体传感器中心与该原点的装配偏差仍未独立测量。本轮未读取当前控制器 TCP1/TCP2 数值，不把用户配置派生模型声称为新一轮官方 FK 或真机标定验收。CAD 模型的毫米级注册误差也未因此消失。

## 为什么以前转腕会看起来偏

同一关节姿态，旧、新工具位置差 **64.655246 mm**，姿态差 **45°**。旧工具存在横向偏置，J6 从参考位置转到 180° 时，旧工具点最大移动 **129.296148 mm**；新轴上 TCP1 的 73 点离线扫描最大位置变化 **0.0000002145 mm**，为浮点/识别数值误差。这是数学比较，不是真机精度。

正确工具点消除了这一项错误。固定阻尼、关节速度缩放、接近奇异位形时的停机与姿态半倍率仍独立存在；不能因此宣称所有跟踪问题或奇异点避让已解决。本轮没有修改滤波、速度、jerk、安全限值或真机许可。

## 界面改变与逐点击使用

1. 点击统一入口的“停止影子任务”，等任务结束，正常关闭旧 V2Twin 和主窗口；正在运行的旧进程不会自动加载新构建。
2. 双击根目录 `START_V2.cmd`，点“连续遥操作影子”。无需机器人上电，本模式没有 SDK 连接或运动命令。
3. 起始视角正对实验台：桌面保留观察台面的俯角，取消相对台边的斜向朝向。VR 世界锚点仅含水平朝向，不把桌面俯角叠加到头显追踪空间。真实转头、俯仰、歪头仍跟随头显。
4. 中央不再出现随头部移动的大面板。桌面保留一行状态；`Tab` 展开/收起详细诊断。VR 展开时，详情放在当前右前方的世界固定位置，此后不跟头移动；看不到时先收起再展开即可重新放置。
5. `Home` 或 `F1` 回到正向起点；头显有效追踪时重置观察锚点。`F2/F3/F4` 仍可用，鼠标操作仍可进入自由视角。桌面左拖环绕、右拖转头、中键或 Shift+左拖平移、滚轮缩放。VR 鼠标左右拖调整水平朝向、上下拖调整观察高度，中键平移、滚轮前后移动；俯仰和侧倾由真实头动提供，避免鼠标倾斜整个追踪空间。
6. 收到新鲜模拟反馈后：**金色球 TCP2 REF** 是恢复的法兰参考原点；**青色球 SIM TCP1** 是沿蓝色 Z 方向延伸 200 mm 的点；两者有连线和 RGB XYZ 轴。紫色线球 HAND GOAL 为手柄映射目标。此处均不是实测真机反馈。
7. 松开再握右 Grip，绑定目标后自然移动/转动。松 Grip 解绑并减速；目标失效或反馈过期会隐藏对应标记。详细误差在 `Tab` 面板及启动器日志中。

## 本轮可复现证据

| 验证 | 结果 | 原始记录 |
|---|---|---|
| Python 回归 | 26 项通过，含 250 组关节角的局部 200 mm 关系、J6 位置不变/姿态改变、雅可比数值导数、停止和滤波既有检查 | `data/tcp_view_fix_20261004/regression.txt` |
| 工具对照 | 73 点 J6 扫描及完整派生模型、来源/配置 SHA256 | `data/tcp_view_fix_20261004/tool_math_report.json`, `derived_TCP1_model.json` |
| 实际 Unity 标记 | 12 项通过；包含原始坐标轴转换、200 mm 局部 Z 关系、松手、伪 live 拒绝和超时隐藏 | `data/tracking_markers_20261004/210217/report.json` |
| 实际 Player 视角与面板 | 10 项通过：默认隐藏、展开后面板固定、旋转/平移/缩放、桌面水平、合成头部姿态下重力方向不变及自由模式不重复叠加头姿 | `data/tcp_view_fix_20261004/player/report.json` |
| 统一入口实际启动/停止 | Tk → Python 影子 → Player 启动、运行和停止通过；本次未实际操纵手柄 | `data/launcher_shadow_smoke_20261004_210442_753045/report.json` |
| 相同合成输入、滤波、速度/加速度/jerk 限值下的新旧工具对照 | 旧工具 1078 周期/8.624 s 触发近奇异保护；新 TCP1 完成 7500 周期/60 s，无故障，最大相对起点位移 153.754 mm | `data/tcp_view_fix_20261004/old_tool_control/report.json`, `synthetic_follow/report.json` |
| Unity 构建 | 当前 Player 重建成功 | `data/tcp_view_fix_20261004/unity_build_final.log` |

实际进程中 OpenXR 为 active，但本轮没有佩戴头显做主观视野/立体感验收。相机锚点检查含合成头部姿态，不能写成真人佩戴通过。MuJoCo 的历史 `Tool1_TCP_candidate_unverified_Link06_relation` 标记不属于这次 Unity/影子链修复；不得拿旧 Link06 候选标记当当前 TCP1。本轮不改现场传感器或 CAD 装配。

60 秒对照使用虚拟时间和理想关节反馈，不是实际 8 ms 调度、真实手柄新会话或真机跟踪验收，也不能证明一般性的奇异点主动避让。它说明修正工具参考点改善了这一条特定轨迹；保护阈值没有放宽。

截图：

![不再遮挡的默认视角](../data/tcp_view_fix_20261004/player/default_unobstructed.png)

![TCP2 到 TCP1 的局部 Z 轴延伸](../data/tracking_markers_20261004/210217/MARKER_BOUND.png)

在 `jaka` 环境、项目根目录重做：

```bat
set PYTHONPATH=src;.deps;.;tests
set PYTHONDONTWRITEBYTECODE=1
set TEMP=D:\ChatGPT\Temp
set TMP=D:\ChatGPT\Temp
python -m unittest test_tool_frames test_tracking_observation test_continuous_pose test_shadow_telemetry test_global_kinematics test_input_gate test_runtime_paths -v
python development\tools\audit_tcp_frames.py
python development\tools\verify_tcp_view.py
python development\tools\verify_tracking_markers.py
python development\tools\verify_launcher_shadow.py
```

Player 测试会拒绝抢占已有 V2Twin；先正常关闭自己的窗口。复测产生新时间戳目录，不与旧报告混用。发布基线回滚仍使用原冻结 Release；不要手工混合新 Python 与旧 Player。
