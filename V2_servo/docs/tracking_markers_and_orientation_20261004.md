# 双TCP标记、姿态映射与跟随诊断（2026-10-04）

后续修正：本文记录前一轮旧工具影子模型。当前已更正 TCP1 局部 +Z 200mm，见[工具与视角修复](tcp_camera_fix_20261004.md)；原截图/测试保留为历史对照。


本轮在development/workbench-hand-wrist推进，已发布的v2.0.0-shadow-smooth.20261004和本机冻结包保持原样。本轮实际实现双球/坐标轴/误差与约束诊断，并加入可选一比一姿态映射；没有解锁真机、修改JAKA接口、关闭滤波或删除奇异保护。

## 现在怎么用

1. 正常停止原影子任务并关闭旧V2Twin和主窗口。用项目根目录`START_V2.cmd`启动；不要打开releases或development中的副本。环境仍为jaka。
2. 日常入口新增“影子姿态映射”下拉框。默认“冻结半倍率（对照）”保持原算法。先用此模式看新增标记即可，不用再做历史逐轴测试。
3. 点“连续遥操作影子”。青色实心球是**当前模拟反馈关节FK的TCP**，紫色线球是**Grip绑定后映射到机器人基座的手柄目标**；两套轴均红X、绿Y、蓝Z。紫球不是头显追踪空间中的原始手柄位置，也不是机器人已到达的位置。
4. 初始松Grip，再握住平移/转腕。误差以mm/deg显示在状态面板和场景阶段提示。`JOINT_SPEED_LIMIT`表示关节速度约束导致整体缩放；`NEAR_SINGULARITY`为提前提示，原0.005保护阈值不变。
5. 松Grip时目标解绑并隐藏，青球继续显示减速中的模拟TCP。新鲜松开并停稳后再握会重绑；反馈超过250ms、缺少合法变换或错误坐标系时隐藏标记。明确停止包会立刻清除标记，不等待旧球过期。
6. 要对比姿态方式，点“停止影子任务”，等任务结束、关闭旧V2Twin，再选“一比一姿态（影子候选）”并启动。运行中下拉框不可切换。该候选修复映射的路径依赖，但尚未改善所有运动，可能更早触发奇异保护，不作为实机方案。
7. 软件停止用摇杆点击/“停止影子任务”，异常真机仍必须用实体急停；本影子入口没有机器人命令。F1/F2预设、鼠标自由旋转/平移/缩放保持原有操作。F12保存截图到当前会话data目录。

标记是辅助叠加显示，不受遮挡隐藏，因此**不能拿球是否穿过模型判断无碰撞**。真实控制器Tool1已改为[0,0,200]mm、姿态0，当前球明确标为旧Tool1影子模型；没有冒充新实机TCP。新标记暂只接收simulation+simulated_fk，拒绝用模型预测冒充live反馈。

## 已落实的代码

- `tracking_observation.py`：机器人基座、米、row-major 4×4变换；目标有效性、位置/姿态误差、源时间、模型SHA、最小奇异值、关节缩放、滤波前后twist、逆映射未限幅速度和OTG目标。观测不修改控制。
- `continuous_shadow.py` / `twin_publish.py`：每8ms将同一观测写审计日志并送独立UDP最新槽；文字仍后台5Hz。过期/终止不复用旧标记。
- `TwinTrackingMarkers.cs`：双球和两套轴，通过既有机器人基座变换转换坐标，不猜测欧拉顺序。反射基转换作用于每一根轴；新增无碰撞体。
- `continuous_pose.py`：保留默认增量半倍率路径，另加`anchored_one_to_one`。后者以握Grip时的手柄/机器人姿态为固定参考，直接使用相对旋转矩阵，避免逐帧缩放的路径依赖及180°主值跳变。
- `launcher.py`：同一入口选对照/候选，运行中禁止切换。没有增加另一个START。

## 两次最新手柄日志的量化结果

这两次是实际Quest输入、理想模拟机器人反馈；不是实机跟踪误差。重放原时间戳并复用记录的外部锁停状态，38084/10237个已记录周期的关节输出与原记录**最大差均为0**。原始失败周期不在sample行中，原报告的奇异保护失败单独保留，不能因已记录行重放完就写成完整运行通过。

| 指标 | 19:35会话 | 19:40会话 |
|---|---:|---:|
| 位置目标误差 P50 / P95 | 65.29 / 318.56 mm | 33.67 / 260.95 mm |
| 姿态目标误差 P50 / P95 | 2.09 / 27.59° | 4.22 / 53.43° |
| 关节速度缩放占跟随周期 | 37.87% | 40.22% |
| sigma<0.02占跟随周期 | 1.76% | 2.64% |
| 原控制循环P99 | 8.074ms | 8.085ms |
| Grip字段缺失拒绝 | 12781包 | 216包 |
| 跳变/超时拒绝 | 0 | 1包 |
| 手柄平移速度P95 | 367.14mm/s | 274.60mm/s |
| 有效握持输入超过120mm/s比例 | 36.87% | 22.80% |
| 原会话最终结果 | 接近奇异保护 | 接近奇异保护 |

结论：不仅是渲染卡顿。目标速度经常超过当前请求限速，约四成跟随周期又受关节约束，造成目标积累与姿态响应迟缓。源输入Grip有效性缺失会锁停；不能用取消deadman校验解决。上面的源速度由相邻新鲜原始源时间戳直接差分，未插值，峰值包含追踪噪声，不能作为人手真实速度标定。

## 已证实的旋转映射问题与候选边界

同一400帧多轴闭合手势（分段旋转，再回原姿态），冻结映射的目标残留旋转为**33.6842°**；一比一固定参考映射残留约**3.53e-31°**（浮点数值零）。这是目标映射实验，证明消除路径依赖，不等于实际末端跟踪零误差。纯单轴缩半与多轴旋转缩半不能直接等同。

一比一候选在相同原始日志重放中分别在第27935/9825行触发奇异保护，比原记录的38084/10237行更早。候选第二会话的姿态P95为81.05°，但因目标定义、有效时长都不同，**不直接用全程百分位与基线排序**。本轮没有据此宣称跟踪更好，也没有把它设为默认。候选只用于独立对比。

六轴完整6DoF通常没有额外冗余自由度，无法承诺任意目标同时严格跟随且绕开所有奇异点。后续将比较：自适应阻尼、约束速度求解、可达目标管理和提前制动；如需牺牲姿态或偏离路径，界面必须明确说明。不能通过删除保护阈值“通过”。

## 实际验证与证据

- 22项回归通过：目标绑定/释放/超时；观测不改变控制；UDP单位/来源；闭合路径与179→181°连续性；重握不跳；滤波、输入、停止、关节约束和模型测试。`data/tracking_markers_20261004/regression.txt`。
- Unity重新构建成功：`data/tracking_markers_20261004/unity_build_final.log`。沿用既有场景/模型，未重做CAD。
- 实际Player九项检查通过：绑定、释放、重绑、250ms过期、错误坐标系、假live、坏矩阵、缺tracking、坐标/轴。Python独立计算与Unity实际对象最大坐标差1.30435e-7m；这是软件变换一致性，不是现场标定精度。`data/tracking_markers_20261004/201918/report.json`。
- 实际双球截图：[末端双球与坐标轴](../data/tracking_markers_20261004/201918/MARKER_BOUND.png)。截图使用显式诊断相机，**普通Player用户相机没有被锁定或重定位**。
- 实际入口→一比一子进程→Unity→停止成功；模式运行中禁切/停止后可切均通过。`data/launcher_shadow_smoke_20261004_202141_271809/report.json`。此次没有有效跟随动作；只算进程/模式门验证。
- 该进程测试周期P99=8.119ms、最大8.473ms，规划P99=0.773ms；属于空闲输入测试，不代替带负载真机时序。实际窗口截图：[模式选择界面](../data/tracking_markers_20261004/launcher_window/launcher.png)。
- 本轮未佩戴头显验收可读性，未连接SDK/发送机器人运动，未验收真实Tool1/台面/手部/线缆碰撞。
- 首次Player探针没有等待Unity场景就绪导致漏记首阶段；已改成等待实际阶段回执后通过。初次UI截图捕获了遮挡窗口，改为捕获指定窗口后重新查看通过。失败证据保留。

## 可复现开发命令

在jaka环境和V2根目录：

```bat
set "PYTHONPATH=%CD%\src;%CD%\.deps;%CD%;%CD%\tests"
set OMP_NUM_THREADS=1
set OPENBLAS_NUM_THREADS=1
python -m unittest test_tracking_observation test_continuous_pose test_shadow_telemetry test_global_kinematics test_input_gate test_runtime_paths -v
python development/tools/compare_orientation_mapping.py
python development/tools/analyze_tracking_follow.py --sessions continuous_quest_20261004_193519_382992 continuous_quest_20261004_194047_304712 --out data/tracking_replay_new
python development/tools/analyze_tracking_follow.py --sessions continuous_quest_20261004_193519_382992 continuous_quest_20261004_194047_304712 --orientation one-to-one --out data/tracking_candidate_new
python development/tools/analyze_tracking_input.py --sessions continuous_quest_20261004_193519_382992 continuous_quest_20261004_194047_304712 --out data/input_replay_new.json
python development/tools/verify_tracking_markers.py
python development/tools/verify_launcher_shadow.py --one-to-one
```

这些是开发验证命令，输出目录必须新建，Player验证前正常关闭已有Player；日常只运行START_V2.cmd。原始手柄日志仅本机保存，公开冻结包不含私人会话。CSV和报告分别在`data/tracking_review_20261004/`、`data/tracking_one_to_one_20261004/`。

## 下一门槛

先用双球与轴识别平移/姿态的真实跟随差异；离线同条件比较奇异区策略时必须使用共同有效时段，报告提早失败和不可达目标，不只看平均误差。随后更新新Tool1数学模型、核对实验台实体碰撞及停止余量，再迁移真机。台面防穿透、Trigger和A/B腕部模式仍未完成，不能因本轮可视化通过而开放真机。
