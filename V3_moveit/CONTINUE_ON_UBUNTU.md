# Ubuntu 接续记录

## 当前工程位置（2026-11-14 用户更正）

Ubuntu 当前工程是 `/home/gq/projects_lgq/V3_moveit`。后续修改、启动、构建、缓存和日志均使用此目录。D 盘目录保留为历史来源；下文带日期的旧记录描述当时事实，不是当前路径指令。

此前 Ubuntu 改进实际落在 D 盘。此次先备份 home 原文件，再复制已完成的源码、设备配置、相机依赖和原始报告；构建和显示组件从当前目录重新生成。原文件备份及迁入清单位于 `outputs/directory_recovery/20261114T081311Z/`。历史报告保留原内容和原路径，不能把复制视为重新验收。当前恢复进度与实际验证结果见 `artifacts/directory_recovery_current.json`。

用户授权新建 V3，复用 MoveIt Servo、OMPL、ros2_control 与厂家 SDK，保留可用的 Windows 工程。当前工作目录为 `/home/gq/projects_lgq/V3_moveit`；build/install/log、缓存和项目环境均留在此处。V2_servo 和 QuestPoseBridge 保持只读。

## DH116 安装方向更新（2026-11-13，11.13 装配）

已按用户提供的 `（11.13）手臂.SLDASM` 更新安装方向。原文件为 `/home/gq/projects_lgq/JAKA-S5_开发资料_2026-09-07/整合模型/（11.13）手臂.SLDASM`，只读副本与更新前模型保存在 `reference/hand_mount_20261113/`。原生装配 XML 经过 CRC 校验；旧 XML 的 66 个组件变换与此前 SolidWorks COM 导出一致。新旧装配只有 DH116 位姿变化，相对腕部旋转约 180°。手掌安装 yaw 从约 +45.083° 改为 −134.917°。

`src/v3_moveit/description/s5.urdf` 的 17 个手部显示几何和 6 个碰撞几何已同步更新；机械臂运动学、TCP、快换 A/B、连接件、网格及碰撞包络尺寸保持不变。厂家 A1 活动手指模型与此前 A4/A1 表面配准继续使用，本次不是实物标定或手指驱动验收。`START_OPERATOR.sh` 每次启动都从当前模型重建显示 URDF，避免复用旧方向缓存。来源、矩阵及变更列表见 `src/v3_moveit/config/hand_mount.json`。

编译、21 项离线检查和整套 ROS fake 回归通过，实际 Qt 画布渲染通过。首次 ROS smoke 曾触发“Servo generation/scene validation exceeded 6 ms”；同配置独立复查和整套回归随后通过，未放宽时序、速度或碰撞阈值，首次原因未完全确定。失败原始记录在 `artifacts/hand_mount_ros_first_failure_20261113/`；最终退出码在 `artifacts/hand_mount_checks_20261113.json`。仍有此前已知的组件退出清理告警，不称为全部组件干净退出。

更新前的 Quest 仿真和真机报告已记录通过，仍保留其历史事实；它们对应旧模型。当前模型的 Quest 仿真、实机反馈/负载/质心检查、保持与真机停止验收需按上位机流程更新。此次模型更新没有连接机械臂或发送运动指令。当前软件状态与交付记录见 `artifacts/hand_mount_delivery_20261113.json`。

## Ubuntu 上位机：V3 Operator（2026-11-13）

运行 `bash START_OPERATOR.sh`，或在 Ubuntu 应用菜单搜索 **V3 Operator**。使用 `bash START_OPERATOR.sh --monitor` 可同时连接所选的相机/反馈监看设备。界面默认仿真；打开窗口和连接监看设备都不会启动机械臂。

- **遥操作**：上位机内部的 3D 画布显示 S5、快换件和 DH116；三路相机（腕部深度、左目 RGB、C922 全局 RGB）在画布内部左侧竖排，只显示三个 16:9 画面，名称、在线点及必要标注叠在图像内，没有外层大底板。点击画面可在场景内放大查看，按 Esc 或关闭按钮返回，右侧停止入口保持可见。左键旋转、中键平移、滚轮缩放，可复位视角。没有独立 RViz 显示窗口。机械臂读取当前会话反馈；无反馈时明确显示模型预览。DH116 六轴使用厂家角度反馈，联动关节按厂家 URDF 映射；无手指反馈时不能将模型姿态视为实物状态。
- **示波器**：黑底网格，独立选择 J1–J6，切换位置/速度、5/10/30 秒窗口、暂停查看；显示采样仅可选择 1/5/10/20/30/60 Hz，不改变控制周期。实际显示率受反馈源限制；过期或无效反馈不会继续画成当前信号。
- **设备连接**：选择 JAKA S5、Quest 3S、DH116、ZED-M、Logitech C922，保存为三套独立方案。工作正常为绿色，异常/缺流为红色，未选用或尚未启动为灰色。监看期间先停止监看再改设备。支持本地 USB 相机或 ROS 图像来源，以及独立的厂家手指反馈 ROS 域/话题；禁止把传感器订阅配置到控制/显示专用域 89、90、91。
- **明暗调节**：顶部月亮/太阳滑块连续调整页面明暗并保存。
- **会话控制**：Quest USB 重连、本次实时权限、仿真/真机选择及常驻启停。系统认证只为当前界面进程及其子进程授予实时优先级，关闭后失效。停止控制会话只发送 SIGINT，等待控制器退出；关闭窗口会等待控制会话安全退出。

日常使用时，在遥操作页选择“真机”，勾选“现场准备已完成”，点击“启动真机遥操作”。本次面板首次需要实时权限时会弹出系统认证，通过后继续刚才的启动。保持右 Grip 松开，听到就绪提示再握住跟随。驱动会核对当前工具、TCP、负载、质心、模型与实时反馈。“检查与验收”供开发调试时主动运行 12 秒保持和 60 秒跟随、停止检查，结果保存在本机。

日常操作：连接 USB → Quest 打开 `https://localhost:8443` 并进入透视 → 选择设备方案与仿真/真机 → 启动 → 等就绪提示音 → 握 Grip 跟随、松 Grip 停止、按右摇杆软件停止 → 停止会话。

验收页会将仿真跟随、真机跟随和两种停止的结果分别归档。日常遥操作读取当前站点配置和控制柜反馈。

界面以 [Apple 布局原则](https://developer.apple.com/design/human-interface-guidelines/layout)和[材质原则](https://developer.apple.com/design/human-interface-guidelines/materials)作为参考，采用内容优先、稳定层级、直接操作、渐进展开与语义状态色。Qt/Linux 的实现没有宣称使用 macOS 原生 Liquid Glass。3D 渲染宽高使用同一个缩放因子限制像素预算，新尺寸帧到达前保持旧帧比例；鼠标坐标使用同一显示矩形反算。页面随窗口重新布局，图像不被横向拉伸。启用高 DPI 图像和分数缩放支持。

### 相机与灵巧手接入状态

ZED 使用 [厂家官方 SDK 与 Python API](https://github.com/stereolabs/zed-python-api)。SDK 5.5.1、依赖、NEURAL_LIGHT 模型和独立 `.venv-zed` 位于当前工程的 `.cache/operator/`/工程目录内；没有修改 Anaconda 或系统 CUDA。依赖版本在 `tools/requirements_zed.lock`。C922 已取得真实 USB RGB。已有 ZED-M 实测记录使用 1280×720 左目 RGB 和 NEURAL_LIGHT 深度（相机 30 fps，界面预览约 10 Hz）；连续 20 次状态采样约 9.4 秒都有新鲜 RGB/有效深度，记录见 `artifacts/operator_zed_verified.json`，这不是深度测量精度标定。当前默认改为数据采集均衡模式 1920×1080@30，尚待该模式的实机确认。设备连接页现可单独启动/停止 ZED-M、C922，也可分别启动或停止所选反馈模块。C922、ZED 左目和深度已同时显示在 3D 内的左侧竖排视图并截图核对；预览保留最高 1280 像素宽度，方便展开查看细节。

相机曾出现 USB `-71` 通信错误；安装 ZED-M 专用访问规则并由用户完全重新插拔后恢复。规则入口 `bash tools/setup_zed_usb.sh` 只针对 `2b03:f681/f682`，不更改机器人网络。以后连接失败可先检查 USB 3.x 线缆/端口，在设备页停止并重新连接监看；程序也会重试已经退出的 USB 采集线程。

DH116 厂家资料位于 `/home/gq/projects_lgq/灵巧手_DH116`，已只读使用其模型和 `HandJointState` 消息接口；未启动厂家的 EtherCAT 主站或电机初始化示例。默认订阅 `/hand_control/realtime_state`（ROS domain 0，可配置）。用户说明手与 JAKA S5 二号机共用交换机；厂家主站文档要求 EtherCAT 网口专用、不设 IP/DHCP，因此手的实物反馈仍待独立网口/正确拓扑接入，不能对现有 JAKA 网口直接启动 SOEM。手目前显示“无当前手指反馈”。

### 验证边界

40 项上位机/门禁/数据/缩放测试和 20 项离线测试通过。传感器 ROS 联通测试使用合成数据，验证 RGB/深度格式、DH116 六轴与联动关系、报警、过期时间戳和断流，不能替代真实手指/深度验收。仿真观察器已从旧的 0.5 rad / 0.15 m 范围对齐用户授权的 30° / 1 m；60 秒时长、控制速度、碰撞及停止保护不变。33 项观察器测试通过，记录于 `artifacts/operator_access_observer_tests.log`。接入流程的真实 Qt 点击与只读状态读取通过，见 `artifacts/operator_access_visual.json`；GUI → fake 反馈 → SIGINT 退出通过，见 `artifacts/operator_smoke.json`。新 3D 画布通过实际 Qt 截图检查（无截图拼接），并核对隐藏渲染器、旋转、平移、缩放与复位。实际窗口分别检查 1240×800、1640×1040、2480×1400、3440×1440，再返回常用尺寸；渲染与绘制均保持宽高比。首帧深浅主题、相机展开/Esc 返回和停止入口可见性均通过，记录于 `artifacts/operator_resize_check.json`。

GUI → ROS fake 会话 → 反馈更新 → SIGINT 停止链路及完整 ROS 回归分别记录于 `artifacts/operator_smoke.json` 和 `artifacts/operator_access_ros.log`。已知 MoveIt move_group 退出时可能出现 -11 异常，报告保留 `cleanup_warning_seen`；不能声称整个 ROS 清理过程无异常。本轮新增 3 次只读 SDK 检查、0 次 Servo 使能；当前记录累计 SDK / Servo 为 31 / 15。

首次构建显示组件使用 `bash tools/build_operator_display.sh`；只构建只读显示器与厂家 ROS 消息接口。截图、模型来源和验证结果见 `artifacts/operator_left_final_light.png`、`operator_left_final_dark.png`、`operator_canvas_check.json`、`operator_model.json`、`operator_sensor_smoke.json`、`operator_delivery.json`。

## 当前状态：2026-11-13 真机跟随方向已确认，继续验证松握与摇杆停止

当前源码 SHA256：`b8c378ad719580258d6318accb84603495d4afbd97f2e645ec1a77bbc3d57244`。本节优先于下方历史记录。

- 柜端负载已只读核对为 2.344393883477562 kg，质心 [-13.745727540130591, 1.471682228026082, 57.40741097290345] mm；TCP1=[0,0,200,0,0,0]。未调用负载/TCP setter。原始记录 `artifacts/payload_readback_20261113T082422Z/`。
- 修复前完整实机栈在 Grip 始终松开时发生目标 jerk 超限，随后 Servo 退出。末端反馈最大变化约 0.037 mm，关节离角度限位至少约 42.6°；不是观察范围或关节限位报错。最新姿态的离线模型碰撞检查为无碰撞，但不是现场环境或物理制动验收。证据 `artifacts/handoff_jerk_failure_20261113/`、`artifacts/handoff_pose_geometry.json`。
- 已定位上游 JTC 4.40.1 对位置命令、位置/速度反馈组合的启动行为：无法完整从 command interfaces 读取位置/速度时，回退到较新的反馈作为 HOLD 起点。这会与 JakaSystem 已经保持的目标不一致。SmoothTrajectoryController 现在保留现有位置命令，确认反馈静止且靠近该命令后，以零 v/a 建立相同 HOLD；真实反馈不被改写，原 Guard 及 v/a/j 限值不变。全未初始化命令仍支持 GenericSystem 引导，部分无效、运动中或偏差过大的接管被拒绝。
- `tools/controller_activation_tests.cpp` 使用真实控制器与内存 loaned interfaces，旧代码复现 jerk 超限，修复后 8 个用例、320 次严格 Guard 写入通过，未加载机器人 SDK。当前源码构建、`tools/verify_offline.py` 与完整 `tools/verify_ros.sh` 均通过；日志 `artifacts/activation_before_fix.log`、`artifacts/activation_after_fix.log`、`artifacts/activation_fix_build.log`、`artifacts/activation_fix_verify_ros.log`。
- 新版本已重采状态/TCP/负载/FK 及 mode1/mode0 各 12 秒反馈，通过记录 `artifacts/hardware_preflight_current.json`。随后原位 HOLD 12.002 秒通过，最大关节漂移约 0.004°，Servo 自动退出且 launch 正常结束；证据 `artifacts/activation_fix_hold_20261113/`。首次真实 Quest 跟随后累计实际 SDK 会话 28 次、Servo 使能 15 次。
- 独立终端 PID 331611 已由用户临时授予 rtprio=50:50，实际控制线程 FIFO/50 已核实。加入 `LD_BIND_NOW=1` 后未跟踪的原位保持通过，完整栈也运行到上述接管问题；此前约 8.5 ms 写入阻塞的根因仍未证明。不得把一次通过写成最坏时延保证。未修改永久系统配置。
- 修复了独立终端下检查退出时未转发 SIGINT 的问题：HOLD 工具显式 `ros2 launch --noninteractive`；保留已通过真人测试的 Quest 观察器原样，从终端运行时用 `</dev/null`，避免 launch 误判为交互 Ctrl-C。两次原始退出失败均保留，后续正常 SIGINT 清理和 Servo 退出另有记录：`artifacts/rt_hold_failure_20261113/`、`artifacts/trace_hold_20261113/`。跟踪工具改变时序，跟踪运行不能替代无跟踪验收。
- USB 输入地址仍为 `https://localhost:8443`。唤醒手柄后，新版 60 秒真实 Quest 仿真通过：有可见跟随、松 Grip 和右摇杆停止后的指令/反馈静止证据，最大 TCP 位移约 44.76 mm；记录 `artifacts/quest_commissioning_fake/result.json`。此前 TRACKING_LOST 的未完成运行保留为历史记录。
- 新版首次实机 Quest 跟随已发生，用户确认“看到跟随，方向符合预期”。本轮最大 TCP 位移约 10.50 mm、姿态变化约 2.55°，第 5 轴变化约 2.88°触发观察器的 0.05 rad 退出阈值，未观察到松 Grip 边沿；Servo 退出及进程清理已确认，没有接管 jerk 或运行时控制故障。证据 `artifacts/activation_fix_first_motion_20261113/`。这不是 Grip 停止通过，下一轮只轻移约 2 mm 随即松握，随后另查摇杆停止。
- 后续实机已记录 5 次松 Grip 后指令与反馈静止，用户确认“是，松开后停住且不再跟随”；原始运行因之后累计位移超出观察阈值仍记为失败，部分停止证据单独保存在 `artifacts/activation_fix_grip_stops_20261113/`。右摇杆停止尚未验证。
- 用户明确指定本轮观察器为单关节 30°、末端 1000 mm、硬件时长 60 秒，生产控制速度与碰撞保护不变。硬件记录容量由 25000 增至 50000 帧以覆盖加倍时长。33 项观察器测试与 20 项离线测试通过；还未在此扩大范围下连接真机。首次新观察器仿真因连续握持约 30 秒而超出仿真观察范围，失败记录保留。加入只订阅输入状态的双声开始、单声松握、三声摇杆提示后，60 秒复验通过：最大关节变化约 25.97°、TCP 82.08 mm，松握和 SOFTWARE_STOP 均有静止确认；报告 `artifacts/quest_commissioning_fake/result.json`。随后会话被中断，尚未开始新范围下的真机检查；进程核对确认当前无 ROS 控制器或观察器残留。改动与授权记录 `artifacts/observer_range_change_20261113/`。
- 普通硬件门禁保持 `teleoperation_ready=false`，实机摇杆停止与完整短会话仍待完成。1:1 映射、20 mm 自碰减速、碰撞/奇异位形及所有时效保护均保留。

## 当前轨迹架构与边界

`FixedServoNode` 复用上游 Servo C++ API 的 IK、碰撞/奇异位形缩放与关节边界处理，随后用原生 Ruckig 在固定 8 ms 命令时间网格上生成真实 q/v/a，并保留每轴 jerk 相位切换点。它不再用上游滑动窗口的割线速度，也不把墙钟抖动重标成 Ruckig 步长。窗口明确包含 32 ms 未来命令，停止延迟须考虑这部分。真实周期、发送截止期和输入/反馈新鲜度仍独立检查；未放宽任何 v/a/j 或时效门槛。

OMPL 经单个在途 `GetMotionPlan` 服务返回几何路径；`PathRuckigRetiming` 仅删除重复/严格共线冗余点，保留路径转角并在转角停稳，重建实际曲线与端点零 v/a，随后交 `ValidateSolution`。每个 8 ms 可取消起点用运行时同一 `JointStopProfile` 做位置范围、场景和路径约束的离散预检查；FixedServo 同样在新段提交前检查实际段及停止包络。超过预算或检查失败不提交相应轨迹。这些不是连续碰撞证明，旧场景的包络不能在环境变化后继续作为验收依据。

`SmoothTrajectoryController` 保留上游 JTC 正常插值；取消在同一控制更新循环实际运行 Ruckig 制动，发送终点零速度/零加速度命令后才确认 CANCELED，期间拒绝覆盖。Supervisor 等待动作终态及命令、反馈的连续静止证明，再用精确命令位置恢复 Servo。硬件最终 Guard 对所有目标保持原限值，没有豁免起步、停止或切换。

固定命令网格通过不等于任意连续时刻插值精度通过。`artifacts/servo_interval_native/result.json` 中 `fixed_grid_contract_passed=true`，但 `continuous_curve_precision.passed=false`：扩展连续速度跟随测试中的 24 ns 极短相位段出现离网格五次插值数值误差，最大加速度差约 1.92747 rad/s²。不得将该结果宣传为全连续曲线精度、柜端插补或真实停止安全证明。

独立合成输入由仅用标准库的子进程持续发送，ROS 记录器与发包分离；显式 stop 才断流，stdin EOF/清理会退出子进程。两次活动发包间隔最大值为 8.727/8.790 ms。总内存记录上限 20000 帧，保留原始输入状态、发送时刻、轨迹与最终 write 证据；这修正了先前记录器暂停 195 ms 误造输入失鲜的测试设计，不改生产 80 ms 保护。

## 接续顺序

1. 只在 `/home/gq/projects_lgq/V3_moveit` 工作。需要补装依赖或重编时运行 `bash INSTALL_UBUNTU.sh`，使用系统 ROS/Python 环境；ROS apt 包安装到系统，项目产物留在当前工程。
2. 修改控制、协议、SDK 或模型后运行 `tools/verify_offline.py` 与 `bash tools/verify_ros.sh`。核对报告的源码摘要、`passed`、严格最终 write 审计及退出日志，文件存在不代表通过。
3. 独立场景脚本为 `tools/fixed_servo_ros_test.py`；必须已加载 Jazzy 与当前 install 环境。它只启动 fake，使用隔离 domain/UDP。不可用它替代 Quest 或实机验收。
4. 当前 USB `https://localhost:8443` 透视输入已连接，新跟随方案正在由用户体验；记录前后/左右/上下、旋转、Grip 重接管及摇杆停止。当前 fake 已停止；后续先完成 `tools/hardware_quest_commissioning.py` 的有界真实输入记录（Servo-only，`auto_replan=false`），避免重复启动。不要把合成输入和真实 Quest 接到同一输入会话；网页刷新重置序号后需重启输入桥。
5. `artifacts/hardware_follow_proposal.json` 所述配置变更已获用户明确授权，新采集和原位保持通过。日志阻塞修正已完成长时 fake 诊断；下一步先记录真实 Grip 松开停止，再做分阶段实机运动检查。`RUN_HARDWARE.sh --hardware` 会进入伺服，不是只读入口。目前 `hardware_readiness.json` 的 `teleoperation_ready=false` 保留；不能因软件测试通过跳过物理停止和现场方向验收，前三次诊断、首次失败和第二次通过的保持检查均不能记为 Quest 实机遥操作或物理停止验收通过。

本站机器人地址 `192.168.110.224`，Ubuntu 有线地址 `192.168.110.91`；旧默认 `.96` 不在本机，本站配置为 `site_ubuntu.yaml`。SDK 在途 RPC 尚不能可靠取消，历史慢查询不能因换 C++ 就称为解决。未启用动态障碍感知，几何模型与 65 姿态一致性也不证明物理停止距离。

以下原样保留早期过程结论与证据名称，均为历史信息；当前软件结果以上方与对应版本 artifacts 为准，实时运行状态应另行检查。

## 历史记录：2026-11-12 Ubuntu 执行记录

- 实际工作目录：`/media/gq/software/ChatGPT/Projects/VR/V3_moveit`，为原 D 盘 NTFS 挂载；本次 build/install/log、临时、ROS 与 colcon 用户目录均留在此处。`/home/gq/projects_lgq/V3_moveit` 是独立副本，本次未在该副本构建。
- 已清除继承的其他 ROS 工作区 overlay 后编译，`colcon build` 成功；SDK 动态库依赖齐全。编译仅有 ros2_control 旧 on_init 接口弃用警告。
- 17 项离线测试、65 姿态 FK 核对及原生 guard 测试通过。旧 Windows 绝对路径在 Linux 不可用，源清单校验数量为 0；不能据此声称重新验证了旧文件哈希。
- 安装脚本已尝试，停在 sudo 密码；缺失包为 `python3-aiohttp` 和 `ros-jazzy-moveit-servo`。请在本机终端执行 `sudo apt-get update && sudo apt-get install -y python3-aiohttp ros-jazzy-moveit-servo`。
- ROS 集成验证和可视仿真均已执行，均因 package moveit_servo not found 退出。安装缺失包后继续这两项并处理真实运行差异；当前没有仿真通过证据。
- 未运行 hardware 模式，未启动任何机器人 SDK 会话。Quest 头显验证和实机验收未进行。

## 历史记录：2026-11-12 补装依赖后的继续执行

- 用户已安装 `python3-aiohttp` 与 `ros-jazzy-moveit-servo` 2.12.4。安装所需依赖与编译已完成；未将最初因 sudo 中断的安装脚本标记为整段成功。
- 在干净的系统 Python/ROS 环境重跑 `bash tools/verify_ros.sh`，退出码 0。17 项离线检查和 guard 测试通过；mock 协调链实测关节运动约 0.0106993 rad，最大速度约 0.0836077 rad/s，输入断流停止、障碍碰撞状态响应通过。
- `bash RUN_SIM.sh` 已启动 RViz，OpenGL 4.6 初始化且桌面窗口存在。`arm_controller` 和 `joint_state_broadcaster` 均 active，场景 ready。
- 运行自带 `TRY_SYNTHETIC_INPUT.sh` 后检测到约 0.0170295 rad 最大关节位移，结束后 IDLE、held=false。仿真窗口在记录时保持运行。连接 Quest 前按 README 重启仿真，避免延续测试输入源。
- 最新依据：`artifacts/ros_smoke_pass.json`、`artifacts/rviz_sim_verification.json`、`artifacts/ubuntu_execution_status.json`。前一次缺包失败报告已保留为 `artifacts/ros_smoke_failed_missing_dependency.json`。
- 日志限制：运动过程中包含 OMPL 规划/执行，尚未独立隔离纯 Servo 跟随；注障测试没有测量停止距离。未配置动态 Octomap 传感器，未启用 FIFO 实时调度。MoveGroup 自身执行管理器未配 controller_names，本项目由 supervisor 直接调用 JTC。
- smoke 发送 SIGINT 清理时出现 Python invalid-context/KeyboardInterrupt 堆栈；测试运行期未出现 FAULT。这是退出清理问题，不能将日志描述为完全无错误。
- 本轮只运行 fake，真实机器人连接和命令均为 0。Quest 头显输入和实机验收仍未进行。原 `/home/gq/projects_lgq/V3_moveit` 副本保持未改动。

## 历史记录：最终 Ubuntu 记录：2026-11-12

- 依赖、Ubuntu 编译、17 项离线检查、C++ guard 与新增 14 项规划回归通过。最后一轮 ROS 功能测试关节运动 0.0109341 rad，最大速度 0.0650037 rad/s；输入断流停止和障碍碰撞状态响应通过。
- 首轮 RViz 曾因规划取消崩溃：MoveIt 2.12.4 在 allow_trajectory_execution=False 时没有 PlanExecution，但 MoveGroup action 的 cancel 回调仍调用 stop。已将本项目改为 GetMotionPlan 纯规划服务，保留单个在途请求、Epoch 失效校验和 JTC 轨迹取消；关闭不用的两个 MoveGroup action capability。
- 修改文件：src/v3_moveit/v3_moveit/supervisor.py、quest_bridge.py、scene_loader.py，src/v3_moveit/launch/bringup.launch.py，tools/ros_smoke.py、verify_ros.sh，并新增 tools/supervisor_tests.py。原文件和首轮崩溃日志保存在 artifacts/before_planning_service_fix。
- 修复后 RViz/模拟输入持续观测 70.02 秒，8687 个关节样本，无运行期 FAULT/崩溃；实际丢弃 1 次过期规划结果，结束后 IDLE。已重启留下一份全新 fake 输入会话，当前 servo_code=0、held=false、scene_ready=true；日志 artifacts/rviz_live.log。
- 功能通过不代表正常退出通过：最后 smoke 和长测关闭时，MoveGroup 仍在 CallbackGroup 析构中段错误；长测关闭时 quest_bridge 另有 rclpy 转换异常。Python 重复 shutdown/重复 SIGINT 已做必要修正，但上游退出问题尚未解决。参考 https://github.com/moveit/moveit2/pull/3828；未修改系统 ROS 库。
- 当前证据：artifacts/ubuntu_execution_status.json、ros_smoke_pass.json、rviz_sim_verification.json、rviz_live_status.json；旧 Windows build_status 已备份为 artifacts/build_status_windows.json。
- 仍未进行 Quest 头显测试、实机连接或运动，也没有独立验收纯 Servo、碰撞停止距离/时间或实时调度。所有修改和产物均在原 D 盘，home 副本及 V2/QuestPoseBridge 未改动。

## 历史记录：三入口顺序复跑：2026-11-12

- 已按顺序实际执行 INSTALL_UBUNTU.sh（退出码 0）、tools/verify_ros.sh（功能断言通过、退出码 0）和 RUN_SIM.sh（当前 fake/IDLE，RViz 保持打开）。
- 安装入口现在逐个检查 18 个依赖，只安装缺失项；全部已装时跳过 apt，但仍检查 SDK、运行离线验证并编译。三个入口均将 ROS_HOME/COLCON_HOME 固定到项目内。
- 日志为 artifacts/install_ubuntu_latest.log、verify_ros_latest.log、rviz_live.log；当前状态见 artifacts/ubuntu_execution_status.json。
- 已知 MoveGroup 在 SIGINT 后的析构崩溃仍复现，不能把功能通过说成退出清理通过。实机和 Quest 头显仍未验证。


## 历史记录：Quest 接入与实机检查：2026-11-12

当前结论：Quest 已进入透视并有真实右手输入到达 fake 仿真；S5 的状态、TCP/负载、模型及 EDG mode 1/mode 0 无运动反馈检查通过，但实机遥操作未放行。

- 用户已确认工作区清空、物理急停可立即使用、末端/安装件沿用 11 月 8 日配置；并明确授权 mode 0 仅反馈采集 12 秒。共建立 3 次诊断 SDK 会话，没有上电/使能、Servo 使能或运动调用；每次均结束，EDG 已关闭且 Servo=false。
- 实机地址 192.168.110.224；本机有线地址 192.168.110.91。旧默认 EDG 接收地址 .96 不在本机，保存本站覆盖配置 site_ubuntu.yaml，未更改网卡或默认硬件选择。
- TCP1=[0,0,200,0,0,0]，负载 2.2872468660570635 kg 及质心逐值匹配。当前关节的本地模型与厂家 FK 一致。mode 1/0 各采集约 12 秒，接受的新帧最大主机间隔分别 20.275/18.263 ms；安全状态最大观察年龄分别 37.967/41.838 ms。观测最大值不是绝对延迟保证。
- SDK 头的 C++ 条件宏导致旧插件存在未解析的 C++ 符号；新增项目 C linkage 包装，编译启用 --no-undefined，验证安装插件的 ldd -r 与 RTLD_NOW。没有修改厂家文件。
- 修补可能已进入伺服但回复失败时的清理、停止失败重试/保留句柄、生命周期错误传播和有限值/程序/队列/拖动检查；激活前增加 0.25 秒连续静止反馈。SDK 在途 RPC 仍不能可靠取消，不能声称软件停止具有确定时限。
- 原生 Guard 现在实际检查 jerk，65 个断言通过；20 项 SDK 替身生命周期测试与 17 项离线/65 姿态 FK、14 项规划回调测试通过。最终编译与 ROS fake 功能回归通过，退出阶段的 MoveGroup 析构崩溃仍未解决。
- 关键阻塞：JTC output 诊断回放在 OMPL 起步第 387 帧触发 jerk 保护；取消制动也有显著加速度/jerk 跳变。固定 8 ms 回放同样拒绝；尚无硬件 write(period) 的精确证据，不能宣称硬件 Guard 与命令链兼容。未放宽保护阈值，也未通过实机运动验证。
- RUN_HARDWARE.sh 新增 hardware_readiness.json 检查；当前 teleoperation_ready=false 会在加载 ROS/SDK 前拒绝普通实机启动。下一步应定位并修复规划起步与取消输出，再做精确周期验证及分阶段物理停止验收，不能把单元测试或普通 fake 运动通过当成实机通过。
- Quest 地址 https://192.168.110.91:8443；用户确认透视、右手 Touch 唤醒，ROS 已观察到 RELEASED。10 秒观测有 2381 条 RELEASED 与 68 条 INPUT_STALE 状态；手柄方向、握持跟随和摇杆停止尚未验收。网页刷新会重置序号，需重启 fake 输入桥再接入；当前保持原会话。
- 证据：artifacts/hardware_readiness.json、hardware_readonly_validation.json、edg_mode0_validation.json、hardware_lifecycle_stub_20261112.json、controller_guard_replay_analysis.json、quest_live_input_check.json、ubuntu_execution_status.json。
- 测试 UDP 端口已隔离；Quest 实时输入不会再占用 smoke 的合成输入源。所有修改及产物仍在原 D 盘；home 副本、V2_servo、QuestPoseBridge 未修改。

## 历史记录：首次实机 Servo 保持检查失败（2026-11-12）

- 本次为第 4 次 SDK 会话，前三次诊断会话的“无运动 API/无 Servo 使能”仅适用于当时。本次硬件激活成功，使能 Servo 1 次（累计至少 1 次），驱动实际调用了保持启动关节目标的 motion API；未记录调用次数，不能按反馈帧数或运行时间推算。
- 请求 12 秒保持观察，采到 1339 帧有效反馈、约 10.7 秒；从硬件激活成功至故障日志约 11.6 秒。运行期报告 `Latched stop: EDG source stopped advancing`，触发原有 24 ms 源新鲜度保护，提前结束，因此 `passed=false`。本次未放宽该时效门槛。
- 反馈相对首帧最大漂移为 `8.726646260015514e-5 rad`，约 0.005°。这是测得的反馈差，不能声称真实机械臂绝对未动，也不能仅凭报告速度为零判定没有物理运动。
- 检查没有创建运动许可发布器、发送 ROS 轨迹或 action goal；这些为零不代表没有 motion API。保持目标本身就是实际伺服调用。本次未接入 Quest 运动，未许可 Quest 实机遥操作。
- 故障后日志确认 `Servo exit confirmed`，报告 `servo_exit_confirmed=true`，ROS 启动进程退出码为 0。硬件读取错误仍造成控制器撤销等清理错误，报告 `fully_clean_passed=false`；退出伺服确认不等于物理停止时间/距离验收。
- 证据：`artifacts/hardware_hold_first_20261112/hardware_hold_check.json`、同目录日志及反馈 JSONL。原顶层同名文件已由后续检查更新，首次失败应以此固定目录为准。本次硬件检查未改普通遥操作放行状态，`normal_hardware_gate=false`；`physical_stopping_accepted=false`、`teleoperation_accepted=false`、`physical_acceptance=false`。

## 历史记录：EDG 修复及第二次实机 Servo 保持检查通过（2026-11-12）

- EDG 读取顺序已改为先 `get_stat` 更新/读取 SDK 解码缓存，再用 `details` 读取同一缓存对应的源时间戳。源时间戳重复不会刷新反馈新鲜度，仍保留原有 24 ms 保护；没有用放宽时效门槛取得通过。
- 修复后的完整软件验证版本 SHA256 为 `2bd5f1d635308ee422d9def0344ea481145715f4f8ef9a52a3897a7d68aea9e7`。`artifacts/edg_cache_fix_verify_ros.log` 记录完整 `verify_ros.sh` 通过，其中原生 EDG 292 项断言、抽取实际驱动方法并替换 SDK 函数的 28 项生命周期测试通过。fake 测试未连接机器人，不能算作实机验收。
- 本次为第 5 次 SDK 会话、第 2 次实机保持检查；累计为 3 次诊断加 2 次保持，Servo 使能累计 2 次。保持目标的 motion API 已实际调用，但准确发送次数未计数，不按 EDG 调用数、反馈帧数或时长估算。Quest 运动未接入实机，未许可 Quest 实机遥操作。
- 第二次保持观察 `passed=true`：实际观察 `12.001757 s`，观察窗口内 1500 个反馈样本，相对首帧最大漂移 `8.726646260015514e-5 rad`（0.005°）。报告反馈总数另含窗口外 1 帧，不能将其写成 1501 个观察样本；小漂移也不证明真实位置绝对未动。
- EDG 审计累计 6169 次读取，1587 个新帧、4582 次重复、0 次拒绝。仅保留末尾 512 条审计记录，其中可观察的新帧最大主机间隔为 8.284 ms；这只是保留尾段的观测值，不是全程最大值或最坏延迟保证。
- 退出时 `servo_exit_confirmed=true`，ROS 启动进程退出码 0。停止事务本次观测为等待在途 RPC 0.044 ms、abort 105.509 ms、disable 5.754 ms、query 0.463 ms；这是一次 SDK 事务耗时，不是物理停止上界。清理仍记录统计线程 invalid-context 错误，`fully_clean_passed=false`，不能描述为全程无错误。
- 本次证据固定保存在 `artifacts/hardware_hold_fixed_20261112/`；首次失败固定保存在 `artifacts/hardware_hold_first_20261112/`。第二次保持通过未改变 `normal_hardware_gate=false`、`physical_stopping_accepted=false`、`teleoperation_accepted=false`、`physical_acceptance=false`。
- RViz 原配置没有鼠标交互工具，现已补上 MoveCamera 等 Tools、基于实时 TF 的 LiveRobot 和 TCP 坐标轴。场景模型话题曾在 3 秒观察内无更新，因此静态场景模型不作为实时跟随证明。用户先前已确认 USB 下的旧方案可见跟随及松握停止；当前新跟随方案仍待现场体验。

## 当前接续结论（USB 接入待验证）

轨迹与 EDG 修复后的软件回归、第二次 12 秒实机保持检查已通过，首次保持失败仍保留。累计仍为 5 次 SDK 会话、2 次 Servo 使能，保持 motion API 的准确次数未知；本轮距离、合成 5 cm 和 USB 排查未增加真实连接。RViz 视角操作已确认，live Quest 可见跟随仍未验收；当前等待进入 USB localhost 新页后复测，首次 USB 探针 0 帧不代表稳定。普通硬件遥操作放行仍为 false，Quest 实机运动未接入，物理停止和现场映射仍待验证。

