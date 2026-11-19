# V3.1.0 · JAKA S5 遥操作上位机

在 Ubuntu 24.04 x86_64 上，从本目录运行 `bash INSTALL_UBUNTU.sh`，随后运行 `bash START_OPERATOR.sh`。工作台使用随版提供的机械臂和灵巧手模型。下面带日期的路径和测试记录来自原工作站。[首次安装、Quest 连接与备份恢复](docs/V3.1.0.md)

## 当前工程位置（2026-11-14 用户更正）

Ubuntu 当前工程是 `/home/gq/projects_lgq/V3_moveit`。后续修改、启动、构建、缓存和日志均使用此目录。D 盘目录保留为历史来源；下文带日期的旧记录描述当时事实，不是当前路径指令。

此前 Ubuntu 改进实际落在 D 盘。此次先备份 home 原文件，再复制已完成的源码、设备配置、相机依赖和原始报告；构建和显示组件从当前目录重新生成。原文件备份及迁入清单位于 `outputs/directory_recovery/20261114T081311Z/`。历史报告保留原内容和原路径，不能把复制视为重新验收。当前恢复进度与实际验证结果见 `artifacts/directory_recovery_current.json`。

这是独立的 Ubuntu 源码工程。Windows 的 `V2_servo`、现有启动器、Unity Player 和历史数据均保留。此 V3 指新的控制架构，与旧 Windows 复现包的 V3.0.0 包版本不同。

## 当前模型、工具与负载（2026-11-16）

当前模型使用 `JAKAS5-Hand-model-SW/（11.16）手臂.SLDASM`。上位机和 VR 共用更新后的显示模型，末端附件相对 11.13 装配整体旋转约 90°。遥操作页可选择当前 TCP（默认 TCP1）和“已校准，刷新负载”。控制、显示和启动检查使用所选 TCP 定义，负载回读保留完整精度。11 月 18 日在 2 号柜核对的地址为 `192.168.110.224`，Ubuntu 有线网卡的 EDG 地址为 `192.168.110.67`。当前站点配置在 `site_ubuntu.yaml`。

离线、编译、上位机回归及完整 ROS 仿真通过。随后修复了负载校准导致普通仿真被旧报告拦截的问题，实际运行普通仿真入口确认跟随及松握停止。JAKA 设备卡新增主动连接和自动状态更新，电源、使能直接显示在按钮上；TCP / 负载信息改为按钮下的弹出框。两次实机只读连接确认当前 TCP1、2.2524310740921463 kg 负载及质心与 V3 一致，没有发送运动指令或完成新的实机验收。操作方法、证据路径及模型复用范围见 [Quest 工作台说明](docs/quest_workbench.md)。下方按日期保留的历史记录不代表当前模型或当前验收状态。

## DH116 安装方向更新（历史记录：2026-11-13，11.13 装配）

已按用户提供的 `（11.13）手臂.SLDASM` 更新安装方向。原文件为 `/home/gq/projects_lgq/JAKA-S5_开发资料_2026-09-07/整合模型/（11.13）手臂.SLDASM`，只读副本与更新前模型保存在 `reference/hand_mount_20261113/`。原生装配 XML 经过 CRC 校验；旧 XML 的 66 个组件变换与此前 SolidWorks COM 导出一致。新旧装配只有 DH116 位姿变化，相对腕部旋转约 180°。手掌安装 yaw 从约 +45.083° 改为 −134.917°。

`src/v3_moveit/description/s5.urdf` 的 17 个手部显示几何和 6 个碰撞几何已同步更新；机械臂运动学、TCP、快换 A/B、连接件、网格及碰撞包络尺寸保持不变。厂家 A1 活动手指模型与此前 A4/A1 表面配准继续使用，本次不是实物标定或手指驱动验收。`START_OPERATOR.sh` 每次启动都从当前模型重建显示 URDF，避免复用旧方向缓存。来源、矩阵及变更列表见 `src/v3_moveit/config/hand_mount.json`。

编译、21 项离线检查和整套 ROS fake 回归通过，实际 Qt 画布渲染通过。首次 ROS smoke 曾触发“Servo generation/scene validation exceeded 6 ms”；同配置独立复查和整套回归随后通过，未放宽时序、速度或碰撞阈值，首次原因未完全确定。失败原始记录在 `artifacts/hand_mount_ros_first_failure_20261113/`；最终退出码在 `artifacts/hand_mount_checks_20261113.json`。仍有此前已知的组件退出清理告警，不称为全部组件干净退出。

更新前的 Quest 仿真和真机报告已记录通过，仍保留其历史事实；它们对应旧模型。当前模型的 Quest 仿真、实机反馈/负载/质心检查、保持与真机停止验收需按上位机流程更新。此次模型更新没有连接机械臂或发送运动指令。当前软件状态与交付记录见 `artifacts/hand_mount_delivery_20261113.json`。

## Ubuntu 上位机：V3 Operator（2026-11-13）

运行 `bash START_OPERATOR.sh`，或在 Ubuntu 应用菜单搜索 **V3 Operator**。使用 `bash START_OPERATOR.sh --monitor` 可同时启动所选的相机/反馈模块；设备连接页也能逐个启动相机。界面默认仿真；打开窗口和连接监看设备都不会启动机械臂。

- **遥操作**：上位机内部的 3D 画布显示 S5、快换件和 DH116；三路相机（腕部深度、左目 RGB、C922 全局 RGB）在画布内部左侧竖排，只显示三个 16:9 画面，名称、在线点及必要标注叠在图像内，没有外层大底板。点击画面可在场景内放大查看，按 Esc 或关闭按钮返回，右侧停止入口保持可见。左键旋转、中键平移、滚轮缩放，可复位视角。没有独立 RViz 显示窗口。机械臂读取当前会话反馈；无反馈时明确显示模型预览。DH116 六轴使用厂家角度反馈，联动关节按厂家 URDF 映射；无手指反馈时不能将模型姿态视为实物状态。
- **示波器**：黑底网格，独立选择 J1–J6，切换位置/速度、5/10/30 秒窗口、暂停查看；显示采样仅可选择 1/5/10/20/30/60 Hz，不改变控制周期。实际显示率受反馈源限制；过期或无效反馈不会继续画成当前信号。
- **设备连接**：选择 JAKA S5、Quest 3S、DH116、ZED-M、Logitech C922，保存为三套独立方案。工作正常为绿色，异常/缺流为红色，未选用或尚未启动为灰色。相机可逐个启动/停止，也可批量启停所选反馈模块；各模块有独立状态。支持本地 USB 相机或 ROS 图像来源，以及独立的厂家手指反馈 ROS 域/话题；禁止把传感器订阅配置到控制/显示专用域 89、90、91。
- **明暗切换**：顶部苹果式小开关点击切换浅色 / 深色并保存，保留过渡动画；会话控制同步主题。
- **灵巧手推动与触觉**　在抓握页连接后，已归零的手同步当前位置并进入手动控制。模型拖动和滑块以最高 30 Hz 发送目标，松开后提交最终位置。工具栏的「连接」「强制归零」「全部定位回零」排在一起。强制归零重找六轴零位，全部定位回零按现有零位张开手指。[操作说明与精度边界](docs/operator_hand_visuals.md)
- **开发者名单**：右上角蓝色「i」打开苹果风格的片尾字幕页面，14 个章节、88 个研发岗位统一署名刘国庆；支持自动滚动、暂停、重播及深浅主题。
- **会话控制**：Quest USB 重连、本次实时权限、仿真/真机选择及常驻启停。系统认证只为当前界面进程及其子进程授予实时优先级，关闭后失效。停止控制会话只发送 SIGINT，等待控制器退出；关闭窗口会等待控制会话安全退出。

Quest 连接 USB 后打开 `https://localhost:8443`，进入透视模式。在上位机启动所选模式的遥操作。头显显示「可以开始遥操作」时，握住右侧 Grip 控制 TCP1，松开暂停。按下右摇杆可软件停止。换用其他工具时，头显显示相应的工具名称。

真机模式需先勾选「现场准备已完成」。如需实时权限，系统认证后继续本次启动。驱动会核对工具、TCP、负载、质心、模型与实时反馈。「检查与验收」供开发调试时单独运行 12 秒保持和 60 秒跟随、停止检查。

验收页会将仿真跟随、真机跟随和两种停止的结果分别归档。日常遥操作读取当前站点配置和控制柜反馈。

界面以 [Apple 布局原则](https://developer.apple.com/design/human-interface-guidelines/layout)和[材质原则](https://developer.apple.com/design/human-interface-guidelines/materials)作为参考，采用内容优先、稳定层级、直接操作、渐进展开与语义状态色。Qt/Linux 的实现没有宣称使用 macOS 原生 Liquid Glass。3D 渲染宽高使用同一个缩放因子限制像素预算，新尺寸帧到达前保持旧帧比例；鼠标坐标使用同一显示矩形反算。页面随窗口重新布局，图像不被横向拉伸。启用高 DPI 图像和分数缩放支持。

### DH116 持续连接与动作编排

在“灵巧手抓握”选择“DH116 实物”，点击“连接”。工作台使用同一个 EtherCAT 会话读取六轴和五指触觉。已归零的手会同步当前位置并进入手动控制，拖动左侧任一可见指节或右侧滑块均可调整姿态。去使能保留触觉，点击“断开连接”或结束配套的真机遥操作会释放总线。设备报警、失去总线反馈时，请按页面显示的具体原因处理。

「连接」右侧并排放着两个回零按钮。「强制归零」重新寻找六轴零位，点击后需要确认。「全部定位回零」按已有零位把六轴移到 0。两枚按钮在「关节控制」和「抓握方案」中都能看到。

右手扳机先松开，再扣动。扳机行程决定抓握动作的位置，按到底到达各轴设定终点，完全松开时六轴按张开速度回到 0。手动调整后，静止在零位的扳机会保留手动姿态，下一次扣动才接管。关节配置须允许展开到 0 才能使用扳机。

在“抓握方案”选择一根手指，拖动轨道两端或输入开始、结束时间，设置握合位置和该指法向力阈值。拇指的侧摆和弯曲各有一个终点。快速扣到底时，播放位置按照设定时长经过各轨道；完全松开立即请求张开。单指接触阈值限制该指继续闭合，整手保护上限会停止运动并去使能。可先播放或拖动标尺预览，再点“保存并应用”。握合与张开速度独立设置，100% 对应厂家 SDK 的 20000 当量/秒，旧方案的 2000 当量/秒显示为 10%。实际速度还受行程、接触和电流限制。

时间轨和选中手指的参数编辑参考 Godot 动画编辑器，分栏和数值控件参考 Apple 的检查器设计。[Godot 动画编辑器](https://docs.godotengine.org/en/stable/tutorials/animation/introduction.html) · [Apple 面板与检查器](https://developer.apple.com/design/human-interface-guidelines/panels)

### 相机与灵巧手接入状态

ZED 使用厂家 SDK 5.5.1 和 Python API。依赖、NEURAL_LIGHT 模型和独立的 `.venv-zed` 位于本工程，依赖版本见 `tools/requirements_zed.lock`。11 月 18 日用 ZED-M 单独采集左目 RGB 和深度，两路实测均为 1920 × 1080，16 次采样的 SDK 帧率中位数为 29.85 fps。两路数据和有效深度占比见 `artifacts/zed_collection_20261119T000347Z/result.json`。早期 1280 × 720 测试记录在 `artifacts/operator_zed_verified.json`。C922、ZED 左目和深度曾同时显示在 3D 视图左侧，预览图像最大宽度为 1280 像素。[厂家 SDK 与 Python API](https://github.com/stereolabs/zed-python-api)

设备连接页的 ZED-M 与 C922 卡片可单独启动或停止，也可批量启动所选反馈模块。ZED-M 默认选择数据采集均衡模式 1920×1080@30；还提供 1280×720@30/60、最高分辨率 2208×1242@15 和最高帧率 672×376@100。C922 提供 1280×720@30/60、最高分辨率 1920×1080@30 和低带宽 640×480@30。所选模式会立即保存在本机 `.config/operator_profiles.json`，下次打开仍沿用；监看运行中更改会重启对应相机。ROS 图像话题模式由发布端控制，因此这些本机采集选项会置灰。遥操作 3D 相机画面显示实际图像尺寸和采集帧率，预览更新最高 20 Hz；缓存只覆盖当前 JPEG 画面，不按帧累积文件。

相机曾出现 USB `-71` 通信错误；安装 ZED-M 专用访问规则并由用户完全重新插拔后恢复。规则入口 `bash tools/setup_zed_usb.sh` 只针对 `2b03:f681/f682`，不更改机器人网络。以后连接失败可先检查 USB 3.x 线缆/端口，在设备页停止并重新连接监看；程序也会重试已经退出的 USB 采集线程。

DH116 厂家资料位于 `/home/gq/projects_lgq/灵巧手_DH116`。ROS 接入可订阅 `/hand_control/realtime_state`（默认域 0，可配置）。11 月 18 日通过现场网卡 `enp11s0` 连接 DH116，在同一 EtherCAT 会话中持续读取六轴和五指触觉。60 秒的只读结果在 `artifacts/dh116_workspace_readonly_20261118T235352Z/result.json`。抓握页的使用方法见上方“DH116 持续连接与动作编排”。

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

## 切换 Ubuntu 后从这里开始

1. 在 Ubuntu 文件管理器打开 `/home/gq/projects_lgq/V3_moveit`，右键在终端中打开。
2. 安装系统依赖并编译：

   ```bash
   bash INSTALL_UBUNTU.sh
   ```

   脚本核对 Ubuntu 24.04 / x86_64，只安装缺失的系统依赖（已齐全时无需 sudo），检查 Linux SDK 依赖并编译本工程。不会升级整个操作系统、重启、操作机器人或修改 V2。工程、临时文件、编译产物和日志在本目录；ROS 系统包安装在 Ubuntu 的系统位置。

3. 运行自动集成验证，整个过程使用模拟硬件：

   ```bash
   bash tools/verify_ros.sh
   ```

   它实际启动 Servo、MoveIt、场景和模拟控制器，检查小幅跟随、输入断流停止和注入障碍后的碰撞响应。失败日志在 `artifacts/ros_smoke.log`。`artifacts/ros_smoke_pass.json` 会记录本次通过或失败状态，必须核对其中的 `passed`、源码摘要和最终 write 审计，不能仅凭文件存在判定通过。

4. 打开可视仿真：

   ```bash
   bash RUN_SIM.sh
   ```

   另开一个终端进入同一目录，可以先用自动生成的小幅手柄轨迹查看运动：

   ```bash
   bash TRY_SYNTHETIC_INPUT.sh
   ```

   此输入程序会先查询运行模式，拒绝在 hardware 模式发送模拟输入。测试后重新启动仿真，再连接真实 Quest，避免混用两个输入来源。

## 在 Ubuntu 接 Quest 手柄

这条路线不依赖 Windows Quest Link。电脑运行控制和 RViz；Quest 浏览器进入透视模式，发送右手柄位姿和按键。

1. Quest 与电脑接入彼此可达的局域网。在 Ubuntu 网络设置中查看电脑连接 Quest 所在网络的 IPv4 地址。它可能与机器人专用网卡的地址不同。
2. 在工程目录运行 `bash START_QUEST_WEB.sh 电脑的IPv4地址`，把最后一项换成实际数字地址。
3. 在 Quest 的 Meta 浏览器打开程序打印的 HTTPS 地址。确认是自己的电脑后接受这次本地证书，点击进入透视模式。
4. 在上位机启动遥操作。看到头显显示「可以开始遥操作」，握住右侧 Grip 移动 TCP1，松开暂停。按右摇杆可软件停止。
5. 首轮在仿真核对前后、左右、上下和旋转方向。`runtime.yaml` 中的坐标映射沿用旧定义，但 WebXR 输入和现场朝向必须实际核对。

### USB 重连（已验证输入时序）

已发现并授权 USB Quest，HTTPS 服务使用 localhost 证书。每次拔插后可重建转发：

```bash
cd /home/gq/projects_lgq/V3_moveit
adb devices
adb reverse tcp:8443 tcp:8443
adb reverse --list
```

确认设备状态为 `device`。连接多台设备时，给 adb 命令添加 `-s 设备序列号`。在 **Quest 浏览器** 打开 `https://localhost:8443`，首次连接时核对并接受本机证书，然后点击「进入透视模式」。

当前 HTTPS 服务进程记录在 `artifacts/quest_usb_server_current.json`；本轮有界输入观察使用 domain 90、`auto_replan=false`，启动/停止由观察工具管理。重连时需核对进程存活，勿重复占用 8443。若 HTTPS 服务已退出，可在另一终端复用现有证书启动：

```bash
/usr/bin/python3 -B tools/operator_xr/server.py \
  --cert .cache/quest-tls/localhost-usb-20261112.crt \
  --key .cache/quest-tls/localhost-usb-20261112.key
```

该服务同时提供三维工作台和遥操作状态卡，右手柄输入通过本机 UDP 5006 和 5008 分别转发给机械臂和灵巧手。USB 和局域网入口显示相同的提示。

右手柄侧面的 Grip 控制 TCP 跟随，前侧扳机控制已连接 DH116 的抓握行程。

## 实机入口

命令行也可启动当前站点的 Servo 遥操作。确认现场准备并启用当前进程的实时权限后运行以下命令。

```bash
bash RUN_HARDWARE.sh --hardware
```

该入口会连接 S5 并进入伺服模式，不能当作只读检查。它不会自动上电、使能、清报警或修改 TCP/负载。驱动在启动时检查当前关节、静止状态、TCP1、负载、碰撞保护、厂家 FK 与安装模型的一致性。首次控制许可到达前只保持启动时的关节位置；计算、输入和硬件通信均正常后才接管。

2 号 S5 的地址是 `192.168.110.224`。本站点的 EDG 地址见 `site_ubuntu.yaml`，启动时核对到控制柜的有线路由和实际源地址。

可以复制 `src/v3_moveit/config/runtime.yaml` 为本地站点配置，修改实际网络地址或场景，再运行：

```bash
bash RUN_SIM.sh runtime_config:=/完整路径/site.yaml
bash RUN_HARDWARE.sh --hardware runtime_config:=/完整路径/site.yaml
```

运行配置允许选择 `s5_only` 清空台面或 `workcell` 原实验箱。关节动态限制固定在本包的 `joint_limits.yaml` 和 C++ 保护器中；只改运行配置中的同名数字不会提高驱动限制。要调整动态必须同步修改并重新验证，不能在现场凭手感放大。

## 控制分工

```text
Quest WebXR / 兼容的原 UDP 位姿输入
                ↓
输入校验、松握接管、相对位姿映射 → Supervisor 状态协调
                ↓
FixedServo：上游 MoveIt Servo IK、碰撞/奇异位形处理
           + 原生 Ruckig，固定 8 ms q/v/a 和相位切换点
                ↓
单一轨迹入口 → SmoothTrajectoryController（复用上游 JTC）
                ↓
C++ JAKA 插件：最终命令 Guard，固定 8 ms 目标与独立反馈/时效检查
                ↓
厂家 Linux SDK → S5 控制柜 → 电机
```

`FixedServoNode` 继续使用上游 Servo 的逆解、碰撞/奇异位形缩放及关节边界处理。原生 Ruckig 从上一个已生成命令的 q/v/a 连续生成下一段；不再采用上游滑动窗口对速度的割线重写，也不把墙钟抖动当作曲线采样间隔。正常跟随与 JTC 取消共用固定 8 ms 命令时钟，保留原生 jerk 相位切换点和真实 q/v/a。32 ms 未来命令窗口是明确的输入延迟，不能忽略它来估计停止距离。实际线程周期、主机发送截止期和反馈新鲜度仍独立检查。

自动绕障先停稳，再等待目标稳定 0.25 秒，通过 `GetMotionPlan` 服务请求 OMPL；只允许一个未完成请求，迟到结果须通过目标代际、场景和静止起点校验。`PathRuckigRetiming` 保留 OMPL 几何路径，仅去除数值重复点和严格共线中间点，在保留的转角处停到零速度/零加速度；输出实际 Ruckig 曲线，而非只改 waypoint 时间。输出仍交给 MoveIt `ValidateSolution`。0.3 秒是规划器预算，不是服务端到端硬超时；当前只规划模式停用了有缺陷且未使用的 MoveGroup action 入口。

`SmoothTrajectoryController` 复用上游 JTC 正常插值，取消时由同一控制更新循环运行共享的 `JointStopProfile`。它发送终点零速度/零加速度命令后才确认 CANCELED，制动期间不接受新轨迹覆盖。Supervisor 等待动作终态及命令、反馈静止窗口，再从精确命令位置恢复 Servo，不以带噪声的测量位置重置正在运动的曲线。

停止轨迹可能离开原 OMPL 折线，因此规划适配器为每个 8 ms 可取消起点预检同一停止算法的位置范围、碰撞和路径约束；FixedServo 在提交新段前检查实际段及停止包络。检查有计算预算，失败或超时不提交对应运动。这些是离散场景检查，不是连续碰撞证明；环境变化也不能沿用旧场景的停止验收。先停稳再绕障仍会有停顿，并未实现连续改变全局目标时的无缝绕行。

## 代码位置

| 文件 | 职责 |
|---|---|
| `src/v3_moveit/launch/bringup.launch.py` | 组装仿真或实机完整链路 |
| `src/v3_moveit/v3_moveit/input.py` | 输入时效、源序号、离合、坐标映射 |
| `src/v3_moveit/v3_moveit/quest_bridge.py` | Quest 输入转 ROS 末端目标 |
| `src/v3_moveit/v3_moveit/supervisor.py` | 跟随/制动/规划/执行协调，拒绝过期计划 |
| `src/v3_moveit/v3_moveit/scene_loader.py` | 载入并读回场景障碍 |
| `src/v3_moveit/config/servo.yaml` | 上游 Servo 参数及固定采样适配要求 |
| `src/v3_moveit/cpp/fixed_servo_node.cpp` | Servo C++ API、固定时间网格、原生 Ruckig 和停止包络预检 |
| `src/v3_moveit/include/v3_moveit/servo_interval.hpp` | 提取原生曲线的 jerk 相位切换点与 q/v/a |
| `src/v3_moveit/cpp/path_ruckig_retiming.cpp` | OMPL 路径保持重定时与取消包络预检 |
| `src/v3_moveit/cpp/smooth_trajectory_controller.cpp` | JTC 正常采样、实际制动、终态与恢复衔接 |
| `src/v3_moveit/include/v3_moveit/joint_stop_profile.hpp` | 规划预检与运行时共用的停止曲线 |
| `src/v3_moveit/cpp/jaka_system.cpp` | 唯一实机 SDK 所有者、反馈与固定周期发送 |
| `vendor/jaka` | 从原安装复制的 V2.3.1_beta3 Linux x86_64 SDK |
| `src/v3_moveit/description` | S5 控制模型、末端和碰撞网格 |
| `quest` | 头显浏览器输入页和只转发输入的服务器 |

关节控制模型由原工程通过厂家 FK 识别的 PoE 模型转换，65 姿态的数值一致性验证在离线脚本中。碰撞网格从旧场景复制，CAD 到 SDK 的名义坐标配准残差记录在 `model_metadata.json`；这不是新做的实物标定。

## 验证范围与待办

以下为 EDG 修复后版本 `2bd5f1d635308ee422d9def0344ea481145715f4f8ef9a52a3897a7d68aea9e7` 的软件证据，完整日志为 `artifacts/edg_cache_fix_verify_ros.log`，`physical_acceptance=false`。此前轨迹修复基线 `ee81ce3313e0528b49ba2e85d1425774cebc3e3537f1e2a29e6d2282625225ae` 的日志仍保留于 `artifacts/trajectory_final_verify_ros_2.log`：

| 检查 | 本次结果与边界 |
|---|---|
| 离线与模型 | 17 项检查、65 姿态 FK；原生 Guard 包括非均匀周期差商、起步、reset 和拒绝后历史不变。未调用 SDK |
| Supervisor 回归 | 63 项通过，覆盖过期规划、取消竞态、场景失效、精确静止起点和恢复 |
| 路径与停止包络 | 4432 项路径断言、858 项停止包络断言；使用安装的 JTC 插值及原生 MoveIt 场景检查，无 ROS 节点/SDK |
| Servo 原生曲线 | 985 个对齐网格采样、3943 次最终 Guard 检查通过；连续任意时点插值精度有限，见下文 |
| EDG 与 SDK 替身 | 原生 EDG 292 项断言、实际驱动方法配 fake SDK 的 28 项生命周期测试通过；未连接厂家 SDK 库或机器人 |
| 全套 ROS smoke | `edg_cache_fix_verify_ros.log` 通过；严格后端 1280 次 write，其中 1223 次接受、57 次启动无命令、0 次拒绝；断流停止与注障响应通过 |
| 独立 JTC action | 0.5/1.0 倍测试限值下，正常执行、取消实际停止、制动中拒绝新目标、取消后重启通过；819 次 write、0 次拒绝 |
| 独立纯 Servo | `auto_replan=false`：连续 FOLLOW 前进/反向、松 Grip、无跳变重新夹持、再次运动和 UDP 断流停止通过；1098 次 write，1062 次接受、0 次拒绝 |
| 独立自动绕障 | 默认 `auto_replan=true`：真实 OMPL 运动、CANCELED/RESUMING、精确静止保持及恢复后独立 FOLLOW 运动通过；752 次 write，678 次接受、0 次拒绝 |

独立用例在 `tools/fixed_servo_ros_test.py`，仅能启动 fake，使用独立 ROS domain、UDP 端口和轻量发送子进程。默认场景未移除；发送进程的两次观测最大活动间隔分别为 8.727/8.790 ms。完整输入、控制器输出和最终 write 审计保存在 `artifacts/fixed_servo_ros_test/`。这消除了此前记录器 195 ms 停顿误造输入断流的问题，没有改变生产的 80 ms 输入保护。

`artifacts/servo_interval_native/result.json` 明确区分 `fixed_grid_contract_passed=true` 与 `continuous_curve_precision.passed=false`：扩展连续速度跟随测试中的 24 ns 极短相位段的离网格 JTC 五次插值有数值精度限制，本次最大加速度差约 1.92747 rad/s²。不能把固定 8 ms 输出采样通过写成连续曲线任意时刻精度通过，也不能据此推断控制柜内部插补或物理停止行为。

当前证据入口：`artifacts/edg_cache_fix_verify_ros.log`、`artifacts/edg_feedback_native/result.json`、`artifacts/hardware_lifecycle_tests.json`、`artifacts/ros_smoke_pass.json`、`artifacts/smooth_controller_ros_test/result.json`、`artifacts/fixed_servo_ros_test/result.json`、`artifacts/servo_interval_native/result.json`。历史根因与反事实重建保存在 `artifacts/servo_handoff_analysis.json`；旧失败证据不是当前 fake 命令链的结论。

FixedServo 本轮正常退出；MoveGroup 仍在 SIGINT 后析构崩溃，独立用例如实记录 `passed=true`、`fully_clean_passed=false`。其表现与上游 [MoveIt 清理问题及修复提案 #3828](https://github.com/moveit/moveit2/pull/3828) 相符，本机尚未证明完全同根，未修改系统 MoveIt 库。重启前确认旧进程退出。

EDG 缓存读取顺序修复后第二次保持检查通过；实机待办仍包括物理停止时间/距离、柜端执行与中断响应、周期尾延迟和现场 Quest 方向、握持与停止操作。前三次诊断、第一次失败与第二次通过的保持检查均不能替代这些验收；SDK 在途 RPC 尚不能可靠取消，旧慢查询 P-005 不能因改用 C++ 就宣称解决。`hardware_readiness.json` 的遥操作放行状态仍为 false。没有动态障碍感知或连续碰撞保证；相机与线缆仍在旧碰撞模型范围之外。

## 上游依据

- [MoveIt Servo](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- [MoveIt Jazzy Servo 源码](https://github.com/moveit/moveit2/tree/jazzy/moveit_ros/moveit_servo)
- [JAKA 官方 ROS2 仓库](https://github.com/JAKARobotics/jaka_ros2)，本包不使用其整段轨迹服务替代实时 Servo。
- [ros2_control JointTrajectoryController](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
- [Meta Browser 混合现实接口](https://developers.meta.com/vr/documentation/web/webxr-mixed-reality/)
- [W3C WebXR 手柄映射](https://immersive-web.github.io/webxr-gamepads-module/)

本工程用于原设备的本地集成。厂家 SDK 和 CAD 资产保留各自权利；未上传或公开发布。


## 历史记录：Quest 接入与实机检查（轨迹修复前，2026-11-12）

本节保留当时的测试计数与阻塞描述；“第 387 帧拒绝”和“下一步修复轨迹”已由上方当前软件验证更新，实机待办仍保留。以下“当前”均指当时。

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

## 当前接续结论（第二次实机保持检查后）

轨迹修复与 EDG 修复后的完整软件回归通过；首次实机保持失败保留，第二次 12 秒保持检查通过。累计 5 次 SDK 会话、2 次 Servo 使能，实际保持目标 motion API 的准确调用次数未知。普通硬件遥操作放行仍为 false，Quest 实机运动未连接/未许可，物理停止和现场方向映射尚未验收；当前新跟随方案的 RViz 手感仍待用户核对。保持通过、退出 Servo 确认、单次停止 RPC 耗时和小漂移均不能替代这些验收。

