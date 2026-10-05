# V2 数字孪生设计与一致性边界

2026-10-04 最新修复：影子控制与标记统一使用 TCP1=TCP2 局部 +Z 200mm；启动视角回正，中央遮挡面板默认收起。`Home` 回正，`Tab` 详情。[报告、截图与操作步骤](tcp_camera_fix_20261004.md)。真机新工具仍未验收，原冻结 Release 不变。

## 2026-09-29/30 Quest主线更新

当前以 [P0/P1实测报告](quest_servo_p0_p1_report.md) 和 [Quest只读逐点击](quest_readonly_click_by_click.md) 为准。本轮S5 10001连接超时，未读回Tool1/负载，未发真实运动。新V2Twin诊断页已实际打开；OpenXR运行时报告头显不可用，Touch/Touch Plus profile缺失已修复，但头显/按键仍需复测。本文其余历史成功不自动升级为当前通过。


2026-09-29。交付位于 `D:\ChatGPT\Projects\VR\V2_servo\digital_twin`。**当前是可渲染、可关节驱动、可运行 MuJoCo 动力学的实验模型；它不是经现场批准的碰撞安全模型，也不是实机伺服验收。** V2 真机运动放行仍由原有部署证据门槛控制。

## 来源、导入与选择

| 资产 | 来源与实际核对 | 派生结果 |
|---|---|---|
| 实验箱 | `F:\规划\工程\设备\新隔离舱\布局装配.STEP`，SHA256 `40ab599b25f979f5a784765f1a611442e310bdec8c13ebbb72d58f979733fee2`；与 D 盘只读副本哈希一致 | 443 节点、1,490,441 原三角面。`config/import_map.json` 保存逐件原名、包围盒、三角面、类别。重复名 7 个用索引保留别名；原 BREP 14、136 无效，保留网格并标未解析。原始 CAD 未写入。 |
| 最左侧 S5 | `Quest-JAKA-Repro-20260913/①启动VR与JAKA数字孪生.cmd` → `Player_通信修复/QuestPosePreview.exe` → `StreamingAssets/JakaS5/Link_00..06.STL` + `jaka_s5.urdf`，已复制到 `digital_twin/cad/derived/s5_player_source` | **仅一条S5六轴动态链，末节另接DH116完整运动学视觉装配**。旧 Player STL 与此前 V2 STL 的七件 SHA 完全相同；本轮改进渲染法线、材质与周边几何。底座平移来自旧 CAD 基座；绕 Z +90° 仍是未测量预览假设。 |
| 其它机械臂 | STEP 右侧旧 TX2 有几何，不能声称它是现场 JAKA；第三台缺精确 CAD/基座 | 右侧是橙色**不透明静态旧 TX2 占位**，无运动/接触；第三台只有 `Third_arm_location_unknown_NO_GEOMETRY_NO_COLLISION` 标记，位置/形状不编造。二者均**不**构成安全间隙证明。 |
| ZED-M | SN12474260、固件1523、eye-in-hand；现有 SDK5.3 取流证据见 `docs/zed_eye_in_hand.md` | 此前相机曾真实取流，本轮B未检测到；camera→tcp_tool0、mm已由用户确认，但 `Link06→ZED` 仍未注册，值为 `null`；蓝色站点仅表示 Link06 原点，不能作为真实镜头位置。 |

本轮先将 F 盘 19 个原生 SolidWorks 文件逐件复制到 V2 派生目录并核对 SHA；原文件与副本均未修改。`digital_twin/build_assets.py` 从 D 盘缓存 STEP 三角面重建网格、四种交换格式与映射。近景是箱体透明外壳 9,204 面、工装 646,457 面、右旧型占位 417,389 面、S5 七件 64,114 面；远景 LOD1 为工装约190,000/右旧臂约50,000面。Unity 对不透明物体使用双面材质，蓝 S5 焊接视觉 STL 的重复顶点以平滑法线；原 STL 未改。此前的 189,999/50,000 面全局降采样已取消，以免薄支架/仪器消失。资产、逐件原名/解析状态与修复前后截图见 [几何修复记录](digital_twin_geometry_fix.md)。原 SLDASM 仍引用 TX2 且缺件，STEP 不保留 SolidWorks Mate；关节定义独立取旧 Player S5 URDF，未从 STEP 推断。材质声明在 `digital_twin/materials/materials.json`。

## 单位与变换链

共用主配置 `digital_twin/config/twin.json`；CAD/MuJoCo 世界为**右手、米、Z 向上**。STEP 原始坐标为 mm，导出视觉网格时除 1000。Unity 为左手、Y 向上，使用反射换基 `B·[x,y,z]=[x,z,y]`，所以每个轴角在 Unity 中取负；三角形索引反向维持朝外法线。所有关节原点、轴、上下限、初始读回样本和三个固定相机位置由这份 JSON 生成到 `unity/Assets/V2Twin/Data/twin_unity.json`、`mujoco/workcell.xml`、`urdf/workcell.urdf`。

预览链：`CAD世界 → 旧左底座位置(+90° Z预览方向) → Link00 → joint_1..6 → Link06`。真实链的 `Link06→当前 Tool1 TCP`、`Link06→ZED 左目`、现场零偏与底座方向仍未知，见 `digital_twin/calibration/frame_chain.json`。控制器当前 Tool1 的 TCP 读回为 `[-23.703,-60.146,200.963] mm`、RZ45°，负载2.028 kg、质心 `[6.641,-4.455,67.216] mm`；这**没有**证明它相对模型 Link06 的变换。三份用户 ZED 文件虽按用户命名为外参/内参，实际均为 4×4 变换；不能直接当 fx/fy/cx/cy 或手眼矩阵。SDK 的校正后焦距、双目基线及深度样本详见 `docs/zed_eye_in_hand.md`，中文路径采用 D 盘英文副本读取。

## 双引擎职责

Unity 2022.3.62f3 工程在 `digital_twin/unity`，生成的 `Scenes/Workcell.unity`、箱体和 S5 Prefab 已由 Unity 实际构建/回读。Standalone OpenXR loader 与双眼目标已配置，桌面箱内操作/固定/自由/Link06 跟随四预设；鼠标一动即进入自由模式，左拖环绕、中键/Shift+左拖平移、滚轮缩放、右拖转头。头显位置、姿态由 XR `InputDevices` 更新，鼠标调整的是 XR 世界锚点。OpenXR 包可用不等于已做 Quest Link 立体深度、FPS 或追踪实测。箱内两张 Unity 渲染图在 `digital_twin/figures/`。`VrPoseUdpSender.cs` 来源 QDU `QuestPoseBridge`，只改默认端口 5005→5006，SHA 见同目录 provenance；授权未核实，限本地研究。完整按键见 [视角操作](digital_twin_view_controls.md)。

MuJoCo 3.3.7 MJCF 同时提供 6 个铰链、6 个**仅仿真**位置执行器、19 个关节/末端传感器（共21标量通道）、4 个相机、重力与接触代理。动态每 8 ms 目标周期使用 8 个 1 ms 子步；测试用了理想重力/科里奥利补偿，仅为仿真算法输入。`digital_twin/policy_env.py` 提供离线策略 `reset/observe/step`，观测关节、速度、Link06、传感器、可选箱内 RGB，动作是经限速/限位的六关节**仿真目标**；250步替身策略烟雾测试通过，不代表 ACT/SmolVLA 原权重复现。关节质量/惯量**逐值取自已复制的旧 ① Player `jaka_s5.urdf`**，不是实测，禁止用于真实力矩/接触预测。视觉 STL 与碰撞 STL 分开保存，但接触由 MuJoCo 转为凸包；箱壁/底板仅 AABB 盒代理，内部工装、线缆、夹爪、另外两臂没有完整可批准碰撞。摩擦系数也是仿真占位。`glTF`/`USD` 是保存姿态的静态交换预览，关节语义在 URDF/MJCF；不能以 glTF/USD 文件存在断言动态功能已转换。

MuJoCo 交互运行调用原生 `launch_passive` Viewer，启动只设置一次 FREE 相机箱内起点，后续不改写 `viewer.cam` 或鼠标事件。左拖旋转、右拖平移、滚轮/中键拖缩放沿用 MuJoCo 原生操作。静态截图先在 keyframe reset 后执行 `mj_forward`，避免几何/相机尚未更新时误导出黑图。

## 状态与安全隔离

GUI 控制进程保持独立。GUI 进程从已有 40 ms 状态快照以**最大长度 1 的最新槽**异步发本机 UDP：Unity5007、MuJoCo5008。数据包含 q、只读 TCP、报警、状态、反馈年龄；只有已连接且反馈年龄 0–250 ms 才标 `feedback_valid`。Unity 与 MuJoCo 对过期状态拒绝更新，避免把历史关节角冒充同步。Unity F9/F10 录制 JSONL、F11 从最新录制文件只读回放、F12 保存 PNG 到 D 盘；F5 演示摆动被标为 `SIM_DEMO_ONLY`，F6 显式启动 QDU VR 输入发送。VR 输入 5006 进入原 V2 GUI 的安全状态机，孪生本身不发机器人命令。夹爪状态、真实 ZED 相机位姿因接口/外参未获验证，在叠加层标未知；不会伪造联动。

## 一致性与已验证结果

`digital_twin/verify_consistency.py` 固定种子 20260929 取全关节限位范围内 100 组随机 q，MuJoCo Link06 对来源 URDF 的最大位置差 **3.02e-13 m**、旋转矩阵元素差 **9.48e-13**；两固定箱内视点与自由起点坐标差均 0，Unity 基座平移差 0，轴/限位字段差 0。此处只验证配置/数学链，不证明 Unity XR 头显追踪、末端真实 TCP 或现场零偏。结果 `digital_twin/data/consistency_report.json`。

Unity 编辑器构建与保存场景回读 `BUILD_OK`、`VALIDATE_OK`，含 OpenXR loader 和两张实际渲染图；Windows Player 的合成 SIM UDP 运行时测试验证关节更新、JSONL 录制/回放以及自由 `Orbit/Pan/Dolly` 相机坐标确实改变，GUI 仿真自测→Unity Player 也通过，见 `digital_twin/data/unity_runtime_probe.json`、`gui_unity_integration.json`。MuJoCo 原生 Viewer 启动并正常退出；8 秒、1000 目标周期仿真稳定，最大关节位置误差 **0.0220 rad**、最大角速 **0.1824 rad/s**、此单轨迹接触代理计数 0，见 `digital_twin/data/mujoco_sim_result.json`。GUI 仿真自测→本机 UDP→MuJoCo 只读影子收到 **209** 包、拒绝 0。全关节域离线扫描 64 角点+1000 随机姿态，**757/1064** 触发当前不完整代理接触，见 `digital_twin/data/workspace_scan_result.json`；不能据此批准实机空间。旧 V2 64 测试通过，数字孪生 6 测试通过。没有头显帧率记录、现场碰撞批准或真实机械臂运动数据；因此不把“丝滑真机”和“全空间真机测试”列为通过。

原始源哈希、逐件导入、Unity/MuJoCo 构建日志、截图和结果都在 `digital_twin/data`、`digital_twin/figures`。下一阶段应测左 S5 底座朝向、Link06/Tool1、手眼与夹爪、另两台位置/外形、线缆/门与工装碰撞，再逐级做现场保护验证。回滚只需关闭 Unity/MuJoCo 状态接收器与 V2 GUI；孪生链路未更改 JAKA SDK 所有者或旧工程。


## 最新ACT/DH116增量（2026-09-29）

最新要求明确左侧蓝色S5**带DH116**，覆盖前轮无手要求。旧Player CompleteEndEffector的20件视觉装配已复制到V2派生目录，来源哈希见 `data/act_source_manifest.json`。Unity/MuJoCo/URDF/USD/glTF接入同一来源；MuJoCo保持6个机械臂动态关节，手为运动学视觉、不伪造接触惯量。Unity F5模拟演示含手指，F1–F4及鼠标自由视角保留；MuJoCo原生视角操作不变。

采集入口与阶段A/B步骤见 [ACT采集操作](data_collection_pipeline.md)，本轮证据/未通过项见 [ACT验收](act_collection_acceptance.md)。原QDU权重和真实训练数据仍未复现，本轮新管线的改进是源时间戳/缺失标记/哈希/在线配对审计和独立异步记录，不宣称训练精度或力控效果优于原方案。模型显示、输入协议和合成episode成功不能代替现场碰撞、工具标定、真实VR和DH116力控验收。
