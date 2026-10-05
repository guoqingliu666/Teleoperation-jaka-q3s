# 数字孪生逐步打开、驱动与核验

2026-10-04 最新修复：影子控制与标记统一使用 TCP1=TCP2 局部 +Z 200mm；启动视角回正，中央遮挡面板默认收起。`Home` 回正，`Tab` 详情。[报告、截图与操作步骤](tcp_camera_fix_20261004.md)。真机新工具仍未验收，原冻结 Release 不变。

## 2026-09-29/30 Quest主线更新

当前以 [P0/P1实测报告](quest_servo_p0_p1_report.md) 和 [Quest只读逐点击](quest_readonly_click_by_click.md) 为准。本轮S5 10001连接超时，未读回Tool1/负载，未发真实运动。新V2Twin诊断页已实际打开；OpenXR运行时报告头显不可用，Touch/Touch Plus profile缺失已修复，但头显/按键仍需复测。本文其余历史成功不自动升级为当前通过。


以下操作默认只用仿真；所有路径在 D 盘。准备：安装 Unity Editor **2022.3.62f3**、OpenXR 包，Anaconda `jaka` 环境；本机 Unity 位于 `D:\计算\Unity Hub\unity编辑器\2022.3.62f3\Editor\Unity.exe`。MuJoCo3.3.7 等本机已安装到项目 `digital_twin/.deps`，新机器按第 0 步重装。数字孪生只读接收，**软件停止不是控制柜硬急停**。真人 VR 和实机运动仍未验收。鼠标细节见 [视角操作表](digital_twin_view_controls.md)，几何来源见 [修复记录](digital_twin_geometry_fix.md)。

| 步骤 | 点击/命令 | 预期反馈 | 失败时看哪里与处理 |
|---|---|---|---|
| 0 环境 | 打开 **Anaconda Prompt**，执行 `conda activate jaka`、`cd /d D:\ChatGPT\Projects\VR\V2_servo`、`set TEMP=D:\ChatGPT\Temp`、`set TMP=D:\ChatGPT\Temp`、`set PYTHONDONTWRITEBYTECODE=1`。新电脑再执行 `python -m pip install --target digital_twin\.deps -r digital_twin\requirements.txt`。 | `python --version` 为兼容的 3.12，`digital_twin/.deps` 有 `mujoco`。 | 若 `conda` 不存在，先安装 Miniconda 到 D 盘并建立 `jaka` 环境；不要安装到 C 盘以免违反项目存储规则。 |
| 1 复制来源并重建资源 | 首次先依次执行 `python digital_twin\cad\copy_solidworks.py`、`python digital_twin\cad\copy_s5_player_source.py`，然后执行 `python digital_twin\build_assets.py`、`python digital_twin\cad\geometry_repair_report.py`。每条命令完成后再输下一条。 | 19 个 SolidWorks 副本 SHA 与 F 原件一致；旧 Player 七件 STL + URDF 被复制；输出 `parts:443`，近景工装 646457 面、右旧臂 417389 面；有 `data/build_report.json` 和 `data/geometry_diagnostics.json`。 | 缺来源先看 `digital_twin/cad/derived/*/source_manifest.json`。若 D 副本与 F 原件哈希不同，脚本会拒绝覆盖，先人工排查，不能写 F 原件；若 STEP 两处 BREP 无效仍可视觉预览，不能当作正确碰撞体。 |
| 2 打开 Unity | 最快可双击 `D:\ChatGPT\Projects\VR\V2_servo\digital_twin\unity\Build\V2Twin.exe`。需要编辑时在 Unity Hub 点“打开/从磁盘添加”，选 `D:\ChatGPT\Projects\VR\V2_servo\digital_twin\unity`，编辑器选 **2022.3.62f3**。 | Windows Player 应显示箱内视角；编辑器 Assets 下看到 `V2Twin/Scenes/Workcell.unity`、`Prefabs`、`Generated`，控制台无编译错误。 | Player 启动失败看 `digital_twin/data/unity_desktop_probe.log`；Unity 包解析失败看 `digital_twin/data/unity_build.log` 与 Console，设置 `UPM_CACHE_ROOT=D:\ChatGPT\Cache\UnityUPM` 后重开。无HMD时日志可能有 `XR_ERROR_FORM_FACTOR_UNAVAILABLE`，桌面模式仍可工作。 |
| 3 构建/回读 Unity | 菜单点 `V2 Twin → Build workcell scene`，完成后点 `V2 Twin → Validate saved scene`，最后点 `V2 Twin → Build Windows player`；双击 `Assets/V2Twin/Scenes/Workcell.unity`。 | Console 有 `[V2Twin] BUILD_OK`、`VALIDATE_OK`、`PLAYER_BUILD_OK`；场景层级只有 **Leftmost_JAKA_S5_only_actuated** 具 6 关节。`digital_twin/figures/unity_inside_operator.png` 和 `unity_inside_fixed.png` 可见蓝色 S5；右旧 TX2 是**不透明橙色静态占位**。 | 缺菜单看 Console 编译错误；场景空白先确认打开了 `Workcell.unity`，再按 F 聚焦。若材质仍像旧图，必须重新点 Build，脚本会刷新 Mesh asset；第三臂无可信位置，未造假。 |
| 4 桌面视角逐次点按 | 点 Unity 顶部 **Play**、再点 **Game** 画面；按 F1 看箱内操作、F2 固定、F3 自由起点、F4 Link06 跟随。F1 后按住左键拖动旋转，滚轮缩放，中键或 Shift+左键平移，右键拖动原地转头；WASD/QE 移动，再按 F1 返回。 | 第一次鼠标动作立刻进入自由视角，松手后相机停在新位置，不被预设拉回；F4 跟随 Link06，**非已标定 Tool1 TCP**；能看到蓝 S5、工装及橙色旧型占位。 | 如果无响应先点 Game 页取得焦点；如果画面被工装遮挡按 F1 复位。运行探针只验证相机坐标会变，仍建议照此人工拖动核验；HMD 未实测。 |
| 5 MuJoCo 原生相机 | 在 Prompt 依次运行 `python digital_twin\run_mujoco.py --render-views`、`python digital_twin\run_mujoco.py --steps 1000`；再双击 `launch\MUJOCO_TWIN.cmd` 或运行 `python digital_twin\run_mujoco.py --viewer`。鼠标点 Viewer 画面，左拖旋转、右拖平移、Shift+右拖水平平移、滚轮或中键拖缩放。 | CLI 生成 `figures/mujoco_inside_operator.png`、`inside_fixed.png`、`mujoco_simulated_motion.png`；`stable_bounded=true`；Viewer 起点是**原生 FREE 相机**且拖动后不自动回位。 | 黑色静态图先核对 `run_mujoco.py` 已在 keyframe reset 后执行 `mj_forward`；Viewer 缺 OpenGL 时看命令行错误，用静态图和 XML 加载核验；检查是否意外把相机设为 fixed。 |
| 6 GUI→双孪生 | 先保持 Unity Play；另开 Prompt 运行 `python digital_twin\run_mujoco.py --viewer --shadow-seconds 1`；再双击根目录 `START_V2_GUI.cmd`，在 **SIM** 窗口依次“连接→上电→使能→进入伺服”，按住 Space+1 约 1 秒，再松手。 | Unity 状态叠加显示 `反馈新鲜`、计数上升，蓝色臂跟随；MuJoCo Viewer 跟随；GUI 为仿真，绝不会下发真机。链路为 GUI 快照→有界 UDP5007/5008→双引擎。 | 无同步先看 GUI 是否为 `SIM` 并已连接、状态 q 有值；`反馈过期` 时看 GUI 反馈年龄，检查 5007/5008 本机端口占用；MuJoCo Viewer 只能有一个进程绑定 5008。关闭其它状态接收器后重试。不能通过删除过期判断来“修好”。 |
| 7 录制/回放/截图 | Unity Game 页按 F9 开始记录，GUI jog 后按 F10；断开/停止实时状态源，按 F11 回放最新；按 F12 截图。 | `digital_twin/data/unity_state_*.jsonl`、`digital_twin/data/unity_*.png`；回放状态 `REPLAY_READONLY`。Unity 回放只改模型，不发机器人命令。 | 若提示“实时反馈新鲜”先停止 GUI 或 Unity 再按 F11；若无文件先 F9 后产生有效状态。录屏未自动接入，需用现场录屏软件输出到 `digital_twin/data/` 并记录帧率。 |
| 8 仅演示运动 | Unity Game 页按 F5。 | 状态 `SIM_DEMO_ONLY`，只在没有新鲜反馈时摆动 J1；再按 F5 关闭。 | 若实时包新鲜，演示被实际状态覆盖，先断开 GUI。此模式不用于真机轨迹验收。 |
| 9 VR 尝试 | 连接 Quest/SteamVR，确认 Windows OpenXR runtime 是所用头显；Unity `Project Settings → XR Plug-in Management` 核对 Standalone **OpenXR**；进入 Play，先按 F1，再佩戴头显左右转头、平移身体，确认双眼 3D 视差。回到桌面 Game 页用鼠标左拖转动头显世界锚点、滚轮前后调位置，按 F1 复位。确认 GUI 的 VR 接收器监听 5006 且死手/姿态检查通过后，按 F6 明确开启 QDU Quest pose sender。 | 头显从箱内看左 S5，头动与鼠标调整都能改变视角；F6 后控制台有 `[VrPoseUdpSender] Sending to udp://127.0.0.1:5006`。**无头显时只算桌面模式通过，不能填 VR 通过。** | OpenXR 初始化失败查 Unity Console、头显连接与当前 runtime；`XR_ERROR_FORM_FACTOR_UNAVAILABLE` 通常表示本机无可用 HMD。F6 不等于允许真机伺服；Grip、坐标方向和安全门需独立验证。 |
| 10 ZED | 在 V2 GUI“诊断 / IP / ZED相机”点“扫描/标定核对”，核对 SN12474260、固件1523、eye-in-hand；需要图像时点“打开双目预览”。 | 独立 SDK 进程抓图，报告在 `data/zed_gui_*/report.json`，数字孪生的 `calibration/status.json` 仍说明 Link06→相机无效。 | SDK 失败查 `sdk.log`；中文路径问题用 `data/zed/settings/` 英文路径副本。不可将用户三个 4×4 矩阵未经方向/单位核对直接启用相机位姿或机器人目标。 |
| 11 一致性回归与哈希 | 顺序执行 `python digital_twin\verify_consistency.py`、`python -m unittest discover -s digital_twin\tests -v`、`python digital_twin\verify_gui_shadow.py`、`python digital_twin\verify_gui_unity.py`，最后 `python digital_twin\make_delivery_manifest.py`。 | 100 随机姿态 FK 报告、6 个孪生单测、GUI→MuJoCo/Unity Player 状态包通过；`data/delivery_manifest.json` 记录截图与模型 SHA。 | 若失败先看同路径 JSON 与 stdout；改关节/相机配置后必须重新构建 Unity 场景与 Player。不要同时开多个 `conda run`，该机共用 D TEMP 下的 Conda 临时文件会冲突。 |
| 12 VLA/全域离线 | 执行 `python digital_twin\policy_env.py` 和 `python digital_twin\scan_workspace.py`。 | 250步替身策略烟雾通过，64个限位角点+1000随机全域姿态接触代理统计写入 `digital_twin/data/workspace_scan_result.json`。 | 代理接触多是危险提示；它不含真实夹爪/线缆/门/第三臂。真实 VLA 权重/训练集缺失，不能称原实验复现。 |

真机窗口仍按 [V2 GUI 的逐点击说明](V2_servo_usage_click_by_click.md) 走现场门槛。左底座方向、Link06/Tool1、手眼外参、三臂实际几何、工具夹爪、线缆/门/工装碰撞和硬件停止验收之前，数字孪生**不可**授权全空间真机运动。仿真可大胆覆盖完整配置关节域；现场按“仿真→悬空/低功率→真机小范围→安全空间逐步扩大”，每一步有人员值守与物理急停。软件按钮仅可停止软件控制。


## 最新ACT/DH116增量（2026-09-29）

最新要求明确左侧蓝色S5**带DH116**，覆盖前轮无手要求。旧Player CompleteEndEffector的20件视觉装配已复制到V2派生目录，来源哈希见 `data/act_source_manifest.json`。Unity/MuJoCo/URDF/USD/glTF接入同一来源；MuJoCo保持6个机械臂动态关节，手为运动学视觉、不伪造接触惯量。Unity F5模拟演示含手指，F1–F4及鼠标自由视角保留；MuJoCo原生视角操作不变。

采集入口与阶段A/B步骤见 [ACT采集操作](data_collection_pipeline.md)，本轮证据/未通过项见 [ACT验收](act_collection_acceptance.md)。原QDU权重和真实训练数据仍未复现，本轮新管线的改进是源时间戳/缺失标记/哈希/在线配对审计和独立异步记录，不宣称训练精度或力控效果优于原方案。模型显示、输入协议和合成episode成功不能代替现场碰撞、工具标定、真实VR和DH116力控验收。
