# 数字孪生验收记录（2026-09-29）

2026-10-04 最新修复：影子控制与标记统一使用 TCP1=TCP2 局部 +Z 200mm；启动视角回正，中央遮挡面板默认收起。`Home` 回正，`Tab` 详情。[报告、截图与操作步骤](tcp_camera_fix_20261004.md)。真机新工具仍未验收，原冻结 Release 不变。

## 2026-10-02 23:34：三轴阶段提示实渲染通过

原V2Twin/左S5＋DH116/场景未重建。`data/quest_guidance_player_20261002_233442_425156/`实际Player应用X/Y/Z演练、H1、准备、正式XYZ、锁停、结束8阶段，并通过过期提示测试；`probe.LIVE.png`已查看，XR active读回true。测试零SDK，不能代替真机或头显端到端显示延迟。新增H6孪生发布继续使用原始`q_actual`/`tcp_raw`，线程保留最新状态，候选目标不冒充反馈。已完成的H5 X/Y真实记录见[本轮报告](quest_tcp_xy_pass_xyz_20261002.md)；H6真实状态驱动和物理停止仍待现场会话。头显佩戴可读性、显示延迟没有本轮实测。

## 2026-10-02 22:33：H4 Z真机通过，X/Y候选

已读取真实H4 `data/h4_20261002_223344_924166/`：30.007秒/3749条，Z反馈峰值0.500039mm，无故障且正常退出。71.887ms仅为近边界低速松Grip后的命令收敛，不是物理停止或代表性速度停止验收。当前用根目录`START_QUEST_TCP_XY_LIVE.cmd`逐轴测X/Y，新代码H1后跟随，无需再跑J1/H2；[详细点击](quest_tcp_xy_quickstart.md)、[本轮指标与证据](quest_tcp_z_pass_xy_20261002.md)。X/Y、三维/姿态、6DoF、对齐动态误差、断网/物理停止和ACT C/D仍待验收。下方旧日期记录保留为历史。

## 2026-10-02 21:59 H3通过；22:19 Tool1 Z阶段面板回归

真实H3完成30秒、3748次下发、无故障且正常退出；旧21:34故障保留为历史。共用反馈发布继续只用实测q/TCP。新Player实际七阶段/过期提示及Tool1 Z文字回归通过，截图见`data/quest_guidance_player_20261002_221930_132482/`；原Workcell场景SHA未变。H4 Tool1 Z尚待现场运动验收，头显阅读和显示延迟未实测，不声称<100ms通过。见[本轮证据与范围](quest_h3_pass_to_tcp_z_20261002.md)、[新入口逐点击](quest_tcp_z_quickstart.md)。

## 2026-10-02 21:34：实际机器人状态主线

现有左侧S5+DH116场景/Player未重建。21:34H3记录真实手柄驱动J1约0.098°，孪生发布核心只取EDG实测q/TCP；但本轮没有量测Unity实际应用/显示延迟，不因此认定头显显示或<100ms通过。1832周期后10.555ms准备迟到保护退出；短时控制调度修复和最终30秒完整零SDK重放已通过，仍需真机复核，见[本轮报告](quest_j1_timing_fix_20261002.md)。未改CAD、相机操作、模型、三臂控制范围或碰撞批准状态。

## 2026-10-02 20:29：Quest现场阶段提示

没有重建场景/CAD；Workcell.unity SHA前后相同。最终Player新增世界空间阶段面板，七阶段切换、过期提示、中文字体和截图经实际进程验证，报告`data/quest_guidance_player_20261002_202924_476362/report.json`；20:08曾active=true，最终20:29无人操作回归active=false；操作者重新佩戴并进入Link后复查，头显阅读/听音仍待确认。现场输入模式禁用仿真演示/回放混入显示；真实机器人显示仍只取反馈。19:54 H3目标恒定，未通过手柄运动；桌面/头显显示延迟与完整碰撞安全不因此通过。详见[本轮修复](quest_j1_pose_fix_20261002_1954.md)。

## 2026-09-29/30 Quest主线更新

当前以 [P0/P1实测报告](quest_servo_p0_p1_report.md) 和 [Quest只读逐点击](quest_readonly_click_by_click.md) 为准。本轮S5 10001连接超时，未读回Tool1/负载，未发真实运动。新V2Twin诊断页已实际打开；OpenXR运行时报告头显不可用，Touch/Touch Plus profile缺失已修复，但头显/按键仍需复测。本文其余历史成功不自动升级为当前通过。


| 条目 | 实测证据与结论 |
|---|---|
| 来源保护 | 原 F 盘 STEP 与 D 盘只读副本 SHA256 同为 `40ab599b25f979f5a784765f1a611442e310bdec8c13ebbb72d58f979733fee2`。19 个 F 盘 SLDASM/SLDPRT 已复制到 V2 派生目录，逐件原件/副本 SHA 一致，原件未修改；`digital_twin/cad/derived/source_manifest.json`。旧 Player 七件 STL 和 URDF 已单独复制并记录 SHA，七件 STL 与旧 V2 SHA 完全相同。 |
| 导入与资产 | 443 节点逐件映射，重复别名7，BREP无效14/136，左 TX2 的17件被排除；唯一可动链为旧 Player S5 七件/六关节，**本轮另接20件DH116末端视觉件，手部接触物理尚未验证**。工装近景646457面、右旧型占位417389面；新 Unity、MJCF、URDF、USD、glTF 同批重建。`digital_twin/data/build_report.json`、`geometry_diagnostics.json`、`delivery_manifest.json`。 |
| Unity | 2022.3.62f3 批处理 Build/Validate/Windows Player 打包退出0，OpenXR Standalone loader 已注册；场景和2预制体回读、2张箱内真实渲染图。Windows Player 合成 SIM UDP+录制/回放及 `Orbit/Pan/Dolly` 相机坐标变化探针退出0；GUI 仿真→Unity Player 更新关节通过。`digital_twin/data/unity_build.log`、`unity_validate.log`、`unity_player_build.log`、`unity_runtime_probe.json`、`gui_unity_integration.json`。桌面截图视觉检查确认蓝 S5 完整、右旧占位不再透明。人工鼠标/HMD 仍需现场复核。 |
| MuJoCo | 3.3.7 MJCF可加载，6关节/6仿真执行器/19传感器/4相机；原生 FREE Viewer 启动2秒正常退出、不在循环中锁相机，记录 `digital_twin/data/mujoco_viewer_result.json`。重新渲染两张箱内截图。1000个8ms目标周期（8秒，物理1ms子步）完成，最大跟踪差0.0220rad、最大速度0.1824rad/s、代理接触0、状态有限。结果 `digital_twin/data/mujoco_sim_result.json`。仿真采用理想动力学前馈，**不能**当实机指标。 |
| 共源一致性 | 固定种子100关节全限位随机姿态：MuJoCo Link06 与来源 URDF 最大位置差3.02e-13m、旋转矩阵元素差9.48e-13；Unity/MuJoCo安装位置、关节轴/限位、三个视点字段一致，`digital_twin/data/consistency_report.json`。Unity C# 动态 FK 尚未做像素级/关节姿态自动测量，不能把配置一致性称为完整运行时验证。 |
| 状态接入 | 重建后 GUI 仿真自测→UDP5008→MuJoCo 影子收到209包、拒绝0；`digital_twin/data/gui_shadow_integration.json`。独立 Unity Windows Player 的合成 SIM UDP 测试验证关节更新、录制与回放；GUI仿真自测→UDP5007→Unity Player 运行时通过，见 `unity_runtime_probe.json`、`gui_unity_integration.json`。均无机器人命令。 |
| 全域离线扫描 | 来源6关节各自上下限角点64组、全域均匀随机1000组，1064组均在限位内，757组有**仿真代理接触**，最大代理穿透0.234m。代理缺另两臂/夹爪/线缆/门/工装，故扫描仅显示危险候选，不构成真实碰撞许可。`digital_twin/data/workspace_scan_result.json`。 |
| 回归 | 原 V2 64项离线测试全过；数字孪生6项离线测试全过；GUI仿真窗口自检通过；离线策略环境250步烟雾测试通过，未使用真实VLA权重。 |
| 尚未通过 | OpenXR头显实际佩戴、双眼立体视差与FPS（无HMD时运行日志 `XR_ERROR_FORM_FACTOR_UNAVAILABLE`）；右/第三台真实 S5 外形及基座、原生 CAD 两个缺失引用与两处无效 BREP、现场三臂/线缆/门/夹爪碰撞完整性、末端Tool1/手眼/左基座方向标定、真实 JAKA 伺服运动/丝滑与全工作空间测试。当前右臂只是旧 TX2 静态占位，不能把“其它 S5 几何已修复”判为通过。 |

两个箱内视角的实际渲染：![Unity 箱内操作者视角](../digital_twin/figures/unity_inside_operator.png)

![MuJoCo 箱内操作者视角](../digital_twin/figures/mujoco_inside_operator.png)

修复前真实渲染与新图的逐项对照、原生 SolidWorks 副本/转换映射见 [几何修复记录](digital_twin_geometry_fix.md)；操作视角见 [视角操作](digital_twin_view_controls.md)。

下一步现场验收按照 [V2伺服逐点击说明](V2_servo_usage_click_by_click.md) 的保护门槛执行；数字孪生当前只提供离线试验与只读影子，不改变真机部署状态。


## 最新ACT/DH116增量（2026-09-29）

最新要求明确左侧蓝色S5**带DH116**，覆盖前轮无手要求。旧Player CompleteEndEffector的20件视觉装配已复制到V2派生目录，来源哈希见 `data/act_source_manifest.json`。Unity/MuJoCo/URDF/USD/glTF接入同一来源；MuJoCo保持6个机械臂动态关节，手为运动学视觉、不伪造接触惯量。Unity F5模拟演示含手指，F1–F4及鼠标自由视角保留；MuJoCo原生视角操作不变。

采集入口与阶段A/B步骤见 [ACT采集操作](data_collection_pipeline.md)，本轮证据/未通过项见 [ACT验收](act_collection_acceptance.md)。原QDU权重和真实训练数据仍未复现，本轮新管线的改进是源时间戳/缺失标记/哈希/在线配对审计和独立异步记录，不宣称训练精度或力控效果优于原方案。模型显示、输入协议和合成episode成功不能代替现场碰撞、工具标定、真实VR和DH116力控验收。
