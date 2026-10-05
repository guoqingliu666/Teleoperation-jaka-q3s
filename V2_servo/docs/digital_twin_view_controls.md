# 数字孪生视角操作（Unity / MuJoCo / VR）

2026-10-04 更新：默认正向起点；`Home`/`F1` 回正，`Tab` 显示/隐藏详情。VR 详情在右前方世界固定位置，不随头移动。VR 鼠标左右拖改变水平朝向、上下拖改变高度，俯仰/侧倾由头显追踪提供。以下历史头显缺失记录不作为本轮状态；本轮 OpenXR active，但未佩戴验收，详见[修复报告](tcp_camera_fix_20261004.md)。


数字孪生入口在 `D:\ChatGPT\Projects\VR\V2_servo`。Unity 和 MuJoCo 的箱内预设只是打开时的起点，可以继续自由观察；改变视角只改变显示，不会发机器人命令。相机源码分别是 [`TwinCameraModes.cs`](../digital_twin/unity/Assets/V2Twin/Scripts/TwinCameraModes.cs) 与 [`run_mujoco.py`](../digital_twin/run_mujoco.py)。

| 操作 | MuJoCo 3.3.7 原生 Viewer | Unity `V2Twin.exe` 的 Game 视图 |
|---|---|---|
| 环绕旋转 | 鼠标左键拖动 | 鼠标左键拖动；立即从预设切到自由视角 |
| 平移 | 鼠标右键拖动（垂直平面），Shift+右键拖动（水平平面） | 鼠标中键拖动，或 Shift+左键拖动 |
| 缩放 | 滚轮或中键拖动 | 滚轮 |
| 原地转头 | 由 MuJoCo 自身的相机操作处理，本项目不增加拦截 | 鼠标右键拖动 |
| 自由移动 | Viewer 原生相机，不由 V2 覆盖 | WASD，Q 下移、E 上移；按住 Shift 加速 |
| 操作/固定/自由/末端预设 | XML 有 `inside_operator`、`inside_fixed`、`free_start`、`tcp_follow` 四个相机，可用 Viewer 原生相机菜单切换；V2 启动先给 FREE 相机一个箱内起点 | F1 箱内操作、F2 箱内固定、F3 自由起点、F4 Link06 跟随 |

MuJoCo 使用 `mujoco.viewer.launch_passive()` 自带鼠标处理。V2 只在启动后设置一次 `mjCAMERA_FREE`、观察中心、距离和角度；循环只调用 `viewer.sync()`，**不重置相机、不接管拖拽、不绑定额外鼠标动作**。因此在弹出窗口中点空白区域后可直接左拖、右拖、滚轮。官方原生操作表见 [MuJoCo Code samples → Mouse](https://mujoco.readthedocs.io/en/latest/programming/samples.html#mouse)。CLI：`python digital_twin\run_mujoco.py --viewer`；无窗口环境用 `python digital_twin\run_mujoco.py --render-views` 重新渲染三张固定视角图。

Unity 先双击 `digital_twin\unity\Build\V2Twin.exe`，鼠标点击窗口里的画面以获得焦点；若在 Unity Editor，打开 `Assets/V2Twin/Scenes/Workcell.unity`、点 Play、切到 **Game** 页。F1/F2/F3/F4 随时切换；任意鼠标拖动或滚轮会从预设切到自由模式，此后不会被下一帧拉回固定相机。若想重新回到箱内起点，再按 F1。F4 跟随的是 **模型 Link06**；它不是已完成实测标定的 Tool1 TCP。

VR 模式使用已注册的 Standalone OpenXR loader，`TwinView_Camera` 设为双眼渲染。若头显设备有效，代码读取 XR `Head` 的位置与姿态；头显自身六自由度跟踪持续生效，鼠标左/右拖会调整其世界锚点方向，滚轮移动锚点，中键/Shift+左键平移锚点，F1 可复位到箱内起点。这样鼠标可“摇动”观察起点，但不会把头显追踪锁在预设姿态。**本机无可用头显会产生 OpenXR `XR_ERROR_FORM_FACTOR_UNAVAILABLE`；桌面视角、编译和运行探针已通过，双目立体深度、头动延迟、帧率及真实头显鼠标联合操作没有实测通过。**正式 VR 验收需佩戴头显检验左右眼不同视差、转头/平移、鼠标调整锚点和持续帧率，保存录屏与日志。

本轮窗口探针结果为 [`unity_runtime_probe.json`](../digital_twin/data/unity_runtime_probe.json)：合成 SIM 状态更新关节、录制/回放以及 `Orbit/Pan/Dolly` 使相机坐标实际变化；MuJoCo 原生 Viewer 启动 2 秒退出码 0，退出报告在 [`mujoco_viewer_result.json`](../digital_twin/data/mujoco_viewer_result.json)，并生成 [`mujoco_inside_operator.png`](../digital_twin/figures/mujoco_inside_operator.png)。这些证明代码路径可运行；自动探针不能代替人手逐次拖动或头显佩戴验收。
