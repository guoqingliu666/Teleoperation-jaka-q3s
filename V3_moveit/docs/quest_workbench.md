# Quest 三维工作台

在上位机正常启动后，Quest 浏览器刷新 `https://localhost:8443/`，点击“进入透视模式 · 三维工作台”。首次加载模型约 21 MiB，资源均由本机提供，不依赖外部 CDN。桌面浏览器也能预览、拖动视角和缩放；只有进入 Quest XR 会话才发送手柄输入。

场景按真实米制尺寸放在初始朝向前方约 1.35 m，保持在地面参考空间中。左右眼分别使用 WebXR 提供的投影和位姿，转头或侧身会改变观察角度。“工作台移到面前”只重置显示位置，不修改输入坐标映射。当前视图尚未与真实机械臂做空间对齐。

机械臂与灵巧手使用上位机的同一份 `.cache/operator/display.urdf` 和同一批网格。关节反馈来自既有本地显示快照；过期时显示“模型预览”，保留最后可见姿态，不宣称它是实时反馈。目标轴来自 `/v3/hand_target` 对应的显示快照，保留原四元数和基坐标，红 X、绿 Y、蓝 Z，长 12 cm。轨迹保留 5 秒，按年龄变色、淡出，不连接不同握持段；输入释放或数据过期时隐藏目标轴。

头显视野上方的状态卡显示启动进度。看到「可以开始遥操作」后，握住右侧 Grip，TCP1 随手柄移动。松开时控制器先减速，停稳后显示「已暂停跟随」。提示来自当前会话的控制器和手柄反馈。头显状态连接中断时会撤下就绪或跟随提示。卡片在小幅转头时保持在空间中，大幅转向时回到视野上方。[Apple 空间布局指南](https://developer.apple.com/design/human-interface-guidelines/spatial-layout)

## 模型来源

当前指定装配体为 `/home/gq/projects_lgq/V3_moveit/JAKAS5-Hand-model-SW/（11.16）手臂.SLDASM`。11 月 16 日更新读取并校验原生组件 XML CRC，核对了 66 个组件。快换、连接件和灵巧手共 58 个组件相对 11.13 装配整体旋转约 90°，机械臂本体的 8 个组件未变。31 个末端显示/碰撞元素同步更新，记录与只读来源副本在 `reference/hand_mount_20261116/`。

两端同用的显示 URDF SHA-256（默认 TCP1）：`61f6fac1bdbf574dbbb01e60b26fb9268b0b15051505514abdddcca1fd7e1dce`。

SLDASM SHA-256：`da5aeb91e873c3f5b8d7c45e20db93c2c11c882b2f58901640e662ad62d82635`。

可动灵巧手沿用厂家 `DH116-R000-A1` URDF 网格，按指定装配体中的 A4 手安装位置配准。已有表面配准记录的截尾平均误差约 0.214 mm；这不是本轮重新做的精度验收，也不是实物标定。两端显示模型一致，不表示它们逐面复制了 SLDASM 的全部原生曲面。

本轮核对记录在 `artifacts/model_tooling_20261116/model_identity.json`。上位机已正常关闭后重开，空闲 VR 服务已重启；`live_reload.json` 记录实际 HTTP 回读的模型哈希与桌面显示模型一致。Quest 已打开的页面需刷新后加载新模型。

## 工具坐标系与负载

遥操作页顶部显示 JAKA 当前工具和 V3 配置工具。点击“读取 TCP / 负载”，按钮下方弹出对照框，显示工具号、TCP XYZ / Rx Ry Rz、质量和质心 XYZ，以及回读时间。仿真模式也可只读查询控制柜；点击其他位置收起弹框。下拉框默认 TCP1；切换真机模式并确认现场准备后，“应用工具”会在静止、无 Servo/程序/拖动占用时选择 JAKA 已保存的工具号，回读成功后保存 TCP 定义。V3 的 FK、控制模型和两端显示使用同一 TCP 变换；原始机械臂运动学不变。

负载或质心不一致时，工作台弹出提醒：先在 JAKA 上位机重新检测并保存，再点击“已校准，刷新负载”。该按钮只读取控制柜并更新 V3 的 `src/v3_moveit/config/payload.json`，不写入柜端负载。保存使用完整浮点精度，原配置和原始回读保存在 `artifacts/tooling_updates/`。读数无效、操作失败、工具不一致或已有控制会话时不保存。负载和质心校准不再改变软件版本标识，也不阻止普通仿真启动；实机验收单独绑定校准数据，更改后须更新对应实机检查。工具号或 TCP 几何改变仍会使原软件与模型检查过期，不会把旧实机结果标为通过。

## 连接与仿真启动

设备页 JAKA S5 卡片保留三个按钮：“连接机械臂”、本体电源、机器人使能。主动连接后每两秒只读更新状态，电源和使能直接显示在按钮上，不再因一次回读超过 30 秒退成待机。再次点击“已连接”可断开监测。后台查询在控制会话启动前退出，释放 SDK 和控制锁；遥操作期间不并行查询。上电、使能仍需选择真机模式、确认现场准备并通过原有模型、TCP、负载及状态检查。

实际 Servo、程序、拖动、运动队列占用，以及 SDK 明确的程序 / EDG 占用错误，会让连接按钮显示红色“被占用”。普通通信失败显示“连接失败 · 重试”，不能仅凭 SDK 的通信错误推断是手机占用。手机或其他上位机退出占用后，自动查询会更新状态。

日常仿真和真机遥操作都从遥操作页启动。真机模式会在当次启动中核对工具、TCP、负载、质心、模型和实时反馈。长时间保持和跟随测试可在“检查与验收”页单独运行。若头显显示「等待右手柄」，唤醒右手柄并让头显看到它。

点击进入后，按钮旁会显示 Quest 是否正在开启透视。系统请求尚未完成时，可以在头显内查看系统提示，或点「取消进入」。如果页面明确提示「Quest 未能开启透视」，可完全退出 Quest Browser，再打开 V3。11 月 19 日的现场记录中，重启曾让头显短暂返回双眼帧，随后旧版工作台在提交画面时导致浏览器退出。新版使用 Three.js 的标准 WebXR 渲染流程。

同日还复现了透视刚打开就退回网页的问题。Quest 第一次确定地面位置时，旧版结束了尚未发出有效位姿的会话。修复后，操作者从正式入口进入并启动真机遥操作。在 61.89 秒的观察中，24 次采样均有双眼画面，每眼 1680 × 1760，浏览器发出 5358 帧有效右手柄位姿。握住 Grip 时头显显示「TCP1 跟随中」，松开后上位机记录到暂停。实测记录在 `artifacts/quest_stereo_20261119/entry_result.json`。

11 月 18 日实测 2 号机械臂地址为 `192.168.110.224`，Ubuntu 有线网卡 `enp11s0` 的 EDG 源地址为 `192.168.110.67`。这两个地址分别对应 `site_ubuntu.yaml` 中的 `robot_ip` 和 `edg_client_ip`。控制柜只读结果见 `artifacts/teleop_cabinet_check_20261118T224232_469284Z/result.json`。

## 实现与输入边界

- `tools/operator_xr/server.py` 组合原 `quest/server.py` 转发器与只读资源、状态接口。模型网格按内容哈希建立明确白名单，不开放工程目录或 TLS 文件。
- `quest/index.html` 管理入口按钮、输入映射、发送频率和停止条件。三维工作台使用同一脚本，并载入 `workbench.js` 作为显示模块。按钮在模型就绪后启用，点击即向头显申请透视，显示会话就绪后才打开手柄通道。
- `tools/operator_quest.py` 启动新版服务，并通过 `/xr/health` 识别服务。原 USB 8443 reverse 和独立 watcher 保留。
- `robot.js` 读取显示 URDF 的导出数据，`view.js` 处理显示坐标和场景摆放。`workbench.js` 通过 Three.js 的 `WebXRManager` 绘制双眼画面，包括模型、反馈、目标、轨迹和状态卡。
- ROS 到 WebXR 的显示变换为 `(x,y,z) → (x,z,-y)`，右手坐标旋转；它不参与控制输入映射。
- 第一帧有效手柄位姿发出前，Quest 如果重新定位地面，工作台会按新的参考空间摆放。有效位姿已开始发送后，参考空间重置会结束输入并退出。渲染失败也会结束输入。

Three.js 固定为官方 `r180`，源码、加载器和 OrbitControls 位于 `tools/operator_xr/vendor/`，附 MIT LICENSE、下载来源和 SHA-256 清单。头显负责提供每只眼的视点和投影，Three.js 管理显示层、帧缓冲和呈现时序。[WebXR 视图规范](https://www.w3.org/TR/webxr/#xrview-interface) · [Three.js r180 WebXRManager](https://github.com/mrdoob/three.js/blob/r180/src/renderers/webxr/WebXRManager.js)

## 验证

```bash
/usr/bin/python3 -B -m unittest discover -s tools -p 'operator_*tests.py'
/usr/bin/python3 -B tools/operator_xr/browser_check.py
/usr/bin/python3 -B tools/verify_offline.py
env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_PACKAGE_PATH PATH=/usr/bin:/bin bash tools/verify_ros.sh
/usr/bin/python3 -B tools/update_hand_mount_20261116.py
/usr/bin/python3 -B tools/quest_usb_probe.py
```

浏览器检查使用真实 Chromium/WebGL、合成 XR 运行时、随机 TLS/UDP 端口，验证 FK、双眼差异、头移时场景固定、状态过期和原输入转发。它不会使用生产 UDP 输入端口或连接机器人。测试使用本机 Codex 附带的 Node/Playwright；生产页面不依赖这些测试工具。

11 月 16 日模型更新阶段：21 项离线检查、123 项上位机回归、31 项原生驱动替身测试、完整 ROS fake 功能回归及真实 Chromium 的合成双眼测试通过。Qt 布局在 1240×800、1640×1040 下无水平裁切，预览截图在 `artifacts/model_tooling_20261116/workbench_1640x1040.png`；重开后的实时 RViz 渲染帧在同目录的 `rviz_model.png`。浏览器报告在 `artifacts/operator_xr_20261116/`。桌面外部截图返回黑色，未用它作为视觉通过证据。这一阶段没有连接机器人。

随后功能精修的验证与截图保存在 `artifacts/operator_refine_20261116/`。普通 `operator_session` 的实际 fake ROS 启动通过，使用独立 ROS 域和随机输入端口；仅替代 USB 发现，真实执行松握等待、启动、跟随反馈及停止。关节最大运动 0.043733 rad，松握后最后一秒漂移为 0；合成输入不代替佩戴 Quest 的 60 秒现场检查。完整 ROS 回归再次通过，仍有既有的 MoveIt 退出清理告警，不能声称所有进程均干净退出。

本次另外执行两次实机只读 SDK 连接：状态读取及 TCP / 负载 / 模型检查通过；控制柜在线、已上电、已使能、空闲，TCP1=[0,0,200,0,0,0]，质量 2.2524310740921463 kg，质心 [-11.531157260634972,-4.866724134676426,63.995698264012205] mm，与 V3 一致。没有上电、使能、Servo、运动或工具切换调用，尚未完成新版真实 Quest 跟随与现场停止验收。最终测试数量和重开记录见该目录 `result.json`。
