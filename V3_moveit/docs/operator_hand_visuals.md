# 灵巧手推动模型与触觉监控

2026-11-15，当前工程 `/home/gq/projects_lgq/V3_moveit`。

顶部小开关点击切换浅色 / 深色，保存选择并保留过渡动画。会话控制在创建完成后同步主题，打开窗口无需再拖动调色条。

灵巧手抓握页左侧使用 LHandPro 1.4.5 的原始 DH116 三维模型。左键只拖动粉色关节，选中时高亮；位置输入框和滑块连续同步。右键拖动旋转视角，中键平移，滚轮缩放，点击复位视角恢复默认取景。空白、银色外壳和黑色指垫不响应左键拖动。拖动灵敏度已限制，避免侧向观察关节时一像素跨越几百当量；模型连续显示与最多 30 Hz 的运动目标发送分开处理，松开鼠标时提交最后目标，失去焦点时取消尚未发送的拖动更新。预览只修改显示目标。实物模式先连接只读反馈，再明确点击“使能关节”；滑块拖动会以最多 30 Hz 发送目标，松开时提交最终位置。目标沿用现有速度、电流、上下限、受力停止和失鲜停止。新的模型推动尚未做实机动作验收。

右上角蓝色圆形「i」打开苹果风格的开发者页面。14 个章节、88 个岗位覆盖产品、架构、评审、设计、开发、验证、交付和维护，署名统一为「刘国庆」。页面以片尾字幕方式缓慢滚动，可暂停、滚轮浏览或从头播放，随工作台切换明暗。它是制作名单，不是验收状态表；打开窗口不会阻塞工作台的控制按钮。

触觉监控位于遥操作下方，右侧重复的五指触觉栏已移除。热力页左侧为波形、右侧为放大的厂商手图；法向、切向、接近等数值只在详细数据页显示。

- 热力颜色显示 SDK 手指通道的归一化强度，范围 0–100%，不是 N。当前 SDK 各指通道数为 5 / 9 / 9 / 9 / 5。
- 通道中心使用厂家返回的局部坐标；手图配准与离散分区用于显示，尚无毫米级定位标定，也不能测出精确物理接触边界。悬浮可查看通道、强度和局部坐标。
- 橙色箭头从通道强度重心出发，将每指法向与切向合成为一个局部向量，长度表示幅值；它是二维斜投影，不是机器人基坐标中的空间方向。方向无效或反馈失鲜时不显示旧箭头。
- 波形可选择合力、法向、切向（N）或通道峰值（%），选择 5 / 10 / 30 秒窗口或暂停。原始样本有界保存，显示使用保峰降采样；大于 80 ms 的数据间隙断线显示，不补造连续信号。采样率和绘制率分开显示。

波形直接使用成熟开源项目 [PyQtGraph](https://github.com/pyqtgraph/pyqtgraph)，绘制采用官方 [PlotDataItem 的 peak / clipToView 选项](https://pyqtgraph.readthedocs.io/en/latest/api_reference/graphicsItems/plotdataitem.html)。调度上限 120 Hz，实际刷新受数据源、窗口尺寸、历史长度及电脑负载影响，不代表设备采样率。

依赖固定在 `tools/requirements_operator_visual.lock`。本机已安装在工程缓存；重建环境时，在工程目录执行：

```bash
python3 -m pip install --no-deps --target .cache/operator/python --requirement tools/requirements_operator_visual.lock
```

NumPy、SciPy、PyQt5 沿用现有 Ubuntu 依赖。模型资产的原文件摘要和显示旋转约定位于 `tools/operator_assets/dh116_vendor/model/`；没有改动控制 URDF 或桌面 / VR 共用的装配模型。

界面与性能验证入口：

```bash
QT_QPA_PLATFORM=offscreen python3 -B -m unittest discover -s tools -p 'operator_*tests.py'
QT_QPA_PLATFORM=xcb python3 -B tools/operator_hand_refine_visual.py
```

后者只创建预览窗口和合成波形，不连接硬件。截图、指标、完整 ROS 和离线回归日志保存在 `artifacts/operator_hand_refine_20261115/`。重建原生 DH116 程序会清除文件能力；本轮已恢复原先的 `cap_net_raw=ep`，没有新增 sudoers 配置。
