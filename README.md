# Meta Quest 3S → JAKA S5 数字孪生与六维遥操作

这是一套 Windows/Unity/Meta Quest 3S/JAKA S5 的研究工程。现行②已经完成启动TCP周围100cm软件包络、50mm/s和会话初始姿态±30°的连续六维功能验收；现场确认方向与连续性正确、关节无异常大幅变化，且无抖动、异响或报警。控制链每条完整TCP命令≤20mm且≤2°，段内调用厂商`kine_inverse`筛查并使用`linear_move_extend_ori`执行，不使用伺服，也不补跑手柄历史轨迹。当前新增100mm/s、400mm/s²、姿态20°/s的高速档作为下一项现场验收；高速档仍保留相同的单段、逆解、关节连续性、限位余量和停止门槛，未经验收前不作为已验证性能。灵巧手实物开合仍未放开。项目曾发生J4欠压及保护停机，见[安全事件记录](docs/安全事件_20260919_真机遥操作停用与初步调查.md)。

## 先找到需要的文件

| 想做什么 | 看哪里 |
| --- | --- |
| 了解现行两个按钮的作用、现阶段限制 | [（0）程序入口说明.md](（0）程序入口说明.md) |
| 看清数据流、每个 Python 文件与目录用途 | [docs/目录与代码索引.md](docs/目录与代码索引.md) |
| 查看当前短段验收与停止逻辑 | [docs/连续采样遥操作_使用与审查.md](docs/连续采样遥操作_使用与审查.md) |
| 区分现行、只读与历史真机源码 | [docs/源码安全分级.md](docs/源码安全分级.md) |
| 查看厂商连续段圆滑接口与下一门槛 | [docs/JAKA连续段圆滑接口核查.md](docs/JAKA连续段圆滑接口核查.md) |
| 查测试记录、原始日志在哪里 | [Validation/README.md](Validation/README.md) |
| 找旧程序、旧文档及事故前后快照 | [Archive/README.md](Archive/README.md) |
| 准备发布到 GitHub | [docs/GitHub发布检查清单.md](docs/GitHub发布检查清单.md) |

日常只用根目录的 `①启动VR与JAKA数字孪生.cmd` 与 `②启动真机六维遥操作.cmd`。其他 `.cmd` 是只读工具、开发工具或旧模拟入口，详见索引。`.cmd` 只负责设置环境并启动 Unity/Python，不是另一套控制算法。也可在 PyCharm 用同一个解释器运行 `Python/quest_endpoint_teleop_gui.py` 打开 ② 的界面。

## 运行前必须知道

1. 复制`本机配置.example.cmd`为`本机配置.cmd`，填写 Python、JAKA SDK、控制柜 IP、限位配置和 D 盘缓存路径；真实配置默认不提交。源码仍保留当前实验机默认值以兼容现场，但**克隆仓库不会自动适配另一台机器**。先按现场核对配置并只做离线/只读检查，不要复制他人的真机参数直接运行。
2. ①运行 Windows Player，并在②未占用 SDK 时只读 JAKA 实测关节/TCP；②开始会话时①先注销，②独占连接并把实测反馈送给 Unity，结束后①恢复只读。黄色点是手柄目标，不是真机到位证明。断连时 Unity 的默认模型姿态也不能当成实机位置。
3. 软件不替操作者上电、使能、清报警或验证碰撞空间；STOP 请求也不能代替物理急停。真实动作需要现场规程和独立确认。静态测试、历史日志与画面正常均不构成真机放行依据。

## 源码结构

```text
①*.cmd / ②*.cmd       当前两个 Windows 入口
Python/                当前 GUI、运行入口、算法模块、离线测试
QuestPoseBridge/       Unity 工程源码：Quest 输入与数字孪生
Player_通信修复/        当前本机 Windows Player 构建
docs/                  当前技术路线、索引与安全审查
Validation/            测试夹具与本机验收输出
Archive/               旧构建、旧入口、旧文档和快照（默认不提交）
3D模型/                CAD 原件（默认不提交）
```

GitHub 上传前先检查[忽略规则](.gitignore)和[源码安全分级](docs/源码安全分级.md)：Unity `Library`、本机 UOS 生成密钥、录制数据、原始真机日志、CAD/机器人模型、第三方运行时和本机 Player 构建均不直接提交。本仓库自有源码使用[MIT许可证](LICENSE)；JAKA SDK、Unity运行时、JAKA/DH116模型、CAD和相机厂商SDK仍分别受其权利人许可约束，不随源码仓库分发。仓库尚未完成异机硬件验收，不要将“能克隆源码”误解为“能安全开机运行”。

离线回归命令和夹具说明见 [Python/tests/README.md](Python/tests/README.md)。
