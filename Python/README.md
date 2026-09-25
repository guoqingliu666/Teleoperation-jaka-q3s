# Python 目录：先读这里

用户入口只有 `quest_endpoint_teleop_gui.py`（②窗口）。它启动 `quest_endpoint_teleop_entry.py`，后者加载 `连续采样位置遥操作.py`；具体输入、运动短段和实测反馈分别在 `src/vla_lab/`。脚本分层是为避免 GUI、Quest UDP 和同步 JAKA SDK 调用互相阻塞，并保证只有一个进程持有 SDK 连接。

`tests/` 是离线/只读回归，`config/` 是早期 Jog/观察器的配置模板，`third_party/` 是本机第三方依赖，`logs/`、`datasets/` 是运行产物。`third_party/` 默认不入 Git；依赖授权和版本必须在公开发布前核对。

根目录的 `真机验证新②_*.py` 是过去的受限单项现场试验，`只读检查*.py` 不发运动命令，`分析*.py` 与 `benchmark_*.py` 只处理本地数据/替身。它们**不是**新的日常遥操作入口。完整逐文件说明与安全等级见 [目录与代码索引](../docs/目录与代码索引.md)。

开发时先运行 `python -m unittest discover -s tests -p 'verify_*.py'`（解释器和 D: 缓存设置见 [tests/README.md](tests/README.md)）。这只验证代码的部分离线行为，不能替代 JAKA App 的状态和现场验收。
