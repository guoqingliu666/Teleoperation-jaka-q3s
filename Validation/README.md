# 验证记录放哪里

`Validation/` 是**本机验证与证据区**，不是程序源码。运行测试或硬件验收会在这里产生 JSONL、JSON、截图和日志；大部分内容不进入公开 GitHub 仓库。

- `continuous_sampled_follow/`：现行 ② 的会话事件与 SDK 追踪，包含真实设备坐标。先看 `finished`、`stop_confirmed`、`failure`，再看每段的命令与实测到位。
- `sdk_*`、`quest_selected_target/`、`offline_fake_sdk_follow/`：各阶段只读、离线或受限真机试验；不能互相代替验收结论。
- `live_*_sessions/`、`shadow_sessions/`：旧版真机与影子模式记录，仅供事故/版本追溯。
- `hardware_observation_synthetic/`、`demo_sessions/`、根目录的测试截图/JSON：离线或只读测试生成物。
- `*.md`：验证方法、历史结论与未解问题。特别注意过去成功的短段试验**不等于**连续、高速、大范围或姿态控制已获准。
- `unity_v2_fixture.json`、`jaka_robot_state_fixture.json`、`gui_test_config.json`、`mapping_test_config.json`：离线测试夹具，可随源码版本管理。

历史源码快照现位于 `Archive/源码与事故快照/`，不要把它们当作当前运行目录。绝不要把日志中的设备 IP、局域网信息、操作时间与实测坐标直接上传公开仓库。
