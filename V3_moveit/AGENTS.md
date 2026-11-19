# V3 Ubuntu MoveIt 工程

- 本目录是用户于 2026-11-12 授权的新 V3，目标 Ubuntu 24.04 / ROS 2 Jazzy。
- V2_servo、QuestPoseBridge 和现有 Windows 启动器只读，不迁移、不覆盖、不删除。
- 复用 MoveIt Servo、OMPL、ros2_control 与厂家 SDK；只写必要适配。
- 默认仿真；真实硬件必须明确选择 hardware 模式并通过模型、反馈、TCP、状态检查。
- 记录仿真、编译、实机分别完成到哪一步，不能将单元测试写成实机验收。
- 用户于 2026-11-14 更正：Ubuntu 当前工程为 `/home/gq/projects_lgq/V3_moveit`，后续代码、build/install/log、缓存与环境均放在这里。本条替代旧的“仅在原 D 盘工作”要求。
- `/media/gq/software/ChatGPT/Projects/VR/V3_moveit` 保留为历史来源，不继续在那里开发或启动。历史交接中的目录结论不得覆盖上述用户更正。
- 对模型、SDK、协议或控制代码的改动须运行 tools/verify_offline.py；有 ROS 时运行 tools/verify_ros.sh。
