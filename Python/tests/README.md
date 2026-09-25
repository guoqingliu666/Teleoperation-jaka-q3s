# 离线测试

原来散落在 `Python/` 根目录的 `verify_*.py` 与 `test_repro.py` 统一放在这里。测试脚本不会成为用户日常入口；部分旧测试会打开 Tk 测试窗口或生成 `Validation/` 文件，因此建议在有桌面的 Windows 上运行。

在项目的 `Python/` 目录执行：

```powershell
$env:TEMP='D:\ChatGPT\Temp'
$env:TMP='D:\ChatGPT\Temp'
$env:PYTHONPYCACHEPREFIX='D:\ChatGPT\Cache\quest-jaka-test-pycache'
& $env:QUEST_JAKA_PYTHON -m unittest discover -s tests -p 'verify_*.py'
```

路径中的 Python 解释器只是本机例子，其他电脑应改成自己的环境。所有真机动作必须由单独入口和现场门槛触发；这里的假 SDK、源码检查或只读测试通过，**不表示**可以做真机连续跟随。`verify_vendor_joint_teleop_candidate.py` 的事故日志回放在公开克隆缺少本机原始记录时会跳过。

测试大致分四组：`verify_sampled_*` 覆盖当前 ②，`verify_lookahead_shadow` 覆盖零命令的最近 3 点影子候选；`verify_jaka_readonly`、`verify_motion_status_readonly` / `verify_hik_preview` 覆盖只读外设；`verify_engineering_*`、`verify_micro_teleop`、`verify_vendor_*` 是旧方案回归；`verify_player` 是当前 Player 的手动检查，不属于纯离线验收。
