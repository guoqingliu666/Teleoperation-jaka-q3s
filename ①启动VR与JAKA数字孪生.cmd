@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"
if not exist "%~dp0本机配置.cmd" goto no_config
call "%~dp0本机配置.cmd"
if not defined QUEST_JAKA_PYTHON goto no_config
if not defined QUEST_JAKA_HOST goto no_config
if not defined QUEST_JAKA_SDK_DIR goto no_config
if not defined QUEST_JAKA_TEMP set "QUEST_JAKA_TEMP=D:\ChatGPT\Temp"
if not defined QUEST_JAKA_CACHE set "QUEST_JAKA_CACHE=D:\ChatGPT\Cache\quest-jaka-pycache"
set "TEMP=%QUEST_JAKA_TEMP%"
set "TMP=%QUEST_JAKA_TEMP%"
set "PYTHONPYCACHEPREFIX=%QUEST_JAKA_CACHE%"
if not exist "%~dp0Logs" mkdir "%~dp0Logs"
if not defined QUEST_JAKA_PLAYER_EXE set "QUEST_JAKA_PLAYER_EXE=%~dp0Player_通信修复\QuestPosePreview.exe"
set "PLAYER_EXE=%QUEST_JAKA_PLAYER_EXE%"
set "PYTHON_EXE=%QUEST_JAKA_PYTHON%"
set "BRIDGE_SCRIPT=%~dp0Python\src\vla_lab\jaka_vr_readonly_bridge.py"

rem 先验证必需文件，避免路径错误时窗口一闪而过。
if not exist "%PLAYER_EXE%" (
    echo [错误] 找不到VR程序：
    echo %PLAYER_EXE%
    echo 请不要单独移动这个CMD，它必须保持在项目根目录。
    pause
    exit /b 2
)
if not exist "%PYTHON_EXE%" (
    echo [错误] 找不到 JAKA Python：
    echo %PYTHON_EXE%
    pause
    exit /b 2
)
if not exist "%BRIDGE_SCRIPT%" (
    echo [错误] 找不到数字孪生只读反馈桥：
    echo %BRIDGE_SCRIPT%
    pause
    exit /b 2
)

rem 给维护人员使用的无运行自检：只检查CMD解析与路径，不启动VR。
if /I "%QUEST_JAKA_CMD_CHECK%"=="1" (
    echo CMD_CHECK_OK: VR与JAKA数字孪生入口
    exit /b 0
)

rem 该程序同时完成三件事：读取Quest头显/手柄、把手柄发给Python、显示JAKA S5实测数字孪生。
rem 为避免两个Player争抢UDP来源，只允许启动一个实例。
tasklist /FI "IMAGENAME eq QuestPosePreview.exe" 2>NUL | find /I "QuestPosePreview.exe" >NUL
if not errorlevel 1 (
    echo [提示] VR数字孪生已经在运行，请勿重复启动。
    echo 若它仍显示平放姿态，请先关闭旧①及Unity窗口，再双击本文件运行新版只读反馈。
    pause
    exit /b 0
)

rem ①单独运行时只读实机关节；②启动时请求①释放SDK，结束后①恢复只读。
rem 机器人模型只接受实测关节，不把手柄目标冒充为实机姿态。
set "PYTHONPATH=%~dp0Python\src"
"%PYTHON_EXE%" -u "%BRIDGE_SCRIPT%" --live-readonly --host "%QUEST_JAKA_HOST%" --sdk-dir "%QUEST_JAKA_SDK_DIR%" --player-exe "%PLAYER_EXE%" --player-log "%~dp0Logs\VR数字孪生.log"
set "EXIT_CODE=%ERRORLEVEL%"
echo.
echo [提示] VR数字孪生程序已退出，返回码为 %EXIT_CODE%。
if not "%EXIT_CODE%"=="0" echo [日志] %~dp0Logs\VR数字孪生.log
pause
exit /b %EXIT_CODE%

:no_config
echo [错误] 缺少本机配置.cmd，或其中未设置 QUEST_JAKA_PYTHON。
echo 请复制“本机配置.example.cmd”为“本机配置.cmd”，填写本机路径后重试。
pause
exit /b 5
