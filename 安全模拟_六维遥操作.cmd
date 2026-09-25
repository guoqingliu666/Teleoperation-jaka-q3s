@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"
if not exist "%~dp0本机配置.cmd" goto no_config
call "%~dp0本机配置.cmd"
if not defined QUEST_JAKA_PYTHON goto no_config
if not defined QUEST_JAKA_TEMP set "QUEST_JAKA_TEMP=D:\ChatGPT\Temp"
if not defined QUEST_JAKA_CACHE set "QUEST_JAKA_CACHE=D:\ChatGPT\Cache\quest-jaka-pycache"
set "TEMP=%QUEST_JAKA_TEMP%"
set "TMP=%QUEST_JAKA_TEMP%"
set "PYTHONPYCACHEPREFIX=%QUEST_JAKA_CACHE%"
set "PYTHONPATH=%~dp0Python\src"
if /I "%QUEST_JAKA_CMD_CHECK%"=="1" (
    echo CMD_CHECK_OK: 安全模拟六维遥操作入口
    exit /b 0
)

rem 与真机界面相同，但底层是模拟机器人；可用来练按钮和观察Unity数字孪生。
"%QUEST_JAKA_PYTHON%" -m vla_lab.engineering_teleop_live --demo --host 127.0.0.1
if errorlevel 1 pause
exit /b %ERRORLEVEL%

:no_config
echo [错误] 缺少本机配置.cmd，或其中未设置 QUEST_JAKA_PYTHON。
echo 请复制“本机配置.example.cmd”为“本机配置.cmd”，填写本机路径后重试。
pause
exit /b 5
