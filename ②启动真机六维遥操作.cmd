@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"
if not exist "%~dp0本机配置.cmd" goto no_config
call "%~dp0本机配置.cmd"
if not defined QUEST_JAKA_PYTHON goto no_config
if not defined QUEST_JAKA_HOST goto no_config
if not defined QUEST_JAKA_SDK_DIR goto no_config
if not defined QUEST_JAKA_LIMITS_FILE goto no_config
if not defined QUEST_JAKA_TEMP set "QUEST_JAKA_TEMP=D:\ChatGPT\Temp"
if not defined QUEST_JAKA_CACHE set "QUEST_JAKA_CACHE=D:\ChatGPT\Cache\quest-jaka-pycache"
set "PYTHON_EXE=%QUEST_JAKA_PYTHON%"
set "APP=%~dp0Python\quest_endpoint_teleop_gui.py"
set "TEMP=%QUEST_JAKA_TEMP%"
set "TMP=%QUEST_JAKA_TEMP%"
set "PYTHONPYCACHEPREFIX=%QUEST_JAKA_CACHE%"
set "PYTHONPATH=%~dp0Python\src"

if not exist "%PYTHON_EXE%" goto no_python
if not exist "%APP%" goto no_app
if /I "%QUEST_JAKA_CMD_CHECK%"=="1" goto check_ok

"%PYTHON_EXE%" -u "%APP%"
if errorlevel 1 goto failed
exit /b 0

:check_ok
echo CMD_CHECK_OK
exit /b 0

:no_python
echo ERROR: Python environment not found.
pause
exit /b 2

:no_app
echo ERROR: Teleoperation GUI not found.
pause
exit /b 3

:no_config
echo [错误] 缺少本机配置.cmd，或其中未填写 Python、JAKA 控制柜、SDK、限位文件。
echo 请把本机配置.example.cmd 复制为本机配置.cmd，按现场设备填写后重试。
pause
exit /b 5

:failed
echo ERROR: Teleoperation GUI exited with an error.
pause
exit /b 4
