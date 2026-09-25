@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"
if not exist "%~dp0本机配置.cmd" goto no_config
call "%~dp0本机配置.cmd"
set "TEMP=%QUEST_JAKA_TEMP%"
set "TMP=%QUEST_JAKA_TEMP%"
set "PYTHONPYCACHEPREFIX=%QUEST_JAKA_CACHE%"
set "PYTHONPATH=%~dp0Python\src"

rem 只查询JAKA关节、TCP、Tool和状态，不上电、不使能、不运动。
"%QUEST_JAKA_PYTHON%" -m vla_lab.jaka_readonly_gui
if errorlevel 1 pause
exit /b 0
:no_config
echo [错误] 请先复制本机配置.example.cmd为本机配置.cmd并填写本机路径。
pause
exit /b 2
