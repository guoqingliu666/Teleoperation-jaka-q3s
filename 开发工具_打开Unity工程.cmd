@echo off
chcp 65001 >nul
setlocal
set "TEMP=D:\ChatGPT\Temp"
set "TMP=D:\ChatGPT\Temp"
if not exist "%~dp0Logs" mkdir "%~dp0Logs"

rem 仅用于修改/调试VR场景；日常遥操作无需打开Unity编辑器。
"D:\计算\Unity Hub\unity编辑器\2022.3.62f3\Editor\Unity.exe" -projectPath "%~dp0QuestPoseBridge" -logFile "%~dp0Logs\Unity编辑器.log"
