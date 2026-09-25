@echo off
rem 复制本文件为“本机配置.cmd”后填写；该真实文件已被 .gitignore 排除。
rem 不要把现场 IP、SDK 路径、控制器配置或密钥提交到公开 GitHub。

set "QUEST_JAKA_PYTHON=C:\path\to\python.exe"
set "QUEST_JAKA_HOST=192.168.x.x"
set "QUEST_JAKA_SDK_DIR=D:\path\to\JAKA_SDK\python3\x64"
set "QUEST_JAKA_LIMITS_FILE=D:\path\to\exported\usersettings.ini"
set "QUEST_JAKA_TEMP=D:\ChatGPT\Temp"
set "QUEST_JAKA_CACHE=D:\ChatGPT\Cache\quest-jaka-pycache"
rem 若不使用项目根目录的默认 Player 构建位置，可再设置：
rem set "QUEST_JAKA_PLAYER_EXE=D:\path\to\QuestPosePreview.exe"
rem 若希望海康预览默认选中某台相机，可在本机配置中填写其序列号：
rem set "HIK_PREFERRED_SERIAL=your-camera-serial"
