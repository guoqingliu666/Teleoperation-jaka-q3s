@echo off
setlocal
cd /d "%~dp0.."
set "TEMP=D:\ChatGPT\Temp"
set "TMP=D:\ChatGPT\Temp"
call "D:\compute\Visual_Studio_products\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64
if errorlevel 1 exit /b 1
if not exist artifacts mkdir artifacts
cl /nologo /std:c++17 /EHsc /utf-8 /W4 /I src\v3_moveit\include /I vendor\jaka\include tools\guard_tests.cpp /Fe:artifacts\guard_tests.exe /Fo:artifacts\guard_tests.obj
if errorlevel 1 exit /b 1
artifacts\guard_tests.exe
