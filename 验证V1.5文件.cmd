@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -Command "$code=Get-Content -LiteralPath 'verify_v1_5_files.ps1' -Raw -Encoding UTF8; Invoke-Expression $code"
if errorlevel 1 (
    echo V1.5文件不完整，请重新下载并完整解压。
    pause
    exit /b 2
)
pause
exit /b 0
