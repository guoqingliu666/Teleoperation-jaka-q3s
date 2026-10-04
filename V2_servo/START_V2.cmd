@echo off
setlocal
cd /d "%~dp0"
set "TEMP=D:\ChatGPT\Temp"
set "TMP=D:\ChatGPT\Temp"
set "PYTHONDONTWRITEBYTECODE=1"
if not defined V2_PYTHON if defined CONDA_PREFIX set "V2_PYTHON=%CONDA_PREFIX%\python.exe"
if not defined V2_PYTHON if exist "%~dp0launch\local_python.cmd" call "%~dp0launch\local_python.cmd"
if not defined V2_PYTHON set "V2_PYTHON=python"
"%V2_PYTHON%" "%~dp0run_v2_app.py" %*
if errorlevel 1 (
  echo V2 failed. Activate conda environment jaka, then run python run_v2_app.py --check
  pause
)
