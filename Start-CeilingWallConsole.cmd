@echo off
cd /d "%~dp0"
where py >nul 2>nul
if %errorlevel% equ 0 (
  py -3 -B src\px4_wall_ceiling_control\scripts\operator\ceiling_wall_console.py --bridge mock --port 8791 --open-browser %*
) else (
  python -B src\px4_wall_ceiling_control\scripts\operator\ceiling_wall_console.py --bridge mock --port 8791 --open-browser %*
)
pause
