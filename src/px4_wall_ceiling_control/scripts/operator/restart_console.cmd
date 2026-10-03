@echo off
"%SystemRoot%\System32\taskkill.exe" /IM WallConsole.exe /T /F >nul 2>&1
start "" "%~dp0WallConsole.exe"
