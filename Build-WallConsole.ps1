param([string]$PythonPath = "$PSScriptRoot/.venv-wall-console/Scripts/python.exe")
$ErrorActionPreference = 'Stop'
# Build dependencies belong on the developer computer only.
& $PythonPath -m PyInstaller --noconfirm --clean --onefile --windowed --name WallConsole `
  --distpath "$PSScriptRoot/dist" --workpath "$PSScriptRoot/build/portable" --specpath "$PSScriptRoot/build/portable" `
  --paths "$PSScriptRoot/src/px4_wall_ceiling_control/src" `
  --hidden-import px4_wall_ceiling_control.wall_console --hidden-import paramiko --hidden-import http.server `
  --add-data "$PSScriptRoot/src/px4_wall_ceiling_control/web;px4_wall_ceiling_control/web" `
  --add-data "$PSScriptRoot/src/px4_wall_ceiling_control/scripts/operator/wall_console_server.py;px4_wall_ceiling_control/scripts/operator" `
  "$PSScriptRoot/src/px4_wall_ceiling_control/scripts/operator/windows_console.py"
if ($LASTEXITCODE -ne 0) { throw 'Portable console build failed.' }
Copy-Item -LiteralPath "$PSScriptRoot/src/px4_wall_ceiling_control/docs/windows_portable.txt" -Destination "$PSScriptRoot/dist/使用说明.txt"
Copy-Item -LiteralPath "$PSScriptRoot/src/px4_wall_ceiling_control/scripts/operator/configure_console.cmd" -Destination "$PSScriptRoot/dist/修改连接设置.cmd"
