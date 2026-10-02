param(
  [string]$SshHost = '192.168.50.66',
  [string]$SshUser = 'pi',
  [string]$SshKey = "$env:USERPROFILE/.ssh/id_ed25519_wallrobot",
  [int]$Port = 8787,
  [switch]$NoBrowser
)
$ErrorActionPreference = 'Stop'
$taskConsoleUrl = "http://127.0.0.1:$Port"
try {
  $taskExisting = Invoke-RestMethod "$taskConsoleUrl/api/status" -TimeoutSec 2
  if ($taskExisting.ssh -and $taskExisting.token) {
    Write-Host "Console already running: $taskConsoleUrl"
    if (-not $NoBrowser) { Start-Process $taskConsoleUrl }
    return
  }
} catch { }
$taskPython = Join-Path $env:USERPROFILE '.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe'
if (-not (Test-Path -LiteralPath $taskPython)) {
  throw 'Codex bundled Python was not found. Run with Python 3.8+ and paramiko installed.'
}
$taskServer = Join-Path $PSScriptRoot 'src/px4_wall_ceiling_control/scripts/operator/wall_console_server.py'
$taskLogs = Join-Path $env:LOCALAPPDATA 'WallConsole'
New-Item -ItemType Directory -Path $taskLogs -Force | Out-Null
$taskArguments = @('"' + $taskServer + '"', '--ssh-host', $SshHost, '--ssh-user', $SshUser,
  '--ssh-key', '"' + $SshKey + '"', '--port', "$Port")
$taskProcess = Start-Process -FilePath $taskPython -ArgumentList $taskArguments -WindowStyle Hidden -PassThru `
  -RedirectStandardOutput (Join-Path $taskLogs 'server.log') -RedirectStandardError (Join-Path $taskLogs 'server-error.log')
for ($taskAttempt = 0; $taskAttempt -lt 30; $taskAttempt++) {
  try {
    $taskStatus = Invoke-RestMethod "$taskConsoleUrl/api/status" -TimeoutSec 1
    if ($taskStatus.token) {
      Write-Host "Console ready: $taskConsoleUrl (PID $($taskProcess.Id))"
      if (-not $NoBrowser) { Start-Process $taskConsoleUrl }
      return
    }
  } catch { }
  if ($taskProcess.HasExited) { throw "Console exited. Check $taskLogs/server-error.log" }
  Start-Sleep -Milliseconds 200
}
throw "Console startup timed out. Check $taskLogs/server-error.log"
