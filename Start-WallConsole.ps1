param(
  [string]$SshHost = '192.168.50.66',
  [string]$SshUser = 'pi',
  [string]$SshKey = "$env:USERPROFILE/.ssh/id_ed25519_wallrobot",
  [int]$SshPort = 22,
  [string]$KnownHosts = "$env:USERPROFILE/.ssh/known_hosts",
  [string]$Container = 'ros1_noetic',
  [string]$RemoteWorkspace = '/root/ros_ws',
  [string]$PythonPath = '',
  [string]$Settings = "$env:USERPROFILE/.codex/wall-console/settings.json",
  [int]$Port = 8787,
  [switch]$NoBrowser
)
$ErrorActionPreference = 'Stop'
if ($Port -lt 1 -or $Port -gt 65535 -or $SshPort -lt 1 -or $SshPort -gt 65535) {
  throw 'HTTP and SSH ports must be between 1 and 65535.'
}
$taskConsoleUrl = "http://127.0.0.1:$Port"
try {
  $taskExisting = Invoke-RestMethod "$taskConsoleUrl/api/status" -TimeoutSec 2
  if ($taskExisting.ssh -and $taskExisting.token) {
    if ($taskExisting.ssh.host -ne "$SshUser@$SshHost") {
      throw "Port $Port is already used by another console target: $($taskExisting.ssh.host). Use a different -Port."
    }
    if ($taskExisting.connection -and (
        $taskExisting.connection.ssh_port -ne $SshPort -or
        $taskExisting.connection.container -ne $Container -or
        $taskExisting.connection.remote_workspace -ne $RemoteWorkspace)) {
      throw "Port $Port is already used by another console connection configuration. Use a different -Port."
    }
    Write-Host "Console already running: $taskConsoleUrl"
    if (-not $NoBrowser) { Start-Process $taskConsoleUrl }
    return
  }
} catch {
  if ($_.Exception.Message -like 'Port * is already used*') { throw }
}
$taskPython = $PythonPath
if (-not $taskPython) {
  $taskBundled = Join-Path $env:USERPROFILE '.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe'
  $taskVenv = Join-Path $PSScriptRoot '.venv-wall-console/Scripts/python.exe'
  if (Test-Path -LiteralPath $taskVenv) { $taskPython = $taskVenv }
  elseif (Test-Path -LiteralPath $taskBundled) { $taskPython = $taskBundled }
  else {
    foreach ($taskName in @('python', 'python3')) {
      $taskCommand = Get-Command $taskName -ErrorAction SilentlyContinue
      if ($taskCommand) { $taskPython = $taskCommand.Source; break }
    }
  }
}
if (-not $taskPython -or -not (Test-Path -LiteralPath $taskPython)) {
  throw 'Python was not found. Install Python 3.10+ or provide -PythonPath with its full executable path.'
}
& $taskPython -c 'import sys; assert sys.version_info >= (3,10), "Python 3.10+ required"; import paramiko' 2>&1 | Out-Null
if ($LASTEXITCODE -ne 0) {
  throw 'Python 3.10+ and Paramiko are required. Install dependencies with: python -m pip install -r src/px4_wall_ceiling_control/requirements-console.txt'
}
foreach ($taskRequiredFile in @($SshKey, $KnownHosts)) {
  if (-not (Test-Path -LiteralPath $taskRequiredFile -PathType Leaf)) {
    throw "SSH file missing: $taskRequiredFile. Configure this computer's authorized SSH key and verify the Pi host fingerprint first."
  }
}
$taskServer = Join-Path $PSScriptRoot 'src/px4_wall_ceiling_control/scripts/operator/wall_console_server.py'
$taskLogs = Join-Path $env:LOCALAPPDATA "WallConsole/$Port"
New-Item -ItemType Directory -Path $taskLogs -Force | Out-Null
$taskArguments = @('"' + $taskServer + '"', '--ssh-host', $SshHost, '--ssh-user', $SshUser,
  '--ssh-key', '"' + $SshKey + '"', '--ssh-port', "$SshPort", '--known-hosts', '"' + $KnownHosts + '"',
  '--container', '"' + $Container + '"', '--remote-workspace', '"' + $RemoteWorkspace + '"',
  '--settings', '"' + $Settings + '"', '--port', "$Port")
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
