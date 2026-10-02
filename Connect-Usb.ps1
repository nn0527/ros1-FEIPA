param(
  [Parameter(Mandatory=$true)][ValidatePattern('^[0-9]+-[0-9]+$')][string]$BusId,
  [Parameter(Mandatory=$true)][ValidatePattern('^/dev/tty(ACM|USB)[0-9]+$')][string]$LinuxPort
)
$ErrorActionPreference = 'Stop'
$taskUsbTool = 'C:\Program Files\usbipd-win\usbipd.exe'
& $taskUsbTool bind --busid $BusId
if ($LASTEXITCODE -ne 0) {
  $taskBind = Start-Process -FilePath $taskUsbTool -ArgumentList @('bind','--busid',$BusId) -Verb RunAs -WindowStyle Hidden -Wait -PassThru
  if ($taskBind.ExitCode -ne 0) { throw 'USB sharing failed' }
}
& wsl.exe -d Ubuntu-20.04 -u root -- modprobe cdc_acm
if ($LASTEXITCODE -ne 0) { throw 'CDC ACM driver unavailable' }
if ($LinuxPort -match '^/dev/ttyUSB') {
  & wsl.exe -d Ubuntu-20.04 -u root -- modprobe ch341
  if ($LASTEXITCODE -ne 0) { throw 'CH340 serial driver unavailable' }
}
& $taskUsbTool attach --wsl --busid $BusId
if ($LASTEXITCODE -ne 0) { throw 'USB attachment failed; check usbipd list' }
for ($taskAttempt=0; $taskAttempt -lt 20; $taskAttempt++) {
  & wsl.exe -d Ubuntu-20.04 -- test -c $LinuxPort
  if ($LASTEXITCODE -eq 0) { break }
  Start-Sleep -Milliseconds 500
}
& wsl.exe -d Ubuntu-20.04 -- test -c $LinuxPort
if ($LASTEXITCODE -ne 0) { throw "No device at $LinuxPort; inspect Linux serial devices" }
& wsl.exe -d Ubuntu-20.04 -u root -- chgrp dialout $LinuxPort
if ($LASTEXITCODE -ne 0) { throw 'Serial group update failed' }
& wsl.exe -d Ubuntu-20.04 -u root -- chmod 660 $LinuxPort
if ($LASTEXITCODE -ne 0) { throw 'Serial permission update failed' }
Write-Host "Attached USB $BusId; serial port $LinuxPort"
