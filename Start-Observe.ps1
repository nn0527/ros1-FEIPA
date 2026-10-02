param(
  [Parameter(Mandatory=$true)][ValidatePattern('^/dev/[A-Za-z0-9_./:-]+$')][string]$FcuPort,
  [ValidateRange(1200,3000000)][int]$Baud = 57600,
  [ValidatePattern('^$|^/dev/[A-Za-z0-9_./:-]+$')][string]$CeilingPort = '',
  [ValidatePattern('^$|^/dev/[A-Za-z0-9_./:-]+$')][string]$FrontPort = ''
)
$ErrorActionPreference = 'Stop'
if (!$CeilingPort) { $CeilingPort = '-' }
if (!$FrontPort) { $FrontPort = '-' }
& wsl.exe -d Ubuntu-20.04 -- bash --noprofile --norc /mnt/c/Users/60497/Desktop/catkin_ws/runtime/run-observe.sh $FcuPort $Baud $CeilingPort $FrontPort
exit $LASTEXITCODE
