param([Parameter(Mandatory = $true)][string]$Method)
# T210: Unity batch run for the player (modelled on work/goblin_swing/rig/scripts/unity_goblin.ps1).
# Log: unity/AvatarCheck/Logs/player_<Method>.log; wall time appended to unity/AvatarCheck/Logs/player_timing.log.
$Repo = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..\..")).Path
$Unity = "C:\Program Files\Unity\Hub\Editor\6000.3.20f1\Editor\Unity.exe"
$LogDir = Join-Path $Repo "unity\AvatarCheck\Logs"
$args2 = @("-batchmode", "-quit", "-projectPath", (Join-Path $Repo "unity\AvatarCheck"),
           "-executeMethod", $Method, "-logFile", (Join-Path $LogDir "player_$Method.log"))
$sw = [System.Diagnostics.Stopwatch]::StartNew()
# Wait for Unity.exe only. Start-Process -Wait also waits for the child dotnet process Unity leaves behind (~+10 min).
$p = Start-Process -FilePath $Unity -ArgumentList $args2 -PassThru -NoNewWindow
$null = $p.Handle   # cache the handle so ExitCode is available after exit
$p.WaitForExit()
$sw.Stop()
$line = "{0} {1} exit {2} wall {3:F1} s" -f (Get-Date -Format "yyyy-MM-ddTHH:mm:ss"), $Method, $p.ExitCode, $sw.Elapsed.TotalSeconds
Write-Output "[unity_player] $line"
Add-Content -Path (Join-Path $LogDir "player_timing.log") -Value $line -Encoding utf8
exit $p.ExitCode
