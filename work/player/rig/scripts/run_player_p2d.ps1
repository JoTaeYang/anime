# T250 (P2.6): player P2d chain. Blender rigtest -> export stages + FBX + copy to Unity -> Unity PlayerImport ->
# Unity PlayerRigCheck. Stops at the first failing step and exits with its exit code. Unity steps go through
# unity_player.ps1 (waits for Unity.exe only, logs unity/AvatarCheck/Logs/player_<Method>.log). The Unity editor
# must be closed. Blender logs: work/player/rig/export/logs/<step>.log.
$ErrorActionPreference = "Stop"
$Repo = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..\..")).Path
$Blender = "C:\Program Files\Blender Foundation\Blender 5.1\blender.exe"
$Rig = Join-Path $Repo "work\player\rig"
$LogDir = Join-Path $Rig "export\logs"
New-Item -ItemType Directory -Force -Path $LogDir | Out-Null
$Timing = @()

function Invoke-BlenderStep([string]$StepName, [string[]]$StepArgs) {
    $log = Join-Path $LogDir "$StepName.log"
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $p = Start-Process -FilePath $Blender -ArgumentList $StepArgs -PassThru -NoNewWindow `
        -RedirectStandardOutput $log -RedirectStandardError "$log.err"
    $null = $p.Handle
    $p.WaitForExit()
    $sw.Stop()
    return @($p.ExitCode, $sw.Elapsed.TotalSeconds)
}

function Invoke-UnityStep([string]$Method) {
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    & (Join-Path $PSScriptRoot "unity_player.ps1") -Method $Method | Out-Host
    $code = $LASTEXITCODE
    $sw.Stop()
    return @($code, $sw.Elapsed.TotalSeconds)
}

$steps = @(
    @{ Name = "p25_rigtest"; Kind = "blender";
       Args = @("--background", "--factory-startup", (Join-Path $Rig "pl_r04_ctrl.blend"), "--python", (Join-Path $PSScriptRoot "p25_rigtest.py")) },
    @{ Name = "p27_export_fbx"; Kind = "blender";
       Args = @("--background", "--factory-startup", "--python", (Join-Path $PSScriptRoot "p27_export_fbx.py")) },
    @{ Name = "PlayerImport.Run"; Kind = "unity" },
    @{ Name = "PlayerRigCheck.Run"; Kind = "unity" }
)
foreach ($s in $steps) {
    if ($s.Kind -eq "blender") { $r = Invoke-BlenderStep $s.Name $s.Args } else { $r = Invoke-UnityStep $s.Name }
    $line = "{0} exit {1} wall {2:F1} s" -f $s.Name, $r[0], $r[1]
    Write-Output "[run_player_p2d] $line"
    $Timing += $line
    if ($r[0] -ne 0) {
        Write-Output "[run_player_p2d] FAILED at $($s.Name)"
        Set-Content -Path (Join-Path $LogDir "run_player_p2d_timing.log") -Value $Timing -Encoding utf8
        exit $r[0]
    }
}
Set-Content -Path (Join-Path $LogDir "run_player_p2d_timing.log") -Value $Timing -Encoding utf8
Write-Output "[run_player_p2d] all steps exit 0"
exit 0
