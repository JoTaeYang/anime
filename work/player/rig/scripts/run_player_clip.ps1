param([Parameter(Mandatory = $true)][string]$Clip, [string]$Prefix = "")  # -Prefix: clip gate prefix for checker row ids (main, 2026-09-27)
# T300 (P3 tooling, spec work/player/d-03-player-anim-tools.md section 2): one player clip, end to end.
#   p30_clip_anim (pl_r04_ctrl.blend -> rig/anim/pl_a_<clip>.blend + sheets) -> check_player_clip.py (skipped with a
#   notice when missing) -> p31_export_clip (stage + Player@<clip>.fbx + Unity copy) -> Unity PlayerClipCheck.Run
#   (PLAYER_CLIP=<clip>, via unity_player.ps1) -> check_player_clip_unity.py (skipped with a notice when missing).
# Stops at the first non-zero exit and exits with that code. Unity must be closed: a running Unity.exe stops the run
# before the Unity step (exit 3). Blender logs: work/player/rig/export/logs/clip_<clip>_<step>.log (+ .err);
# Unity log: unity/AvatarCheck/Logs/player_PlayerClipCheck.Run.log. Wall times: clip_<clip>_timing.log.
$ErrorActionPreference = "Stop"
$S = $PSScriptRoot
$Repo = (Resolve-Path (Join-Path $S "..\..\..\..")).Path
$Blender = "C:\Program Files\Blender Foundation\Blender 5.1\blender.exe"
$Rig = Join-Path $Repo "work\player\rig"
$LogDir = Join-Path $Rig "export\logs"
New-Item -ItemType Directory -Force -Path $LogDir | Out-Null
$Timing = @()

function Invoke-BlenderStep([string]$StepName, [string[]]$StepArgs) {
    $log = Join-Path $LogDir "clip_${Clip}_$StepName.log"
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $p = Start-Process -FilePath $Blender -ArgumentList $StepArgs -PassThru -NoNewWindow `
        -RedirectStandardOutput $log -RedirectStandardError "$log.err"
    $null = $p.Handle
    $p.WaitForExit()
    $sw.Stop()
    return @($p.ExitCode, $sw.Elapsed.TotalSeconds)
}

function Invoke-UnityStep([string]$Method) {
    $running = Get-Process -Name "Unity" -ErrorAction SilentlyContinue
    if ($running) {
        Write-Host "[run_player_clip] Unity.exe is running (pids $($running.Id -join ', ')); close it first"
        return @(3, 0.0)
    }
    $env:PLAYER_CLIP = $Clip
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    & (Join-Path $S "unity_player.ps1") -Method $Method | Out-Host
    $code = $LASTEXITCODE
    $sw.Stop()
    return @($code, $sw.Elapsed.TotalSeconds)
}

$steps = @(
    @{ Name = "p30_clip_anim"; Kind = "blender"; Script = "p30_clip_anim.py";
       Args = @("--background", "--factory-startup", (Join-Path $Rig "pl_r04_ctrl.blend"), "--python", (Join-Path $S "p30_clip_anim.py"), "--", "--clip", $Clip) },
    @{ Name = "check_player_clip"; Kind = "blender"; Script = "check_player_clip.py";
       Args = @("--background", "--factory-startup", "--python", (Join-Path $S "check_player_clip.py"), "--", "--clip", $Clip) + $(if ($Prefix) { @("--prefix", $Prefix) } else { @() }) },
    @{ Name = "p31_export_clip"; Kind = "blender"; Script = "p31_export_clip.py";
       Args = @("--background", "--factory-startup", "--python", (Join-Path $S "p31_export_clip.py"), "--", "--clip", $Clip) },
    @{ Name = "PlayerClipCheck.Run"; Kind = "unity"; Script = $null },
    @{ Name = "check_player_clip_unity"; Kind = "blender"; Script = "check_player_clip_unity.py";
       Args = @("--background", "--factory-startup", "--python", (Join-Path $S "check_player_clip_unity.py"), "--", "--clip", $Clip) + $(if ($Prefix) { @("--prefix", $Prefix) } else { @() }) }
)
$all = [System.Diagnostics.Stopwatch]::StartNew()
foreach ($step in $steps) {
    if ($step.Script -and -not (Test-Path (Join-Path $S $step.Script))) {
        $line = "{0} skipped ({1} missing)" -f $step.Name, $step.Script
        Write-Host "[run_player_clip] NOTICE $line"
        $Timing += $line
        continue
    }
    if ($step.Kind -eq "blender") { $r = Invoke-BlenderStep $step.Name $step.Args } else { $r = Invoke-UnityStep $step.Name }
    $line = "{0} exit {1} wall {2:F1} s" -f $step.Name, $r[0], $r[1]
    Write-Host "[run_player_clip] $line"
    $Timing += $line
    if ($r[0] -ne 0) {
        Write-Host "[run_player_clip] FAILED at $($step.Name) (clip $Clip)"
        $Timing += "FAILED at $($step.Name); total wall {0:F1} s" -f $all.Elapsed.TotalSeconds
        Set-Content -Path (Join-Path $LogDir "clip_${Clip}_timing.log") -Value $Timing -Encoding utf8
        exit $r[0]
    }
}
$Timing += "all steps exit 0; total wall {0:F1} s" -f $all.Elapsed.TotalSeconds
Set-Content -Path (Join-Path $LogDir "clip_${Clip}_timing.log") -Value $Timing -Encoding utf8
Write-Host "[run_player_clip] all steps exit 0 (clip $Clip), total wall $("{0:F1}" -f $all.Elapsed.TotalSeconds) s"
exit 0
