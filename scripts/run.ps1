param(
    [Parameter(Position = 0)][string]$Stage = "all",
    [string]$Profile = "character"
)
$env:ANIME_PROFILE = $Profile

$Root = Split-Path -Parent $PSScriptRoot
$Blender = "C:\Program Files\Blender Foundation\Blender 5.1\blender.exe"
$Unity = "C:\Program Files\Unity\Hub\Editor\6000.3.20f1\Editor\Unity.exe"

function Invoke-Blender([string]$RelScript) {
    Write-Host ">>> blender: $RelScript"
    & $Blender --background --factory-startup --python-exit-code 1 --python (Join-Path $Root $RelScript)
    if ($LASTEXITCODE -ne 0) {
        Write-Host "FAILED: $RelScript (exit $LASTEXITCODE)" -ForegroundColor Red
        exit $LASTEXITCODE
    }
}

function Invoke-Unity([string]$Method, [string]$LogName, [string]$ReportName, [switch]$Graphics) {
    Write-Host ">>> unity: $Method"
    # Unity relaunches as a separate process, so $LASTEXITCODE after `& $Unity` is
    # unreliable (Task 8). Use Start-Process -Wait -PassThru and read .ExitCode.
    # -Graphics drops -nographics for stages that render frames (e.g. capture); all
    # existing callers omit it and keep the byte-identical headless arg list.
    $unityArgs = @("-batchmode")
    if (-not $Graphics) { $unityArgs += "-nographics" }
    $unityArgs += @(
        "-quit",
        "-projectPath", (Join-Path $Root "unity\AvatarCheck"),
        "-executeMethod", $Method,
        "-logFile", (Join-Path $Root "unity\AvatarCheck\Logs\$LogName")
    )
    $proc = Start-Process -FilePath $Unity -ArgumentList $unityArgs -Wait -PassThru -NoNewWindow
    $code = $proc.ExitCode
    $report = Join-Path $Root "unity\AvatarCheck\$ReportName"
    if (Test-Path $report) { Get-Content $report }
    if ($code -ne 0) {
        Write-Host "FAILED: $Method (exit $code)" -ForegroundColor Red
        exit $code
    }
}
function Invoke-UnityCheck { Invoke-Unity "AvatarCheck.Run" "check.log" "report.json" }

function Invoke-UnityCapture {
    # Renders clip frames -> needs graphics, so NOT -nographics. No report.json to print;
    # print the per-clip capture.json instead if it landed.
    $clip = if ($env:ANIME_CAPTURE_CLIP) { $env:ANIME_CAPTURE_CLIP } else { "Walk" }
    Invoke-Unity "ClipCapture.Run" "capture.log" "unused_capture_report.json" -Graphics
    $capJson = Join-Path $Root "previews\character\unity_capture\${clip}_capture.json"
    if (Test-Path $capJson) { Get-Content $capJson }
}

$MeshStage = "scripts\stages\00_mesh.py"
if ($Profile -ne "dummy") { $MeshStage = "scripts\stages\00_intake.py" }
$Pipeline = [ordered]@{
    "mesh"   = @($MeshStage, "scripts\checks\check_00.py")
    "rig"    = @("scripts\stages\01_rig.py", "scripts\checks\check_01.py")
    "anim"   = @("scripts\stages\02_anim.py", "scripts\checks\check_02.py")
    "bake"   = @("scripts\stages\03_bake.py", "scripts\checks\check_03.py")
    "export" = @("scripts\stages\04_export.py", "scripts\checks\check_04.py")
}

switch ($Stage) {
    "smoke" { Invoke-Blender "scripts\stages\smoke.py" }
    "unity" { Invoke-UnityCheck }
    "clips" { Invoke-Unity "ClipImport.Run" "clips.log" "clips_report.json" }
    "playscene" { Invoke-Unity "PlaySceneBuild.Run" "playscene.log" "playscene_report.json" }
    "capture" { Invoke-UnityCapture }
    "probe" {
        # Numeric retarget diagnosis. Like `clips`, it needs no rendering, so keep the
        # headless (-nographics) arg list. No unity/AvatarCheck report to print; the probe
        # writes previews/character/unity_capture/retarget_probe.json — print that after.
        Invoke-Unity "RetargetProbe.Run" "probe.log" "unused_probe_report.json"
        $probeJson = Join-Path $Root "previews\character\unity_capture\retarget_probe.json"
        if (Test-Path $probeJson) { Get-Content $probeJson }
    }
    "sheet" { Invoke-Blender "scripts\preview\contact_sheet.py" }
    "userpreview" { Invoke-Blender "scripts\preview\user_preview.py" }
    "overlay" { Invoke-Blender "scripts\preview\metarig_overlay.py" }
    "walkcompare" { Invoke-Blender "scripts\preview\walk_compare.py" }
    "all" {
        Invoke-Blender "scripts\stages\smoke.py"
        foreach ($pair in $Pipeline.Values) { foreach ($s in $pair) { Invoke-Blender $s } }
        Invoke-UnityCheck
        Invoke-Blender "scripts\preview\contact_sheet.py"
        Write-Host "ALL STAGES PASSED ($Profile)" -ForegroundColor Green
    }
    default {
        if (-not $Pipeline.Contains($Stage)) {
            Write-Host "usage: run.ps1 [smoke|mesh|rig|anim|bake|export|unity|clips|playscene|capture|probe|sheet|overlay|userpreview|walkcompare|all]"
            exit 2
        }
        foreach ($s in $Pipeline[$Stage]) { Invoke-Blender $s }
    }
}
