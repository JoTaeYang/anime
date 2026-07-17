param([Parameter(Position = 0)][string]$Stage = "all")

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

function Invoke-UnityCheck {
    Write-Host ">>> unity: AvatarCheck.Run"
    # Unity relaunches as a separate process, so $LASTEXITCODE after `& $Unity` is
    # unreliable (Task 8). Use Start-Process -Wait -PassThru and read .ExitCode.
    $unityArgs = @(
        "-batchmode", "-nographics", "-quit",
        "-projectPath", (Join-Path $Root "unity\AvatarCheck"),
        "-executeMethod", "AvatarCheck.Run",
        "-logFile", (Join-Path $Root "unity\AvatarCheck\Logs\check.log")
    )
    $proc = Start-Process -FilePath $Unity -ArgumentList $unityArgs -Wait -PassThru -NoNewWindow
    $code = $proc.ExitCode
    $report = Join-Path $Root "unity\AvatarCheck\report.json"
    if (Test-Path $report) { Get-Content $report }
    if ($code -ne 0) {
        Write-Host "FAILED: Unity check (exit $code)" -ForegroundColor Red
        exit $code
    }
}

$Pipeline = [ordered]@{
    "mesh"   = @("scripts\stages\00_mesh.py", "scripts\checks\check_00.py")
    "rig"    = @("scripts\stages\01_rig.py", "scripts\checks\check_01.py")
    "anim"   = @("scripts\stages\02_anim.py", "scripts\checks\check_02.py")
    "bake"   = @("scripts\stages\03_bake.py", "scripts\checks\check_03.py")
    "export" = @("scripts\stages\04_export.py", "scripts\checks\check_04.py")
}

switch ($Stage) {
    "smoke" { Invoke-Blender "scripts\stages\smoke.py" }
    "unity" { Invoke-UnityCheck }
    "sheet" { Invoke-Blender "scripts\preview\contact_sheet.py" }
    "all" {
        Invoke-Blender "scripts\stages\smoke.py"
        foreach ($pair in $Pipeline.Values) { foreach ($s in $pair) { Invoke-Blender $s } }
        Invoke-UnityCheck
        Invoke-Blender "scripts\preview\contact_sheet.py"
        Write-Host "ALL STAGES PASSED" -ForegroundColor Green
    }
    default {
        if (-not $Pipeline.Contains($Stage)) {
            Write-Host "usage: run.ps1 [smoke|mesh|rig|anim|bake|export|unity|sheet|all]"
            exit 2
        }
        foreach ($s in $Pipeline[$Stage]) { Invoke-Blender $s }
    }
}
