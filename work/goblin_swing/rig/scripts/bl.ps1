$Script = ""; $Blend = ""; $ScriptArgs = @(); $i = 0
while ($i -lt $args.Count) {
    $t = [string]$args[$i]
    if ($t -eq "-Script") { $Script = [string]$args[$i + 1]; $i += 2; continue }
    if ($t -eq "-Blend") { $Blend = [string]$args[$i + 1]; $i += 2; continue }
    if ($t -eq "--") {
        if ($i + 1 -lt $args.Count) { $ScriptArgs = @($args[($i + 1)..($args.Count - 1)]) }
        break
    }
    Write-Error "bl.ps1: unexpected argument '$t' (usage: -Script <py> [-Blend <blend>] [-- <args>])"
    exit 64
}
if (-not $Script) { Write-Error "bl.ps1: -Script is required"; exit 64 }
$Blender = "C:\Program Files\Blender Foundation\Blender 5.1\blender.exe"
$Rig = Split-Path -Parent $PSScriptRoot
$a = @("--background", "--factory-startup")
if ($Blend) { $a += (Join-Path $Rig $Blend) }
$a += @("--python-exit-code", "1", "--python", (Join-Path $PSScriptRoot $Script))
if ($ScriptArgs.Count) { $a += @("--") + $ScriptArgs }
& $Blender @a
exit $LASTEXITCODE
