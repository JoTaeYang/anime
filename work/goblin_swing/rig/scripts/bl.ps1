# T340: thin wrapper. The logic (headless Blender run of rig/scripts/<script> on rig/<blend>) lives in
# run_goblin.py bl. Usage unchanged: bl.ps1 -Script <py> [-Blend <blend>] [-- <args>]. See docs/mac-setup.md.
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
$Py = "python"; $PyArgs = @()
if (-not (Get-Command python -ErrorAction SilentlyContinue)) { $Py = "py"; $PyArgs = @("-3") }
$a = @((Join-Path $PSScriptRoot "run_goblin.py"), "bl", $Script)
if ($Blend) { $a += $Blend }
if ($ScriptArgs.Count) { $a += @("--") + $ScriptArgs }
& $Py @PyArgs @a
exit $LASTEXITCODE
