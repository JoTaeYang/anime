# T340: thin wrapper. The logic (whole goblin chain s00..s09 with the G1..G8 checkers, Unity GoblinRigCheck, compare_g9
# and the G10.3 scope list) lives in run_goblin.py gates. See docs/mac-setup.md.
$Py = "python"; $PyArgs = @()
if (-not (Get-Command python -ErrorAction SilentlyContinue)) { $Py = "py"; $PyArgs = @("-3") }
& $Py @PyArgs (Join-Path $PSScriptRoot "run_goblin.py") gates
exit $LASTEXITCODE
