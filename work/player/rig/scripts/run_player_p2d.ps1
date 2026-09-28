# T340: thin wrapper. The logic (T250 P2d chain: p25 rigtest -> p27 export -> Unity PlayerImport -> Unity PlayerRigCheck;
# logs, timing log and exit codes) lives in run_player.py p2d. See docs/mac-setup.md.
$Py = "python"; $PyArgs = @()
if (-not (Get-Command python -ErrorAction SilentlyContinue)) { $Py = "py"; $PyArgs = @("-3") }
& $Py @PyArgs (Join-Path $PSScriptRoot "run_player.py") p2d
exit $LASTEXITCODE
