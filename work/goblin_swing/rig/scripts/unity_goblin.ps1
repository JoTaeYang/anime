param([Parameter(Mandatory = $true)][string]$Method)
# T340: thin wrapper. The logic (Unity batch run that waits for the Unity process only; log
# unity/AvatarCheck/Logs/goblin_<Method>.log) lives in run_goblin.py unity. See docs/mac-setup.md.
$Py = "python"; $PyArgs = @()
if (-not (Get-Command python -ErrorAction SilentlyContinue)) { $Py = "py"; $PyArgs = @("-3") }
& $Py @PyArgs (Join-Path $PSScriptRoot "run_goblin.py") unity $Method
exit $LASTEXITCODE
