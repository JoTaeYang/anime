param([Parameter(Mandatory = $true)][string]$Method)
# T340: thin wrapper. The logic (T210: Unity batch run that waits for the Unity process only; log
# unity/AvatarCheck/Logs/player_<Method>.log, wall time appended to player_timing.log) lives in run_player.py unity.
$Py = "python"; $PyArgs = @()
if (-not (Get-Command python -ErrorAction SilentlyContinue)) { $Py = "py"; $PyArgs = @("-3") }
& $Py @PyArgs (Join-Path $PSScriptRoot "run_player.py") unity $Method
exit $LASTEXITCODE
