param([Parameter(Mandatory = $true)][string]$Clip, [string]$Prefix = "")  # -Prefix: clip gate prefix for checker row ids (main, 2026-09-27)
# T340: thin wrapper. The logic (T300: p30_clip_anim -> check_player_clip -> p31_export_clip -> Unity PlayerClipCheck.Run
# -> check_player_clip_unity; logs, timing log and exit codes) lives in run_player.py clip. See docs/mac-setup.md.
$Py = "python"; $PyArgs = @()
if (-not (Get-Command python -ErrorAction SilentlyContinue)) { $Py = "py"; $PyArgs = @("-3") }
$a = @((Join-Path $PSScriptRoot "run_player.py"), "clip", $Clip)
if ($Prefix) { $a += @("--prefix", $Prefix) }
& $Py @PyArgs @a
exit $LASTEXITCODE
