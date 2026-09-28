# T340: thin wrapper. The logic (T254: chain p20..p24 + P2d, the seven P2 checkers, collector and compare) lives in
# run_player.py gates (collector / compare: gates_collect.py). See docs/mac-setup.md.
#   Full run:   powershell -NoProfile -File run_player_gates.ps1 [-Run <n>] [-SkipChain] [-ContinueOnChainFailure]
#   Compare:    powershell -NoProfile -File run_player_gates.ps1 -Compare -A 1 -B 2
param(
    [int]$Run = 0,
    [switch]$SkipChain,
    [switch]$ContinueOnChainFailure,
    [switch]$Compare,
    [int]$A = 0,
    [int]$B = 0
)
$Py = "python"; $PyArgs = @()
if (-not (Get-Command python -ErrorAction SilentlyContinue)) { $Py = "py"; $PyArgs = @("-3") }
$a2 = @((Join-Path $PSScriptRoot "run_player.py"), "gates")
if ($Compare) { $a2 += @("--compare", "$A", "$B") } else { $a2 += @("--run", "$Run") }
if ($SkipChain) { $a2 += "--skip-chain" }
if ($ContinueOnChainFailure) { $a2 += "--continue-on-chain-failure" }
& $Py @PyArgs @a2
exit $LASTEXITCODE
