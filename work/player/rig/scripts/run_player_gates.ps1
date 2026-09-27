# T254 (P2.7): run the whole player chain, then every player checker, and collect exit codes + evidence rows +
# input sha256 into work/player/inspect/P2/gates_run_<n>.json.  Analogue of work/goblin_swing/rig/scripts/
# run_all_gates.ps1 (goblin file not used).  Main runs it twice and compares the two runs.
#
#   Full run:   powershell -NoProfile -File run_player_gates.ps1 [-Run <n>] [-SkipChain] [-ContinueOnChainFailure]
#   Compare:    powershell -NoProfile -File run_player_gates.ps1 -Compare -A 1 -B 2
#
# Chain: every production script p20*..p24* in name order, launched with the blend named on the "CLI:" line of its
# docstring (so a new p24x step is picked up automatically), then run_player_p2d.ps1 (p25 rigtest -> p27 export, which
# builds the p26 stages -> Unity PlayerImport -> PlayerRigCheck).  The chain stops at the first non-zero exit; the
# checkers are then skipped unless -ContinueOnChainFailure.
# Checkers (always all, in this order, exit code recorded, a failing checker does not stop the run): check_p2_static,
# check_p2_deform, check_p2_skel, check_p2_skin, check_p2_ctrl, check_p2_export, check_p2_skirt.
# Collection (Blender's bundled python, embedded below): for every checker evidence JSON: sha256 of the file, every row
# id / ok / sha256 of its measured value (timing keys "seconds", "timing_s", "timing_s_report" removed before
# hashing), the evidence inputs [path, sha256] and renders.  Compare: rows (ok, measured hash), inputs (sha256),
# exit codes; writes gates_compare_<a>_<b>.json; exit 0 when identical, 1 when anything differs.
# Logs: work/player/rig/export/logs/gates_<step>.log (Blender stdout / stderr).  The Unity editor must be closed.
param(
    [int]$Run = 0,
    [switch]$SkipChain,
    [switch]$ContinueOnChainFailure,
    [switch]$Compare,
    [int]$A = 0,
    [int]$B = 0
)
$ErrorActionPreference = "Stop"
$S = $PSScriptRoot
$Repo = (Resolve-Path (Join-Path $S "..\..\..\..")).Path
$Blender = "C:\Program Files\Blender Foundation\Blender 5.1\blender.exe"
$Py = "C:\Program Files\Blender Foundation\Blender 5.1\5.1\python\bin\python.exe"
$Rig = Join-Path $Repo "work\player\rig"
$Out = Join-Path $Repo "work\player\inspect\P2"
$LogDir = Join-Path $Rig "export\logs"
New-Item -ItemType Directory -Force -Path $Out, $LogDir | Out-Null
$Checkers = @("check_p2_static", "check_p2_deform", "check_p2_skel", "check_p2_skin", "check_p2_ctrl",
              "check_p2_export", "check_p2_skirt")

$Collector = @'
import hashlib, json, sys, os
from pathlib import Path

DROP = {"seconds", "timing_s", "timing_s_report"}

def strip(v):
    if isinstance(v, dict):
        return {k: strip(x) for k, x in v.items() if k not in DROP}
    if isinstance(v, list):
        return [strip(x) for x in v]
    return v

def h(b):
    return hashlib.sha256(b).hexdigest()

def fsha(p):
    with open(p, "rb") as f:
        return h(f.read())

def collect(out_json, run_n, steps_json, repo, out_dir, checkers):
    steps = json.loads(Path(steps_json).read_text(encoding="utf-8-sig"))
    res = []
    for c in checkers:
        ev = Path(out_dir) / f"{c}.json"
        st = next((s for s in steps.get("checkers", []) if s.get("name") == c), {})
        row = {"name": c, "exit": st.get("exit"), "wall_s": st.get("wall_s"), "evidence": str(ev)}
        if ev.exists():
            d = json.loads(ev.read_text(encoding="utf-8"))
            crit = d.get("criteria") or []
            row.update({"evidence_sha256": fsha(ev), "gate": d.get("gate"), "task": d.get("task"),
                        "rows": [{"id": r.get("id"), "ok": r.get("ok"),
                                  "measured_sha256": h(json.dumps(strip(r.get("measured")), sort_keys=True,
                                                                  ensure_ascii=False).encode("utf-8"))}
                                 for r in crit],
                        "n_ok": sum(1 for r in crit if r.get("ok") is True),
                        "n_false": sum(1 for r in crit if r.get("ok") is not True),
                        "false_rows": [r.get("id") for r in crit if r.get("ok") is not True],
                        "inputs": d.get("inputs") or [], "renders": d.get("renders") or []})
        else:
            row["evidence_sha256"] = None
            row["error"] = "evidence missing"
        res.append(row)
    doc = {"run": run_n, "repo": repo, "started": steps.get("started"), "finished": steps.get("finished"),
           "chain": steps.get("chain"), "chain_ok": steps.get("chain_ok"), "checkers": res,
           "summary": {c["name"]: [c.get("exit"), c.get("n_ok"), c.get("n_false")] for c in res}}
    Path(out_json).write_text(json.dumps(doc, indent=1, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(doc["summary"]))

def compare(a_json, b_json, out_json):
    a = json.loads(Path(a_json).read_text(encoding="utf-8"))
    b = json.loads(Path(b_json).read_text(encoding="utf-8"))
    diffs = {"chain_exit": [], "checker_exit": [], "rows": [], "inputs": [], "missing": []}
    ca = {s.get("name"): s for s in a.get("chain") or []}
    for s in b.get("chain") or []:
        if s.get("name") not in ca or ca[s["name"]].get("exit") != s.get("exit"):
            diffs["chain_exit"].append([s.get("name"), ca.get(s.get("name"), {}).get("exit"), s.get("exit")])
    ka = {c["name"]: c for c in a.get("checkers") or []}
    for c in b.get("checkers") or []:
        x = ka.get(c["name"])
        if x is None:
            diffs["missing"].append(c["name"])
            continue
        if x.get("exit") != c.get("exit"):
            diffs["checker_exit"].append([c["name"], x.get("exit"), c.get("exit")])
        ra = {r["id"]: r for r in x.get("rows") or []}
        for r in c.get("rows") or []:
            y = ra.get(r["id"])
            if y is None or y.get("ok") != r.get("ok") or y.get("measured_sha256") != r.get("measured_sha256"):
                diffs["rows"].append({"checker": c["name"], "id": r["id"], "ok": [None if y is None else y.get("ok"), r.get("ok")],
                                      "measured_equal": y is not None and y.get("measured_sha256") == r.get("measured_sha256")})
        ia = {i["path"]: i.get("sha256") for i in x.get("inputs") or []}
        for i in c.get("inputs") or []:
            if ia.get(i["path"]) != i.get("sha256"):
                diffs["inputs"].append({"checker": c["name"], "path": i["path"], "a": ia.get(i["path"]), "b": i.get("sha256")})
    same = not any(diffs.values())
    doc = {"a": a_json, "b": b_json, "identical": same,
           "n_rows_compared": sum(len(c.get("rows") or []) for c in b.get("checkers") or []),
           "n_inputs_compared": sum(len(c.get("inputs") or []) for c in b.get("checkers") or []), "differences": diffs}
    Path(out_json).write_text(json.dumps(doc, indent=1, ensure_ascii=False), encoding="utf-8")
    print(json.dumps({"identical": same, **{k: len(v) for k, v in diffs.items()}}))
    return 0 if same else 1

if __name__ == "__main__":
    if sys.argv[1] == "collect":
        collect(sys.argv[2], int(sys.argv[3]), sys.argv[4], sys.argv[5], sys.argv[6], sys.argv[7].split(","))
    else:
        sys.exit(compare(sys.argv[2], sys.argv[3], sys.argv[4]))
'@
$CollectorPy = Join-Path $env:TEMP ("run_player_gates_collector_{0}.py" -f $PID)
Set-Content -Path $CollectorPy -Value $Collector -Encoding utf8

function Invoke-Py([string[]]$PyArgs) {
    & $Py -B $CollectorPy @PyArgs | Out-Host
    return $LASTEXITCODE
}

try {
    if ($Compare) {
        if ($A -le 0 -or $B -le 0) { throw "-Compare needs -A <n> -B <n>" }
        $ja = Join-Path $Out "gates_run_$A.json"
        $jb = Join-Path $Out "gates_run_$B.json"
        foreach ($j in @($ja, $jb)) { if (-not (Test-Path $j)) { throw "missing $j" } }
        $code = Invoke-Py @("compare", $ja, $jb, (Join-Path $Out "gates_compare_${A}_${B}.json"))
        Write-Output "[run_player_gates] compare $A vs $B exit $code"
        exit $code
    }
    if ($Run -le 0) {
        $nums = @(Get-ChildItem -Path $Out -Filter "gates_run_*.json" -ErrorAction SilentlyContinue |
                  ForEach-Object { if ($_.BaseName -match '^gates_run_(\d+)$') { [int]$Matches[1] } })
        $Run = 1 + (($nums + 0) | Measure-Object -Maximum).Maximum
    }
    $started = Get-Date -Format "yyyy-MM-ddTHH:mm:ss"

    function Invoke-BlenderStep([string]$Name, [string[]]$StepArgs) {
        $log = Join-Path $LogDir "gates_$Name.log"
        $sw = [System.Diagnostics.Stopwatch]::StartNew()
        $p = Start-Process -FilePath $Blender -ArgumentList $StepArgs -PassThru -NoNewWindow `
            -RedirectStandardOutput $log -RedirectStandardError "$log.err"
        $null = $p.Handle
        $p.WaitForExit()
        $sw.Stop()
        $r = [ordered]@{ name = $Name; exit = $p.ExitCode; wall_s = [math]::Round($sw.Elapsed.TotalSeconds, 1); log = $log }
        Write-Host ("[run_player_gates] {0} exit {1} wall {2} s" -f $Name, $r.exit, $r.wall_s)
        return $r
    }

    $chain = @()
    $chainOk = $true
    if (-not $SkipChain) {
        $prod = Get-ChildItem -Path $S -Filter "p2*.py" | Where-Object { $_.Name -match '^p2[0-4]' } | Sort-Object Name
        foreach ($f in $prod) {
            $cli = Select-String -Path $f.FullName -Pattern '^CLI:\s+blender\s+(.*)$' | Select-Object -First 1
            if ($null -eq $cli) { throw "no CLI: line in $($f.Name)" }
            $blend = $null
            if ($cli.Matches[0].Groups[1].Value -match '--factory-startup\s+(\S+\.blend)\s+--python') { $blend = $Matches[1] }
            $argList = @("--background", "--factory-startup")
            if ($blend) { $argList += (Join-Path $Repo $blend) }
            $argList += @("--python", $f.FullName)
            $r = Invoke-BlenderStep $f.BaseName $argList
            $r["blend"] = $blend
            $chain += $r
            if ($r.exit -ne 0) { $chainOk = $false; break }
        }
        if ($chainOk) {
            $sw = [System.Diagnostics.Stopwatch]::StartNew()
            & powershell -NoProfile -File (Join-Path $S "run_player_p2d.ps1") | Out-Host
            $code = $LASTEXITCODE
            $sw.Stop()
            $chain += [ordered]@{ name = "run_player_p2d"; exit = $code; wall_s = [math]::Round($sw.Elapsed.TotalSeconds, 1) }
            Write-Output ("[run_player_gates] run_player_p2d exit {0}" -f $code)
            if ($code -ne 0) { $chainOk = $false }
        }
    }
    $checks = @()
    if ($chainOk -or $ContinueOnChainFailure) {
        foreach ($c in $Checkers) {
            $checks += Invoke-BlenderStep $c @("--background", "--factory-startup", "--python", (Join-Path $S "$c.py"))
        }
    } else {
        Write-Output "[run_player_gates] chain failed: checkers skipped (use -ContinueOnChainFailure to run them)"
    }
    $steps = [ordered]@{ started = $started; finished = (Get-Date -Format "yyyy-MM-ddTHH:mm:ss"); chain_ok = $chainOk;
                         skip_chain = [bool]$SkipChain; chain = $chain; checkers = $checks }
    $stepsJson = Join-Path $env:TEMP ("run_player_gates_steps_{0}.json" -f $PID)
    $steps | ConvertTo-Json -Depth 6 | Set-Content -Path $stepsJson -Encoding utf8
    $outJson = Join-Path $Out "gates_run_$Run.json"
    $code = Invoke-Py @("collect", $outJson, "$Run", $stepsJson, $Repo, $Out, ($Checkers -join ","))
    Remove-Item $stepsJson -ErrorAction SilentlyContinue
    Write-Output "[run_player_gates] wrote $outJson (collector exit $code)"
    if (-not $chainOk) { exit 1 }
    exit 0
} finally {
    Remove-Item $CollectorPy -ErrorAction SilentlyContinue
}
