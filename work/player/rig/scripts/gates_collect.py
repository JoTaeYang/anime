"""gates_collect - collector and compare for run_player.py gates (T340: ported verbatim from the Python that was
embedded in run_player_gates.ps1, T254; only the file writes changed to newline="").

collect: for every checker evidence JSON: sha256 of the file, every row id / ok / sha256 of its measured value (timing
keys "seconds", "timing_s", "timing_s_report" removed before hashing), the evidence inputs [path, sha256] and renders.
compare: rows (ok, measured hash), inputs (sha256), exit codes; exit 0 when identical, 1 when anything differs.

CLI:  python gates_collect.py collect <out_json> <run_n> <steps_json> <repo> <out_dir> <c1,c2,...>
      python gates_collect.py compare <a_json> <b_json> <out_json>
"""
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

def _write(path, text):
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write(text)

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
    _write(out_json, json.dumps(doc, indent=1, ensure_ascii=False))
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
    _write(out_json, json.dumps(doc, indent=1, ensure_ascii=False))
    print(json.dumps({"identical": same, **{k: len(v) for k, v in diffs.items()}}))
    return 0 if same else 1

if __name__ == "__main__":
    if sys.argv[1] == "collect":
        collect(sys.argv[2], int(sys.argv[3]), sys.argv[4], sys.argv[5], sys.argv[6], sys.argv[7].split(","))
    else:
        sys.exit(compare(sys.argv[2], sys.argv[3], sys.argv[4]))
