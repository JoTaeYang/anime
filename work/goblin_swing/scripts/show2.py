import json, sys
raw = open(r"C:\Users\whxod\orca\anime\work\goblin_swing\inspect\step02_analyze2.json").read()
d = json.loads(raw.split("@@@JSON_START@@@")[1].split("@@@JSON_END@@@")[0])
for key in sys.argv[1:]:
    s = d[key]
    if "rows" not in s:
        print("==", key, json.dumps(s)); continue
    print("==", key, "|", s["label"], "dir", s["dir"])
    for r in s["rows"]:
        e = r["ext"]
        print("t=%7.3f n=%-5d c=(%7.3f,%7.3f,%7.3f) r=%.3f/%.3f  X[%6.3f,%6.3f] Y[%6.3f,%6.3f] Z[%6.3f,%6.3f]" % (
            r["t"], r["n"], r["c"][0], r["c"][1], r["c"][2], r["rmean"], r["rp90"],
            e[0][0], e[0][1], e[1][0], e[1][1], e[2][0], e[2][1]))
