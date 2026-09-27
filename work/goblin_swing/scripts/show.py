import json, sys
raw = open(r"C:\Users\whxod\orca\anime\work\goblin_swing\inspect\step02_analyze.json").read()
d = json.loads(raw.split("@@@JSON_START@@@")[1].split("@@@JSON_END@@@")[0])
key = sys.argv[1]
s = d[key]
print(s["label"], "| slice axis", s["axis"])
for r in s["rows"]:
    parts = []
    for c in r["cl"]:
        parts.append("n=%-5d c=(%7.3f,%7.3f,%7.3f) r=%.3f" % (c["n"], c["c"][0], c["c"][1], c["c"][2], c["r"]))
    print("%7.3f  N=%-6d %s" % (r["t"], r["n"], "   ||   ".join(parts)))
