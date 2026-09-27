"""Per-slice extents of a point cloud, used to align the new T-pose goblin to the old mesh.

Below the shoulders and above the neck there are no arms, so a plain min/max x per
z-slice is a faithful body profile there.  The shoulder band is contaminated by the
arms (and the club) and is simply not used for landmarks.
"""


def profile(P, bins=200):
    zs = [p[2] for p in P]
    zmin, zmax = min(zs), max(zs)
    dz = (zmax - zmin) / bins
    sl = [[] for _ in range(bins)]
    for p in P:
        i = int((p[2] - zmin) / dz)
        sl[min(max(i, 0), bins - 1)].append(p)
    out = []
    for i, s in enumerate(sl):
        z = zmin + (i + 0.5) * dz
        if not s:
            out.append({"z": z, "n": 0, "xmin": 0, "xmax": 0, "ymin": 0, "ymax": 0, "w": 0})
            continue
        xs = [p[0] for p in s]
        ys = [p[1] for p in s]
        out.append({"z": z, "n": len(s), "xmin": min(xs), "xmax": max(xs),
                    "ymin": min(ys), "ymax": max(ys), "w": max(xs) - min(xs)})
    return out, zmin, zmax


def chart(prof, zmin, zmax, label=""):
    H = zmax - zmin
    wmax = max(s["w"] for s in prof) or 1.0
    lines = ["%s  H=%.4f  Wmax=%.4f" % (label, H, wmax)]
    for i in range(len(prof) - 1, -1, -4):
        s = prof[i]
        f = (s["z"] - zmin) / H
        bar = "#" * int(round(60 * s["w"] / wmax))
        lines.append("f=%.3f z=%+.4f w=%.4f d=%.4f |%s" %
                     (f, s["z"], s["w"], s["ymax"] - s["ymin"], bar))
    return "\n".join(lines)
