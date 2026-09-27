import bpy, json, math
from mathutils import Vector, Matrix
from mathutils.kdtree import KDTree

ob = bpy.data.objects["GOB_body"]
V = [v.co.copy() for v in ob.data.vertices]
kd = KDTree(len(V))
for i, p in enumerate(V):
    kd.insert(p, i)
kd.balance()

out = {}


def section(center, d, slab, rad):
    """verts within `rad` of center and within +-slab along d; return centroid + radius"""
    d = d.normalized()
    pts = [V[i] for (co, i, dist) in kd.find_range(center, rad)
           if abs((V[i] - center).dot(d)) <= slab]
    if len(pts) < 8:
        return None
    c = Vector((0, 0, 0))
    for p in pts:
        c += p
    c /= len(pts)
    # radius in plane perpendicular to d
    rr = []
    for p in pts:
        v = p - c
        v -= d * v.dot(d)
        rr.append(v.length)
    rr.sort()
    return {"c": c, "n": len(pts), "rmean": sum(rr) / len(rr),
            "rmax": rr[-1], "rp90": rr[int(len(rr) * 0.9)]}


def trace(name, seed, direction, step=0.025, slab=0.012, rad=0.30, nsteps=40,
          stop_r=0.22):
    d = Vector(direction).normalized()
    c = Vector(seed)
    rows = []
    for i in range(nsteps):
        s = section(c, d, slab, rad)
        if s is None:
            rows.append({"i": i, "stop": "empty"})
            break
        rows.append({"i": i, "c": [round(x, 4) for x in s["c"]],
                     "rmean": round(s["rmean"], 4), "rp90": round(s["rp90"], 4),
                     "n": s["n"], "dir": [round(x, 3) for x in d]})
        if s["rp90"] > stop_r:
            rows[-1]["stop"] = "fat"
            break
        newc = s["c"] + d * step
        s2 = section(newc, d, slab, rad)
        if s2 is not None:
            nd = (s2["c"] - s["c"])
            nd -= Vector((0, 0, 0))
            if nd.length > 1e-5:
                nd.normalize()
                d = (d * 0.55 + nd * 0.45).normalized()
        c = s["c"] + d * step
    out[name] = rows
    return rows


def blob(center, rad, iters=8):
    c = Vector(center)
    for _ in range(iters):
        pts = [V[i] for (co, i, dist) in kd.find_range(c, rad)]
        if not pts:
            return None
        nc = Vector((0, 0, 0))
        for p in pts:
            nc += p
        nc /= len(pts)
        if (nc - c).length < 1e-5:
            c = nc
            break
        c = nc
    return {"c": [round(x, 4) for x in c], "n": len(pts)}


def pca_line(pts):
    n = len(pts)
    c = Vector((0, 0, 0))
    for p in pts:
        c += p
    c /= n
    M = [[0.0] * 3 for _ in range(3)]
    for p in pts:
        v = p - c
        for a in range(3):
            for b in range(3):
                M[a][b] += v[a] * v[b]
    m = Matrix(M)
    # power iteration for dominant eigenvector
    d = Vector((0, 0, 1))
    for _ in range(200):
        d = (m @ d)
        if d.length < 1e-12:
            break
        d.normalize()
    ts = [(p - c).dot(d) for p in pts]
    return {"center": [round(x, 4) for x in c], "dir": [round(x, 4) for x in d],
            "t_min": round(min(ts), 4), "t_max": round(max(ts), 4),
            "p_min": [round(x, 4) for x in (c + d * min(ts))],
            "p_max": [round(x, 4) for x in (c + d * max(ts))], "n": n}


# ---------- club shaft: clearly isolated region ----------
shaft = [p for p in V if p.x < -0.55 and p.y < -0.25 and 0.68 < p.z < 1.30]
out["club_shaft_pca"] = pca_line(shaft)
club_all = [p for p in V if p.x < -0.55 and p.y < -0.26 and p.z > 0.60]
out["club_all_pca"] = pca_line(club_all)
out["club_all_bbox"] = {
    "min": [round(min(p[i] for p in club_all), 4) for i in range(3)],
    "max": [round(max(p[i] for p in club_all), 4) for i in range(3)],
    "n": len(club_all)}
# shaft bottom
sb = [p for p in V if p.x < -0.55 and p.y < -0.25 and p.z < 0.70]
out["shaft_bottom"] = {"n": len(sb),
                       "zmin": round(min(p.z for p in sb), 4) if sb else None,
                       "c": [round(sum(p[i] for p in sb) / len(sb), 4) for i in range(3)] if sb else None}

# ---------- hand blobs ----------
out["blob_R_hand"] = blob((-0.70, -0.20, 1.00), 0.13)
out["blob_L_hand"] = blob((0.78, 0.05, 0.73), 0.13)

# ---------- arms ----------
trace("armR_from_hand", (-0.70, -0.20, 1.02), (0.55, 0.35, 0.45), step=0.022,
      rad=0.16, stop_r=0.16, nsteps=30)
trace("armL_from_hand", (0.80, 0.05, 0.75), (-0.5, -0.1, 0.85), step=0.022,
      rad=0.16, stop_r=0.16, nsteps=30)

# ---------- legs ----------
trace("legR_up", (-0.30, 0.10, 0.28), (0, 0, 1), step=0.02, rad=0.13,
      stop_r=0.16, nsteps=20)
trace("legL_up", (0.30, 0.10, 0.28), (0, 0, 1), step=0.02, rad=0.13,
      stop_r=0.16, nsteps=20)

print("@@@JSON_START@@@")
print(json.dumps(out, indent=1))
print("@@@JSON_END@@@")
