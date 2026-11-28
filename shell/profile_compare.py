#!/usr/bin/env python3
"""Old (previous restyle) vs new outer skin: top-view outline, centreline side profile, and cross-sections through
the middle (y = 0) and the rear (y = -40), with the board and cell outlines. Writes renders/profiles_old_vs_new.png
and prints the dimension table as JSON. Each skin is built in its own process (parameters are module-level)."""
import json, os, subprocess, sys
HERE = os.path.dirname(os.path.abspath(__file__))
OLD = dict(H_BASE=14.0, SIDE=35.03, W_REAR=78.0, NOSE_Y=70.0, NOSE_C=50.0, WIDEN_Y=22.0, REAR_C=-25.0, REAR_Y=-60.0, REAR_N=3.0,
           Z_PEAK=36.0, PEAK_Y=-4.0, DOME_N=2.5, DOME_N_END=2.5, DOME_BLEND=1.0, NOSE_N=2.0, TUCK_FRONT=1.5, TUCK_REAR=3.5,
           DOME_MODE="radial")
CHILD = r"""
import sys, json; sys.path.insert(0, %r)
import shell_model as m, numpy as np, trimesh
ob = m.skin_body(0.0, 0.0)
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.TopLoc import TopLoc_Location
BRepMesh_IncrementalMesh(ob.wrapped, 0.01, False, 0.5, True)       # 0.01 mm absolute chord sag
V, F, off = [], [], 0
for f in ob.Faces():
    loc = TopLoc_Location(); tri = BRep_Tool.Triangulation_s(f.wrapped, loc)
    if tri is None: continue
    tr = loc.Transformation(); V += [tri.Node(i).Transformed(tr).Coord() for i in range(1, tri.NbNodes() + 1)]
    F += [tuple(off + k - 1 for k in tri.Triangle(i).Get()) for i in range(1, tri.NbTriangles() + 1)]; off += tri.NbNodes()
mesh = trimesh.Trimesh(np.array(V), np.array(F), process=True)
def sec(pnt, nrm):
    seg = trimesh.intersections.mesh_plane(mesh, nrm, pnt)
    return [s.tolist() for s in seg]
def width(z):
    seg = np.array(sec((0, 0, z), (0, 0, 1))).reshape(-1, 3)
    return round(2 * float(np.abs(seg[:, 0]).max()), 2), round(2 * float(np.abs(seg[seg[:, 1] > -7][:, 0]).max()), 2)
out = dict(plan=sec((0, 0, m.p.H_BASE), (0, 0, 1)), side=sec((0, 0, 0), (1, 0, 0)), mid=sec((0, 0, 0), (0, 1, 0)),
           rear=sec((0, -40, 0), (0, 1, 0)), widest_z=m.p.H_BASE,
           width={str(z): width(z) for z in (2.0, m.p.H_BASE, 15.1, 20.0, 25.0)},
           length=round(float(mesh.bounds[1][1] - mesh.bounds[0][1]), 2), skin_top=round(float(mesh.bounds[1][2]), 2),
           peak_y=m.p.PEAK_Y, peak_pct=round((m.p.NOSE_Y - m.p.PEAK_Y) / (m.p.NOSE_Y - m.p.REAR_Y) * 100, 1),
           front_drop_at_wheel=round(m.p.Z_PEAK - float(m.z_skin(0.0, m.WHEEL_Y)), 2),
           wheel_proud=round(m.WHEEL_Z + m.p.WHEEL_D / 2 - float(m.z_skin(0.0, m.WHEEL_Y)), 2))
print("JSON" + json.dumps(out))
""" % HERE


def run(params):
    env = dict(os.environ, SHELL_NO_CACHE="1", SHELL_PARAMS=json.dumps(params) if params else "")
    r = subprocess.run([sys.executable, "-c", CHILD], env=env, capture_output=True, text=True, cwd=HERE)
    line = [l for l in r.stdout.splitlines() if l.startswith("JSON")]
    if not line:
        raise RuntimeError(r.stderr[-2000:])
    return json.loads(line[0][4:])


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    old, new = run(OLD), run(None)
    fig, ax = plt.subplots(2, 2, figsize=(16, 12))
    def draw(a, data, i, j, style, label):
        from matplotlib.collections import LineCollection
        segs = [[(q[i], q[j]) for q in e] for e in data]
        lc = LineCollection(segs, linestyles="dashed" if style == "--" else "solid", colors="0.55" if style == "--" else "k",
                            linewidths=1.6, label=label)
        a.add_collection(lc); a.autoscale_view()
    for d, st, lb in ((old, "--", "previous restyle"), (new, "-", "new")):
        c = "0.55" if st == "--" else "k"
        draw(ax[0, 0], d["plan"], 0, 1, st, lb); draw(ax[0, 1], d["side"], 1, 2, st, lb)
        draw(ax[1, 0], d["mid"], 0, 2, st, lb); draw(ax[1, 1], d["rear"], 0, 2, st, lb)
    from matplotlib.patches import Rectangle
    ax[0, 0].add_patch(Rectangle((-27.5, -7), 60, 58, fill=False, ec="tab:green", lw=1, label="board"))
    ax[0, 0].add_patch(Rectangle((-30, -46.7), 60, 90, fill=False, ec="tab:orange", lw=1, ls=":", label="cell"))
    for a, y in ((ax[1, 0], 0.0), (ax[1, 1], -40.0)):
        if y > -7: a.add_patch(Rectangle((-27.5, 13.5), 60, 1.6, color="tab:green", alpha=0.6, label="board"))
        a.add_patch(Rectangle((-30, 3.5), 60, 6, fill=False, ec="tab:orange", lw=1, label="cell"))
        a.axhline(new["widest_z"], color="tab:blue", lw=0.8, ls=":", label="new widest line z %.0f" % new["widest_z"])
    ax[0, 0].set_title("Top view outline at the widest line (front up)"); ax[0, 1].set_title("Side profile on the centreline (front right)")
    ax[1, 0].set_title("Cross-section through the middle (y = 0), seen from the front")
    ax[1, 1].set_title("Cross-section through the rear (y = -40), seen from the front")
    for a in ax.ravel():
        a.set_aspect("equal"); a.grid(alpha=0.3); a.legend(fontsize=8, loc="lower center")
    fig.tight_layout()
    path = os.path.join(HERE, "renders", "profiles_old_vs_new.png")
    fig.savefig(path, dpi=110)
    print("wrote", path)
    keys = ("width", "length", "skin_top", "peak_y", "peak_pct", "front_drop_at_wheel", "wheel_proud", "widest_z")
    print(json.dumps({"old": {k: old[k] for k in keys}, "new": {k: new[k] for k in keys}}, indent=1))


if __name__ == "__main__":
    main()
