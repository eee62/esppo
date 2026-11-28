"""Step 1b: scale the decimated reference 2.79x and place it on the fixed internals.

Mapping (reference-local units -> shell mm):  shell_x = -(ref_y - CY) * S,  shell_y = (ref_x - CX) * S,  shell_z = ref_z * S
(the reference nose, +x, becomes shell +y; its +y becomes shell -x). CY = 14.75 puts the reference's centre key seam
(measured from the top height map: y 14.75 in front of the wheel) on shell x = 0, so the left key covers SW_L1 and
the right key SW_R1, with the real wheel (x = 0) in the seam. CX (along the length) and a small seam shift are
searched for the largest clearance of the internals inside the scaled surface (BVH nearest-surface depth, sampled
points on the cell, board, components, switches and screw columns)."""
import sys
sys.path.insert(0, "/home/etanpong/mouse-shell/shell/blender")
import importlib
import json

import bpy
import numpy as np
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree

import lib
importlib.reload(lib)

S = 2.79
CY0 = 14.75
NEED = 2.0          # cell / board / components: 1.5 wall + 0.5 clearance below the outer skin
NEED_SCREW = 2.55   # screw pilot axis: 1.05 pilot radius + 1.5 wall

I = lib.internals()


def box_pts(b, step):
    x0, x1, y0, y1, z0, z1 = b
    out = []
    for xs in (np.arange(x0, x1 + 1e-9, step).tolist() + [x1],):
        pass
    gx = np.unique(np.append(np.arange(x0, x1, step), x1))
    gy = np.unique(np.append(np.arange(y0, y1, step), y1))
    gz = np.unique(np.append(np.arange(z0, z1, step), z1))
    for X in (x0, x1):
        Y, Z = np.meshgrid(gy, gz); out.append(np.column_stack([np.full(Y.size, X), Y.ravel(), Z.ravel()]))
    for Y_ in (y0, y1):
        X, Z = np.meshgrid(gx, gz); out.append(np.column_stack([X.ravel(), np.full(X.size, Y_), Z.ravel()]))
    for Z_ in (z0, z1):
        X, Y = np.meshgrid(gx, gy); out.append(np.column_stack([X.ravel(), Y.ravel(), np.full(X.size, Z_)]))
    return np.vstack(out)


def outer_only(P, b, d=6.0):
    """keep points within d of the box's lateral faces or its top (interior points never limit)."""
    x0, x1, y0, y1, z0, z1 = b
    m = (P[:, 0] < x0 + d) | (P[:, 0] > x1 - d) | (P[:, 1] < y0 + d) | (P[:, 1] > y1 - d) | (P[:, 2] > z1 - 0.01) | (P[:, 2] < z0 + 0.01)
    return P[m]


pts, need, label = [], [], []
def add(P, n, lab):
    pts.append(P); need.append(np.full(len(P), n)); label.extend([lab] * len(P))

add(outer_only(box_pts(I["battery"]["box"], 1.0), I["battery"]["box"]), NEED, "battery")
add(outer_only(box_pts(I["board"]["box"], 1.0), I["board"]["box"]), NEED, "board")
for c in I["components"]:
    if c["box"][5] > I["board"]["box"][5] + 0.05:
        add(box_pts(c["box"], 1.0)[lambda q: q[:, 2] > c["box"][4] + 0.01] if False else box_pts(c["box"], 1.0), NEED,
            "tallest" if c["ref"] == I["tallest"]["ref"] else "component")
for k, sw in I["switches"].items():
    x, y = sw["xy"]
    add(box_pts([x - 3, x + 3, y - 3, y + 3, I["board"]["box"][5], sw["top_z"]], 1.0), NEED, "switch")
zb = I["board"]["box"][4] - I["screws"]["board"]["head_k"]       # H3 / H4: pan head under the board
scr = [(*I["holes"][h], 0.5, I["screws"]["front"]["pilot_top_z"]) for h in ("H1", "H2")] + \
      [(*I["rear_cover_screws"][s], 0.5, I["screws"]["rear_cover"]["pilot_top_z"]) for s in ("L", "R")] + \
      [(*I["holes"][h], zb, I["screws"]["board"]["pilot_top_z"]) for h in ("H3", "H4")]
for x, y, z0, zt in scr:
    zz = np.arange(z0, zt + 1e-9, 0.5)
    add(np.column_stack([np.full(len(zz), x), np.full(len(zz), y), zz]), NEED_SCREW, "screw")
P = np.vstack(pts)
NEEDV = np.concatenate(need)
LAB = np.array(label)

dec = lib.obj("ref_dec")
bvh = BVHTree.FromObject(dec, bpy.context.evaluated_depsgraph_get())


SKIRT_Z = 6.0       # shell z: below it the surface becomes the straight skirt (step 3)


def footprint():
    """Plan outline (reference units) of everything below the skirt top: polar maximum of the full-resolution
    vertices below SKIRT_Z / S, 0.25 deg bins about the plan centre, lightly smoothed (closed polygon)."""
    v = lib.verts(lib.obj("ref_raw"))
    v = v[v[:, 2] <= SKIRT_Z / S]
    c = np.array([25.0, 16.0])
    a = np.arctan2(v[:, 1] - c[1], v[:, 0] - c[0])
    r = np.hypot(v[:, 0] - c[0], v[:, 1] - c[1])
    nb = 1440
    b = ((a + np.pi) / (2 * np.pi) * nb).astype(int) % nb
    R = np.zeros(nb)
    np.maximum.at(R, b, r)
    for _ in range(3):                                   # fill empty bins, then a 3-bin max filter (envelope)
        R = np.maximum(R, np.maximum(np.roll(R, 1), np.roll(R, -1)) * (R == 0))
    R = np.maximum.reduce([np.roll(R, k) for k in (-1, 0, 1)])
    t = (np.arange(nb) + 0.5) / nb * 2 * np.pi - np.pi
    return c + np.column_stack([R * np.cos(t), R * np.sin(t)])


FOOT = footprint()


def foot_depth(Q2):
    """Signed 2D distance (reference units, positive inside) of points to the footprint polygon."""
    A, B = FOOT, np.roll(FOOT, -1, axis=0)
    AB = B - A
    L2 = (AB ** 2).sum(1)
    out = np.empty(len(Q2))
    for k in range(0, len(Q2), 500):
        q = Q2[k:k + 500, None, :]
        t = np.clip(((q - A) * AB).sum(2) / L2, 0, 1)
        d = np.sqrt((((A + t[..., None] * AB) - q) ** 2).sum(2)).min(1)
        # inside test: even-odd crossings of a +x ray
        y = q[..., 1]; x = q[..., 0]
        cr = ((A[:, 1] > y) != (B[:, 1] > y)) & (x < A[:, 0] + (y - A[:, 1]) * AB[:, 0] / np.where(AB[:, 1] == 0, 1e-12, AB[:, 1]))
        ins = cr.sum(1) % 2 == 1
        out[k:k + 500] = np.where(ins, d, -d)
    return out


def depth(cx, cy, Q=P):
    """Depth (mm, positive inside) of shell points below the scaled reference surface; below SKIRT_Z the skirt
    (footprint extruded straight down) is added: union -> the larger depth."""
    out = np.full(len(Q), -99.0)
    for i in np.flatnonzero(Q[:, 2] >= SKIRT_Z):
        x, y, z = Q[i]
        r = Vector((cx + y / S, cy - x / S, z / S))
        co, n, _, dist = bvh.find_nearest(r)
        s = -1.0 if (r - co).dot(n) > 0 else 1.0
        out[i] = s * dist * S
    lowm = Q[:, 2] < SKIRT_Z
    if lowm.any():
        q2 = np.column_stack([cx + Q[lowm, 1] / S, cy - Q[lowm, 0] / S])
        out[lowm] = foot_depth(q2) * S       # straight skirt: the footprint is the outer surface
    return out


def score(cx, cy):
    d = depth(cx, cy) - NEEDV
    return float(d.min()), d


# The keys only need to stay over their switches: the reference's centre seam may sit anywhere between
# SW_L1 + 4 and SW_R1 - 4 (shell x -17 ... 10.6); CY is searched over that range (coarse, then refined).
SEAM_X = (-17.0, 10.6)
cy_lo, cy_hi = CY0 - SEAM_X[1] / S, CY0 - SEAM_X[0] / S
best = None
for cx in np.arange(20.0, 28.01, 0.5):
    for cy in np.arange(cy_lo, cy_hi + 1e-9, 0.4):
        m, _ = score(cx, cy)
        if best is None or m > best[0]:
            best = (m, cx, cy)
coarse = best
for cx in np.arange(best[1] - 0.5, best[1] + 0.51, 0.1):
    for cy in np.arange(max(cy_lo, best[2] - 0.4), min(cy_hi, best[2] + 0.4) + 1e-9, 0.08):
        m, _ = score(cx, cy)
        if m > best[0] + 1e-3:
            best = (m, cx, cy)
m, cx, cy = best
_, d = score(cx, cy)
worst = {}
for lab in np.unique(LAB):
    k = np.flatnonzero(LAB == lab)
    i = k[np.argmin(d[k])]
    worst[lab] = dict(margin=round(float(d[i]), 3), at=np.round(P[i], 2).tolist(), need=float(NEEDV[i]))

# the placed decimated mesh (shell coordinates) and the raw reference placed the same way (overlay)
M = Matrix.Translation((0, 0, 0)) @ Matrix(((0, -S, 0, cy * S), (S, 0, 0, -cx * S), (0, 0, S, 0), (0, 0, 0, 1)))
for src, name in (("ref_dec", "ref_placed"), ("ref_raw", "ref_overlay")):
    lib.remove(name)
    o = lib.obj(src)
    c = o.copy(); c.data = o.data.copy(); c.name = c.data.name = name
    lib.collection("ref").objects.link(c)
    c.data.transform(M)
    c.matrix_world = Matrix.Identity(4)
lib.obj("ref_overlay").hide_set(True)
v = lib.verts(lib.obj("ref_placed"))
wheel_ref = (37.8, 14.75)
res = dict(scale=S, CX=round(cx, 3), CY=round(cy, 3), seam_x_mm=round(-(CY0 - cy) * S, 3),
           matrix=[list(r) for r in M], min_margin=round(m, 3), coarse=dict(margin=round(coarse[0], 3), CX=coarse[1], CY=round(coarse[2], 3)),
           worst_by_part=worst, bbox=dict(lo=v.min(0).round(3).tolist(), hi=v.max(0).round(3).tolist(),
                                          size=(v.max(0) - v.min(0)).round(3).tolist()),
           ref_wheel_in_shell=[round(-(wheel_ref[1] - cy) * S, 2), round((wheel_ref[0] - cx) * S, 2)],
           ref_key_seam_end_y=round((21.5 - cx) * S, 2))
with open(lib.HERE + "/placement.json", "w") as f:
    json.dump(res, f, indent=1)
np.save(lib.OUT + "/footprint_ref.npy", FOOT)
print(json.dumps(res, indent=1))
