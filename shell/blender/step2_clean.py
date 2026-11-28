"""Step 2 (+ the outer-surface part of step 3): the cleaned outer surface `outer_clean`, from the placed, decimated
reference (`ref_placed`), with Blender's geometry-node SDF grid nodes (OpenVDB level sets, voxel VOXEL mm).

  1. well fill: the reference's own wheel, its well and the small button behind it (the reference's WELL_BOX, as
     in ref_skin.py), plus a strip along its centre key seam, are replaced by a biharmonic patch of the top heights
     around them: a cap solid above the patch is subtracted and a plug solid below it is added;
  2. skirt: the plan outline of everything below SKIRT_Z (polar envelope, 0.25 deg) as a prism from z -6 to SKIRT_Z,
     its top edge rolled inward on SKIRT_ROLL; added (union). It covers the sensor hole and the tuck under the lip;
  3. closing (CLOSE_R): seams, side lines, engraved lettering, side texture, and the crease above the skirt;
  4. bowl around the real wheel: floor WHEEL_PROUD below the tyre top, flat elliptical floor over the tyre,
     parabolic walls (elliptical contours) at the sides and rear, open at the front (removes the ridge in front of
     the wheel);
  5. opening (OPEN_R): embossed marks; every convex edge (bowl rim included) rounded to >= OPEN_R; then a light
     SDF mean filter (residual dips over the closed seams);
  6. bottom cut at z = 0 (plane bisect, flat floor face).
"""
import sys
sys.path.insert(0, "/home/etanpong/mouse-shell/shell/blender")
import importlib
import json
import time

import bmesh
import bpy
import numpy as np

import lib
importlib.reload(lib)

VOXEL = 0.2
BAND = 18
CLOSE_R = 3.0          # fills seams / lettering / texture (grooves up to 6 mm wide); concave creases >= 3 mm
MEAN_W, MEAN_IT = 3, 4  # final SDF mean filter (3-voxel box, 4 passes ~ 1 mm Gaussian): residual dips over closed seams
OPEN_R = 3.0           # convex edges >= 3.0 mm (brief: >= 2.1, bowl rim 3)
SKIRT_Z = 6.0
SKIRT_ROLL = 3.0
WHEEL_PROUD = 3.0
BOWL_A, BOWL_B = 6.5, 9.5       # elliptical flat floor: semi-axes across (x) / along (y) the wheel
BOWL_KX, BOWL_KY = 0.25, 0.12   # wall rise KX ex^2 + KY ey^2, (ex, ey) = radial excess beyond the floor ellipse (mm)
WELL_BOX_REF = (28.3, 44.4, 11.2, 18.2)   # reference units (ref_skin.py): its wheel, well and small button
SEAM_REF_Y = 14.75                        # reference centre key seam
SEAM_HW = 3.2                             # half-width of the seam strip in the fill (mm): covers the seam at x 2.9
                                          # (behind the wheel) and x 4.1 (in front), centred between them

t0 = time.time()
I = lib.internals()
pl = lib.placement()
S, CX, CY = pl["scale"], pl["CX"], pl["CY"]
ref = lib.obj("ref_placed")
rep = {}

# ---------------------------------------------------------------- 1. well fill (biharmonic patch)
x0, x1, y0, y1 = WELL_BOX_REF
bx = sorted([-(y0 - CY) * S, -(y1 - CY) * S])
by = [(x0 - CX) * S, (x1 - CX) * S]
seam_x = -(SEAM_REF_Y - CY) * S - 0.6        # 3.5: between the rear (2.9) and front (4.1) seam lines
H = 0.4
xs = np.arange(bx[0] - 8, bx[1] + 8 + 1e-9, H)
ys = np.arange(-12.0, 69.0 + 1e-9, H)          # whole seam: it ends near y -7 behind, runs to the nose in front
Z = lib.topmap(ref, xs, ys)
X, Y = np.meshgrid(xs, ys, indexing="ij")
# rounded-rectangle mask (corner radius 3) over the box + 1 mm, and the seam strip over the whole grid
m = 1.0
dx = np.maximum(np.maximum(bx[0] - m - X, X - bx[1] - m) + 3, 0)
dy = np.maximum(np.maximum(by[0] - m - Y, Y - by[1] - m) + 3, 0)
box_mask = np.hypot(dx, dy) <= 3
mask = box_mask | (np.abs(X - seam_x) <= SEAM_HW)
mask[[0, 1, -2, -1], :] = False          # outer two rows / columns stay known (the solve's boundary)
assert np.isfinite(Z[~mask]).all(), "top map misses outside the fill mask"
Zf, iters = lib.biharmonic_fill(np.nan_to_num(Z), mask, iters=30000, tol=1e-8)
rep["well_fill"] = dict(box_x=[round(v, 2) for v in bx], box_y=[round(v, 2) for v in by], seam_x=round(seam_x, 2),
                        grid=[len(xs), len(ys)], cg_iterations=int(iters),
                        removed_max_mm=round(float(np.nanmax(np.where(mask, Z - Zf, np.nan))), 2),
                        filled_max_mm=round(float(np.nanmax(np.where(mask, Zf - Z, np.nan))), 2))
zlow = float(np.nanmin(Zf)) - 8.0
plug_top = np.where(mask, Zf, Z - 0.4)
cap_bot = np.where(mask, Zf, Z + 0.4)
lib.heightfield_solid("tool_plug", xs, ys, plug_top, np.full_like(Zf, zlow))
ztop_cap = float(np.nanmax(Z)) + 3.0
lib.heightfield_solid("tool_cap", xs, ys, np.full_like(Zf, ztop_cap), np.minimum(cap_bot, ztop_cap - 1.0))

# nose: the seam notches the front face, which the top-down fill cannot reach. Same fill on the front face seen
# along -y (height = y over an x-z grid), plug solid built in (x, z, y) and swapped into place.
from mathutils import Vector
from mathutils.bvhtree import BVHTree
bvh = BVHTree.FromObject(ref, bpy.context.evaluated_depsgraph_get())
nxs = np.arange(seam_x - 8, seam_x + 8 + 1e-9, 0.3)
nzs = np.arange(2.0, 24.0 + 1e-9, 0.3)
YN = np.full((len(nxs), len(nzs)), np.nan)
for i, x in enumerate(nxs):
    for j, z in enumerate(nzs):
        h = bvh.ray_cast(Vector((float(x), 120.0, float(z))), Vector((0, -1, 0)))
        if h[0] is not None:
            YN[i, j] = h[0].y
NX, NZ = np.meshgrid(nxs, nzs, indexing="ij")
nmask = (np.abs(NX - seam_x) <= SEAM_HW)
nmask[[0, 1, -2, -1], :] = False
nmask[:, [0, 1, -2, -1]] = False
assert np.isfinite(YN).all(), "nose ray misses"
YF, it2 = lib.biharmonic_fill(YN, nmask, iters=30000, tol=1e-8)
pn = lib.heightfield_solid("tool_noseplug", nxs, nzs, np.where(nmask, YF, YN - 0.4), np.full_like(YF, float(YF.min()) - 10.0))
pn.data.transform(__import__("mathutils").Matrix(((1, 0, 0, 0), (0, 0, 1, 0), (0, 1, 0, 0), (0, 0, 0, 1))))   # (x, z, y) -> (x, y, z)
bm_ = bmesh.new(); bm_.from_mesh(pn.data); bmesh.ops.recalc_face_normals(bm_, faces=bm_.faces); bm_.to_mesh(pn.data); bm_.free()
rep["nose_fill"] = dict(grid=[len(nxs), len(nzs)], cg_iterations=int(it2),
                        filled_max_mm=round(float(np.nanmax(np.where(nmask, YF - YN, np.nan))), 2))

# ---------------------------------------------------------------- 2. skirt prism
v = lib.verts(lib.obj("ref_overlay"))
v = v[v[:, 2] <= SKIRT_Z]
c = np.array([(v[:, 0].min() + v[:, 0].max()) / 2, (v[:, 1].min() + v[:, 1].max()) / 2])
a = np.arctan2(v[:, 1] - c[1], v[:, 0] - c[0])
r = np.hypot(v[:, 0] - c[0], v[:, 1] - c[1])
nb = 1440
b = ((a + np.pi) / (2 * np.pi) * nb).astype(int) % nb
R = np.zeros(nb)
np.maximum.at(R, b, r)
assert (R > 0).all(), "empty footprint bins"
R = np.maximum.reduce([np.roll(R, k) for k in range(-4, 5)])          # envelope over +-1 deg
g = np.exp(-0.5 * (np.arange(-12, 13) / 4.0) ** 2); g /= g.sum()     # then smooth (sigma 1 deg), never inside
Rs = np.real(np.fft.ifft(np.fft.fft(R) * np.fft.fft(np.roll(np.pad(g, (0, nb - len(g))), -12))))
R = Rs + max(0.0, float((R - Rs).max()))
t = (np.arange(nb) + 0.5) / nb * 2 * np.pi - np.pi
foot = c + np.column_stack([R * np.cos(t), R * np.sin(t)])
np.save(lib.OUT + "/footprint_shell.npy", foot)
lib.prism_solid("tool_skirt", foot, -6.0, SKIRT_Z, roll=SKIRT_ROLL, n_roll=12)
rep["skirt"] = dict(z=SKIRT_Z, roll=SKIRT_ROLL, plan_x=[round(float(foot[:, 0].min()), 2), round(float(foot[:, 0].max()), 2)],
                    plan_y=[round(float(foot[:, 1].min()), 2), round(float(foot[:, 1].max()), 2)])

# ---------------------------------------------------------------- 4. bowl cutter
wx, wy, wz = I["wheel"]["centre"]
wtop = wz + I["wheel"]["d"] / 2
zf = wtop - WHEEL_PROUD
ZTOP_B = 56.0                                   # above the skin (49.7)
bxs = np.arange(wx - 22, wx + 22 + 1e-9, 0.25)
bys = np.arange(wy - 30, wy + 30 + 1e-9, 0.25)
BX, BY = np.meshgrid(bxs, bys, indexing="ij")
DX, DY = BX - wx, BY - wy
DYW = np.minimum(DY, 0.0)                       # front: no rising wall - the floor runs out into the dome (no ridge
                                                # in front of the wheel, where the dome is at or below the floor)
rho = np.maximum(np.hypot(DX / BOWL_A, DYW / BOWL_B), 1e-9)
k = np.maximum(1 - 1 / rho, 0)                  # fraction of the radius beyond the floor ellipse
f = np.minimum(zf + BOWL_KX * (k * DX) ** 2 + BOWL_KY * (k * DYW) ** 2, ZTOP_B - 1.0)
lib.heightfield_solid("tool_bowl", bxs, bys, np.full_like(f, ZTOP_B), f)
rep["bowl"] = dict(floor_z=round(zf, 3), wheel_top_z=round(wtop, 3), proud=WHEEL_PROUD, A=BOWL_A, B=BOWL_B, KX=BOWL_KX, KY=BOWL_KY)
print(f"tools {time.time() - t0:.0f} s", flush=True)

# ---------------------------------------------------------------- SDF recipe
body = ("mesh", "ref_placed")
filled = ("union", ("union", ("diff", body, ("mesh", "tool_cap")), ("mesh", "tool_plug")), ("mesh", "tool_noseplug"))
skirted = ("union", filled, ("mesh", "tool_skirt"))
closed = ("close", skirted, CLOSE_R)
bowled = ("diff", closed, ("mesh", "tool_bowl"))
opened = ("open", bowled, OPEN_R)
smoothed = ("mean", opened, MEAN_W, MEAN_IT)
o = lib.sdf_mesh("outer_sdf", smoothed, voxel=VOXEL, band=BAND, coll="work")
print(f"sdf {time.time() - t0:.0f} s, {len(o.data.polygons)} faces", flush=True)

# ---------------------------------------------------------------- 6. bottom cut at z = 0
bm = bmesh.new()
bm.from_mesh(o.data)
geom = bm.verts[:] + bm.edges[:] + bm.faces[:]
res = bmesh.ops.bisect_plane(bm, geom=geom, dist=1e-6, plane_co=(0, 0, 0), plane_no=(0, 0, 1), clear_inner=True)
edges = [e for e in bm.edges if e.is_boundary]
bmesh.ops.holes_fill(bm, edges=edges, sides=0)
bmesh.ops.triangulate(bm, faces=[f_ for f_ in bm.faces if len(f_.verts) > 3])
bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
bm.to_mesh(o.data)
bm.free()
o.data.update()
lib.remove("outer_clean")                  # else the new object would be named outer_clean.001
o.name = o.data.name = "outer_clean"
rep["outer_clean"] = lib.topo(o)
vv = lib.verts(o)
rep["outer_clean"]["size"] = (vv.max(0) - vv.min(0)).round(3).tolist()
rep["outer_clean"]["lo"] = vv.min(0).round(3).tolist()
rep["outer_clean"]["hi"] = vv.max(0).round(3).tolist()
rep["time_s"] = round(time.time() - t0, 1)
for n in ("tool_plug", "tool_cap", "tool_noseplug", "tool_skirt", "tool_bowl"):
    lib.obj(n).hide_set(True)
with open(lib.OUT + "/step2_report.json", "w") as fh:
    json.dump(rep, fh, indent=1)
print(json.dumps(rep, indent=1))
