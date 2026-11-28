"""Reference-shape skin for the shell (DOME_MODE = "ref").

The outer shape is the reference mouse (reference/Mini Computer Mouse_2.stl, 50 x 32.1 x 17.8 mm, rear at low x, nose at
high x), scaled uniformly and placed on the fixed internals. This module:

1. voxelises the STL (ray parity on a REF_H grid, reference units) and cleans it: fills the sensor hole and the cable
   notch underneath, closes the seams and engraved lettering (morphological closing), removes embossed marks (opening),
   and fills the reference's own wheel well, wheel and small button (biharmonic inpaint of the top height map);
2. finds the smallest uniform scale (and the placement) that keeps every internal part (cell, board + components,
   switches, plungers) GAP inside the outer surface: 1.5 wall + 0.5 clearance;
3. lowers the skin around the fixed wheel (a smooth bowl, WHEEL_PROUD below the tyre top) - the only place the surface
   departs from the reference;
4. writes the tables the shell model lofts from: for each skin offset d, cross-sections across X at many Y (closed
   curves in the X-Z plane, resampled by arc length from the bottom centre) and the two tip points; the top height map;
   the top-view outline.

Run directly to rebuild the tables (python3 ref_skin.py); shell_model.py calls tables() and builds them if missing.
"""
import hashlib
import json
import os
import pickle
import sys
import time

import numpy as np
from scipy import ndimage
from scipy.ndimage import map_coordinates

HERE = os.path.dirname(os.path.abspath(__file__))
REF_STL = os.path.join(HERE, "reference", "Mini Computer Mouse_2.stl")
CACHE = os.path.join(HERE, ".cache")

REF_H = 0.06          # voxel pitch, reference units (0.17 mm at 2.8x)
PAD = 20              # empty voxels around the part (> every closing radius)
BELOW = 45            # voxels below z = 0: the footprint extruded down, so the lower skin runs on below the bottom cut
R_CLOSE = 0.5         # closing radius, reference units: fills seams / engraved lettering narrower than 1.0 (2.8 mm)
BAND = 1.2            # exact (mesh) distance within this band of the surface, reference units (covers the 2.03 mm fit
                      # depth and every skin offset); the voxel field is used only where the cleaning changed the shape
R_OPEN = 0.75         # opening radius: removes embossed marks; every convex edge keeps a radius >= 0.75 (2.1 mm at
                      # 2.78x), so the 1.55 mm inward offset (cavity) never folds and stays well sampled
R_CREASE = 0.5        # blend of the skirt into the shape above (smooth union): rounds the crease where the reference's lower side meets the
                      # straight extension below z = 0 (~2.8 mm), so the skin needs no extreme sampling there
LIP_TOP = 2.3         # the reference's lower lip (a band ~1.4 mm proud of the side panel, z ~0.5 - 2.2 reference
                      # units) becomes a straight skirt: the envelope of everything below LIP_TOP, extruded down (the
                      # reference also tucks in under the lip: a floor there would be thinner than the wall, and the
                      # bottom / split cuts near-tangent) ...
FLARE_R = 1.1         # ... rolling into the body above LIP_TOP on this radius (3 mm): no sharp ledge for the wall
                      # offset to fold on
SMOOTH = 1.5          # Gaussian sigma of the distance field, voxels
# the reference's wheel, wheel well and the small button behind it (reference units, x along the length)
WELL_BOX = (28.3, 44.4, 11.2, 18.2)
GAP = 2.16            # internal parts stay this far inside the outer skin: cavity offset 1.65 + 0.5 clearance (+ 0.01)
WHEEL_PROUD = 3.0     # tyre top above the bowl floor around the wheel
BOWL_A, BOWL_B = 5.6, 7.5        # flat floor half-width (x, beside the tyre + 1.6) and half-length (y, exposed tyre)
BOWL_KX, BOWL_KY = 0.25, 0.12    # wall rise: KX (|x| - A)^2 + KY (|y - wheel y| - B)^2, mm
BOWL_BLEND = 3.0                 # smooth-min blend between the bowl and the reference surface, mm (rim radius > 1.52)
OFFSETS = (0.0, 0.9, 1.65)       # skin offsets the model lofts: outer, feature trim (WALL - EMBED), cavity (WALL + margin)
CAVITY_D, FLOOR, FLOOR_BLEND = 1.65, 1.53, 1.0  # the cavity carries its own floor (smooth blend into the walls): a flat
                                                # cut at z = FLOOR would be tangent to the offset of the rear flares' underside
N_THETA, N_PHI = (int(v) for v in os.environ.get("REF_GRID", "360,240").split(","))   # radial grid
EMB_FLOOR_BLEND = 0.5


def _log(*a):
    print("[ref_skin]", *a, flush=True)


# ----------------------------------------------------------------------------------------------
# 1. voxelise and clean (reference units)
# ----------------------------------------------------------------------------------------------
def voxelise():
    import trimesh
    from trimesh.ray.ray_pyembree import RayMeshIntersector
    m = trimesh.load(REF_STL)
    m.apply_translation(-m.bounds[0])
    h = REF_H
    nx, ny = [int(np.ceil(e / h)) + 2 * PAD for e in m.extents[:2]]
    nz = int(np.ceil(m.extents[2] / h)) + PAD + BELOW
    ox, oy, oz = -PAD * h, -PAD * h, -BELOW * h               # world coordinate of voxel index 0 (voxel centres at +h/2)
    xs = ox + (np.arange(nx) + 0.5) * h
    ys = oy + (np.arange(ny) + 0.5) * h
    zs = oz + (np.arange(nz) + 0.5) * h
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    org = np.column_stack([X.ravel(), Y.ravel(), np.full(X.size, -1.0)])
    loc, ir, _ = RayMeshIntersector(m).intersects_location(org, np.tile([0, 0, 1.0], (len(org), 1)), multiple_hits=True)
    o = np.lexsort((loc[:, 2], ir))
    loc, ir = loc[o], ir[o]
    M = np.zeros((nx, ny, nz), bool)
    odd = 0
    for zz, rr in zip(np.split(loc[:, 2], np.flatnonzero(np.diff(ir)) + 1), np.split(ir, np.flatnonzero(np.diff(ir)) + 1)):
        zz = np.unique(np.round(zz, 6))
        if len(zz) % 2:
            odd += 1
            zz = zz[:-1]
        i, j = divmod(int(rr[0]), ny)
        for a, b in zip(zz[0::2], zz[1::2]):
            M[i, j, (zs > a) & (zs < b)] = True
    _log(f"voxelised {M.shape}, {odd} odd ray columns")
    return M, (ox, oy, oz), m.extents


def _dilate(M, r):
    return ndimage.distance_transform_edt(~M) * REF_H <= r


def _erode(M, r):
    return ndimage.distance_transform_edt(M) * REF_H > r


def clean(M, origin):
    """Mask cleaning: fill the underside (sensor hole, notch), extrude the footprint down, close / open."""
    ox, oy, oz = origin
    h = REF_H
    k0 = BELOW                                    # first voxel layer above z = 0
    raw = M.copy()
    # underside: the footprint (filled, notches closed) is solid from z = 0 up to the first solid voxel
    foot = ndimage.binary_fill_holes(M[:, :, k0])
    rr = int(round(1.2 / h))
    disk = np.hypot(*np.mgrid[-rr:rr + 1, -rr:rr + 1]) <= rr
    foot = ndimage.binary_fill_holes(ndimage.binary_closing(np.pad(foot, rr + 1), disk)[rr + 1:-rr - 1, rr + 1:-rr - 1])
    any_above = M[:, :, k0:].any(axis=2)
    first = np.where(any_above, np.argmax(M[:, :, k0:], axis=2), 0)
    kk = np.arange(M.shape[2] - k0)[None, None, :]
    M[:, :, k0:] |= foot[:, :, None] & (kk <= first[:, :, None]) & any_above[:, :, None]
    # every column solid from its lowest to its highest solid voxel (no internal voids)
    lo = np.where(M.any(2), np.argmax(M, 2), M.shape[2])
    hi = np.where(M.any(2), M.shape[2] - 1 - np.argmax(M[:, :, ::-1], 2), -1)
    kz = np.arange(M.shape[2])[None, None, :]
    M = (kz >= lo[:, :, None]) & (kz <= hi[:, :, None])
    # below z = 0: the z = 0 layer extruded down
    M[:, :, :k0] = M[:, :, k0][:, :, None]
    M = _erode(_dilate(M, R_CLOSE), R_CLOSE)       # closing: seams, engraved lettering
    M = _dilate(_erode(M, R_OPEN), R_OPEN)         # opening: embossed marks, convex edges >= R_OPEN
    M[:, :, :k0] = M[:, :, k0][:, :, None]         # keep the extrusion straight (closing / opening ran into it)

    return M, raw


def _exact_top(xs, ys):
    """Top height of the raw reference STL over a grid (ray cast)."""
    import trimesh
    from trimesh.ray.ray_pyembree import RayMeshIntersector
    m = trimesh.load(REF_STL)
    m.apply_translation(-m.bounds[0])
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    o = np.column_stack([X.ravel(), Y.ravel(), np.full(X.size, 30.0)])
    loc, ir, _ = RayMeshIntersector(m).intersects_location(o, np.tile([0, 0, -1.0], (len(o), 1)), multiple_hits=False)
    z = np.full(len(o), np.nan)
    z[ir] = loc[:, 2]
    return z.reshape(X.shape)


def _mesh_samples(n=8_000_000):
    """Dense samples of the raw reference surface with interpolated (smooth) vertex normals, in a KD-tree."""
    import trimesh
    from scipy.spatial import cKDTree
    m = trimesh.load(REF_STL)
    m.apply_translation(-m.bounds[0])
    P, fi = trimesh.sample.sample_surface(m, n, seed=1)
    bary = trimesh.triangles.points_to_barycentric(m.triangles[fi], P)
    N = (m.vertex_normals[m.faces[fi]] * bary[:, :, None]).sum(1)
    N /= np.linalg.norm(N, axis=1)[:, None]
    return cKDTree(P), N, P


def exact_sd(Q, samples):
    """Signed distance (positive inside) to the raw reference surface: point-to-plane at the nearest surface sample."""
    tree, N, P = samples
    out = np.empty(len(Q), np.float32)
    for a in range(0, len(Q), 2_000_000):
        _, i = tree.query(Q[a:a + 2_000_000], workers=-1)
        out[a:a + 2_000_000] = -np.einsum("ij,ij->i", Q[a:a + 2_000_000] - P[i], N[i])
    return out


def _fields_key():
    import inspect
    with open(REF_STL, "rb") as f:
        stl = hashlib.sha1(f.read()).hexdigest()
    fns = (voxelise, clean, _mesh_samples, exact_sd, _dilate, _erode, distance_field)
    consts = (REF_H, PAD, BELOW, R_CLOSE, R_OPEN, SMOOTH, BAND)
    return hashlib.sha1(repr((stl, consts, [inspect.getsource(fn) for fn in fns])).encode()).hexdigest()[:16]


def build_field(M, raw, origin):
    """The cleaned reference as a smooth field (reference units, positive inside):
    - exact distance to the reference mesh wherever the cleaning left the surface as it was (no voxel terracing);
    - the (smoothed) voxel field of the cleaned mask where the cleaning changed it (seams, lettering, rounded edges);
    - the wheel well, wheel and small button replaced by a biharmonic fill of the exact top heights (a smooth height
      field, not voxels).
    Returns (phi_ref, phi_raw): the cleaned field, and the raw reference's exact field (for the report's raw fit)."""
    from skimage.restoration import inpaint_biharmonic
    h = REF_H
    ox, oy, oz = origin
    k0 = BELOW
    fc = os.path.join(CACHE, f"ref_fields-{_fields_key()}")
    if os.path.exists(fc + "-phi_raw.npy"):
        d_clean = np.load(fc + "-d_clean.npy")
        d_raw = np.load(fc + "-d_raw.npy")
        phi_raw = np.load(fc + "-phi_raw.npy")
    else:
        rawx = raw.copy()
        rawx[:, :, :k0] = rawx[:, :, k0][:, :, None]
        d_clean = distance_field(M)
        d_raw = distance_field(rawx)
        del rawx
        # exact distance in the band
        band = np.abs(d_raw) < BAND
        idx = np.argwhere(band)
        Q = np.asarray(origin) + (idx + 0.5) * h
        samples = _mesh_samples()
        ex = exact_sd(Q, samples)
        del samples, Q, idx
        phi_raw = d_raw.copy()
        wb = np.clip((BAND - np.abs(d_raw[band])) / 0.3, 0.0, 1.0)            # fade out at the band edge
        phi_raw[band] = wb * ex + (1 - wb) * d_raw[band]
        del band, ex, wb
        os.makedirs(CACHE, exist_ok=True)
        for nm, arr in (("d_clean", d_clean), ("d_raw", d_raw), ("phi_raw", phi_raw)):
            np.save(fc + f"-{nm}.npy", arr)
    # where did the cleaning change the shape? (smoothed fields differ by more than a voxel)
    changed = np.abs(d_clean - d_raw) > 1.0 * h
    w = np.clip(ndimage.gaussian_filter(changed.astype(np.float32), 2.0) * 3.0, 0.0, 1.0)
    d_cs = ndimage.gaussian_filter(d_clean, 3.0)            # cleaned voxel field, smoothed further (no terracing)
    phi = (w * d_cs + (1 - w) * phi_raw).astype(np.float32)
    del d_clean, d_cs, changed, w
    # top-surface marks: the exact top height map (ray cast) over the whole part; marks = where it departs from its own
    # smoothed version (seams, lettering, the nose mark) inside the top region (surface slope < ~53 deg); the wheel
    # well, wheel and small button (inside WELL_BOX, outlines filled). Both get a biharmonic fill of the exact heights.
    xs = ox + (np.arange(M.shape[0]) + 0.5) * h
    ys = oy + (np.arange(M.shape[1]) + 0.5) * h
    zs = oz + (np.arange(M.shape[2]) + 0.5) * h
    Te = _exact_top(xs, ys)
    valid = np.isfinite(Te)
    Tf = np.where(valid, Te, 0.0)
    gx, gy = np.gradient(ndimage.gaussian_filter(Tf, 3), h)
    topreg = valid & (np.hypot(gx, gy) < 1.33) & ndimage.binary_erosion(valid, iterations=12)
    hp = np.abs(Tf - ndimage.gaussian_filter(Tf, 3))
    x0, x1, y0, y1 = WELL_BOX
    box_ = ((xs[:, None] >= x0) & (xs[:, None] <= x1)) & ((ys[None, :] >= y0) & (ys[None, :] <= y1))
    well = ndimage.binary_fill_holes(ndimage.binary_dilation((hp > 0.03) & box_, iterations=2))
    well = ndimage.binary_dilation(well, iterations=5) & box_
    marks = ndimage.binary_dilation((hp > 0.015) & topreg & ~box_, iterations=4) & topreg
    feat = well | marks
    Ti = Tf.copy()
    Ti[feat] = np.nan
    Ti = inpaint_biharmonic(np.where(valid, Tf, np.nan_to_num(Tf)), feat)
    Ti = np.where(valid, Ti, np.nan)
    g2x, g2y = np.gradient(np.nan_to_num(Ti), h)
    norm = np.sqrt(1 + g2x ** 2 + g2y ** 2)
    ww = np.clip(ndimage.gaussian_filter(ndimage.binary_dilation(well, iterations=3).astype(float), 2.0) * 2.0, 0, 1)
    wm = np.clip(ndimage.gaussian_filter(ndimage.binary_dilation(marks, iterations=2).astype(float), 1.5) * 2.0, 0, 1)
    ww = np.where(valid, ww, 0.0)              # the blurred weights must not reach columns off the part (no top there)
    wm = np.where(valid, wm, 0.0)
    for i in np.flatnonzero((ww > 0).any(1) | (wm > 0).any(1)):
        js = np.flatnonzero((ww[i] > 0) | (wm[i] > 0))
        pin = (Ti[i, js][:, None] - zs[None, :]) / norm[i, js][:, None]
        # marks: only near the top (they are shallow; deeper down the column belongs to the side walls); well: every depth
        dz = (zs[None, :] - (Ti[i, js][:, None] - 0.6)) / 0.2
        wz = np.clip(dz, 0.0, 1.0)
        wv = np.maximum(ww[i, js][:, None], wm[i, js][:, None] * wz)
        phi[i, js] = (1 - wv) * phi[i, js] + wv * pin
    assert np.isfinite(phi).all(), "top fill left NaN in the field"
    _log(f"top fill: well {int(well.sum())} columns, marks {int(marks.sum())} columns")
    return phi, phi_raw


def skirt(phi):
    """Straight skirt: the envelope of every section below LIP_TOP, extruded down; above LIP_TOP it rolls in on radius
    FLARE_R (parabolic profile), and it joins the body with a smooth union (blend R_CREASE). Adds material only, after
    the scale fit (which is done on the reference shape itself)."""
    h = REF_H
    k0 = BELOW
    kt = BELOW + int(round(LIP_TOP / h))
    S = phi[:, :, k0:kt + 1].max(axis=2)
    t = (np.arange(phi.shape[2]) - kt) * h                     # height above LIP_TOP
    out = np.empty_like(phi)
    for k in range(phi.shape[2]):
        L = S - (t[k] ** 2 / (2 * FLARE_R) if t[k] > 0 else 0.0)
        out[:, :, k] = -smin(-phi[:, :, k], -L, R_CREASE)
    return out


def _top_map(M, origin):
    """Top surface height per column (sub-voxel, from the distance field across the boundary)."""
    h = REF_H
    oz = origin[2]
    has = M.any(2)
    hi = np.where(has, M.shape[2] - 1 - np.argmax(M[:, :, ::-1], 2), 0)
    return np.where(has, oz + (hi + 1) * h, np.nan)


def distance_field(M):
    """Signed distance (reference units, positive inside), smoothed."""
    d = (ndimage.distance_transform_edt(M) - ndimage.distance_transform_edt(~M)) * REF_H
    d = d + np.where(M, -0.5, 0.5) * REF_H          # boundary half-way between the voxel centres
    return ndimage.gaussian_filter(d.astype(np.float32), SMOOTH)


# ----------------------------------------------------------------------------------------------
# 2. scale and placement
# ----------------------------------------------------------------------------------------------
class Field:
    """The cleaned reference distance field, evaluated in shell coordinates for a scale s and a placement (cx, cy):
    the reference point (cx, cy) lands on the shell origin, its +x (nose) on shell +y, its +y on shell -x."""

    def __init__(self, sdf, origin, s=None, cx=None, cy=None, coef=None):
        self.sdf, self.origin = sdf, np.asarray(origin, float)
        self.coef = coef
        self.s, self.cx, self.cy = s, cx, cy
        self.bowl = None

    def to_ref(self, P, s=None, cx=None, cy=None):
        s = self.s if s is None else s
        cx = self.cx if cx is None else cx
        cy = self.cy if cy is None else cy
        P = np.atleast_2d(P)
        return np.column_stack([cx + P[:, 1] / s, cy - P[:, 0] / s, P[:, 2] / s])

    def ref_sd(self, u, order=1):
        g = (u - self.origin) / REF_H - 0.5
        if order == 3:
            return map_coordinates(self.coef, g.T, order=3, mode="nearest", prefilter=False)
        return map_coordinates(self.sdf, g.T, order=1, mode="nearest")

    def __call__(self, P, order=3):
        """Signed distance (mm, positive inside) of the outer skin at shell points P, bowl included."""
        P = np.atleast_2d(P)
        d = self.s * self.ref_sd(self.to_ref(P), order)
        if self.bowl is not None:
            d = smin(d, self.bowl(P), BOWL_BLEND)
        return d


def smin(a, b, k):
    """Polynomial smooth minimum (intersection of two inside-positive fields), blend width k."""
    hh = np.clip(0.5 + 0.5 * (b - a) / k, 0.0, 1.0)
    return b + (a - b) * hh - k * hh * (1 - hh)


def internal_points():
    """Dense surface samples of the fixed internals (cell, board + components, switches, plungers), taken in a
    subprocess (shell_model imports this module; the internals do not depend on the skin)."""
    import subprocess
    import tempfile
    code = r"""
import sys, json, numpy as np, trimesh
sys.path.insert(0, %r)
import shell_model as m
pts, labels = [], []
for name, shape in (("board", m.board_model()), ("switches", m.switches_model()),
                    ("battery", m.battery_model()), ("keys", m.keys_part())):
    v, f = shape.tessellate(0.02, 0.2)
    me = trimesh.Trimesh(np.array([[a.x, a.y, a.z] for a in v]), np.array(f), process=False)
    p_, _ = trimesh.sample.sample_surface_even(me, int(me.area / 0.25) + 1, radius=0.35, seed=1)
    p_ = np.vstack([p_, me.vertices])
    pts.append(p_); labels += [name] * len(p_)
np.savez(sys.argv[1], P=np.vstack(pts), labels=np.array(labels))
""" % HERE
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, "pts.npz")
        env = dict(os.environ, SHELL_PARAMS=json.dumps({"DOME_MODE": "separable"}), SHELL_NO_CACHE="1")
        subprocess.run([sys.executable, "-c", code, out], check=True, env=env, cwd=HERE,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        z = np.load(out)
        return z["P"], z["labels"]


def fit_scale(F, P, labels):
    """Smallest s (and its cx, cy) with every internal point GAP inside the outer skin."""
    from scipy.optimize import minimize

    def margin(s, cx, cy, order=1, idx=slice(None)):
        return s * F.ref_sd(F.to_ref(P[idx], s, cx, cy), order) - GAP

    def smin_(cx, cy, order=1, idx=slice(None), lo=1.5, hi=5.0):
        if margin(hi, cx, cy, order, idx).min() < 0:
            return hi + 1
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if margin(mid, cx, cy, order, idx).min() >= 0:
                hi = mid
            else:
                lo = mid
        return hi

    best = min((smin_(a, b), a, b) for a in np.linspace(22, 30, 17) for b in np.linspace(14.5, 17, 11))
    r = minimize(lambda q: smin_(q[0], q[1]), best[1:], method="Nelder-Mead",
                 options=dict(xatol=0.005, fatol=1e-5, maxiter=400))
    cx, cy = r.x
    s1 = smin_(cx, cy)
    # final scale on the cubic-interpolated field, near points only
    near = np.flatnonzero(margin(s1, cx, cy) < 1.0)
    r = minimize(lambda q: smin_(q[0], q[1], 3, near), [cx, cy], method="Nelder-Mead",
                 options=dict(xatol=0.002, fatol=1e-6, maxiter=200))
    cx, cy = r.x
    s = smin_(cx, cy, 3, near)
    mg = margin(s, cx, cy, 3)
    o = np.argsort(mg)
    lim = []
    for i in o:
        key = (labels[i], tuple(np.round(P[i], 0)))
        if key not in [l[:2] for l in lim]:
            lim.append((labels[i], tuple(np.round(P[i], 0)), tuple(np.round(P[i], 2)), round(float(mg[i]), 3)))
        if len(lim) >= 6:
            break
    return dict(s=float(s), cx=float(cx), cy=float(cy), limiting=[dict(part=l[0], at=l[2], margin=l[3]) for l in lim])


# ----------------------------------------------------------------------------------------------
# 3. bowl around the fixed wheel
# ----------------------------------------------------------------------------------------------
def make_bowl(wheel_x, wheel_y, wheel_top):
    zf = wheel_top - WHEEL_PROUD

    def bowl(P):
        P = np.atleast_2d(P)
        ex = np.maximum(np.abs(P[:, 0] - wheel_x) - BOWL_A, 0.0)
        ey = np.maximum(np.abs(P[:, 1] - wheel_y) - BOWL_B, 0.0)
        f = zf + BOWL_KX * ex ** 2 + BOWL_KY * ey ** 2
        g = np.sqrt(1 + (2 * BOWL_KX * ex) ** 2 + (2 * BOWL_KY * ey) ** 2)
        return (f - P[:, 2]) / g                 # inside (below the bowl surface) positive, ~ distance
    return bowl, zf


# ----------------------------------------------------------------------------------------------
# 4. tables
# ----------------------------------------------------------------------------------------------
def _bbox(F, d):
    """Shell-coordinate box of the level set {F >= -d} (from the reference grid)."""
    M = F.sdf > -d / F.s
    idx = np.argwhere(M)
    lo = F.origin + (idx.min(0) + 0.5) * REF_H
    hi = F.origin + (idx.max(0) + 0.5) * REF_H
    c = np.array([[lo[0], lo[1]], [hi[0], lo[1]], [lo[0], hi[1]], [hi[0], hi[1]]])
    ys = (c[:, 0] - F.cx) * F.s
    xs = (F.cy - c[:, 1]) * F.s
    return xs.min() - 2, xs.max() + 2, ys.min() - 2, ys.max() + 2


def _field2(F, d, z_floor, blend=1.0):
    """The field the sections are traced on: skin offset d, closed below at z_floor (smooth-min blend)."""
    def f(P):
        return smin(F(P) - d, P[:, 2] - z_floor, blend)
    return f


def _tip(f, y0, y1, xr, zr, sign):
    """Extreme point (along +y if sign > 0) of the region f >= 0: bisection on y for the last y with any inside point."""
    X, Z = np.meshgrid(np.arange(*xr, 0.2), np.arange(*zr, 0.2), indexing="ij")
    lo, hi = y0, y1                                  # y0 inside, y1 outside (beyond the tip on the `sign` side)
    for _ in range(30):
        mid = 0.5 * (lo + hi)
        v = f(np.column_stack([X.ravel(), np.full(X.size, mid), Z.ravel()]))
        if v.max() >= 0:
            lo = mid
        else:
            hi = mid
    v = f(np.column_stack([X.ravel(), np.full(X.size, lo), Z.ravel()]))
    i = int(np.argmax(v))
    return np.array([X.ravel()[i], lo, Z.ravel()[i]])


def _refine_tip(f, tip, sign):
    """Tip (x, z) = centre of the end section 0.002 mm in from the tip, traced on a fine window."""
    from skimage import measure
    y = tip[1] - sign * 0.002
    xs = np.arange(tip[0] - 1.5, tip[0] + 1.5, 0.005)
    zs = np.arange(tip[2] - 1.5, tip[2] + 1.5, 0.005)
    X, Z = np.meshgrid(xs, zs, indexing="ij")
    v = f(np.column_stack([X.ravel(), np.full(X.size, y), Z.ravel()])).reshape(X.shape)
    cs = measure.find_contours(v, 0.0)
    if not cs:
        return tip
    c = max(cs, key=len)
    return np.array([xs[0] + c[:, 0].mean() * 0.005, tip[1], zs[0] + c[:, 1].mean() * 0.005])


def _section(f, y, xr, zr, x_start, step=0.08):
    """Closed curve of the level set f = 0 in the plane at y, as N_SEC_PTS points equally spaced in arc length,
    counter-clockwise in (x, z), starting where the curve crosses x = x_start on its lower side (a smooth line through
    both tips, so the start never jumps between sections)."""
    from skimage import measure
    xs = np.arange(xr[0], xr[1], step)
    zs = np.arange(zr[0], zr[1], step)
    X, Z = np.meshgrid(xs, zs, indexing="ij")
    v = f(np.column_stack([X.ravel(), np.full(X.size, y), Z.ravel()])).reshape(X.shape)
    cs = [c for c in measure.find_contours(v, 0.0) if np.allclose(c[0], c[-1])]
    if not cs:
        return None, 0
    c = max(cs, key=len)
    pts = np.column_stack([xr[0] + c[:, 0] * step, zr[0] + c[:, 1] * step])[:-1]
    n_closed = len(cs)
    if 0.5 * np.sum(pts[:, 0] * np.roll(pts[:, 1], -1) - np.roll(pts[:, 0], -1) * pts[:, 1]) < 0:
        pts = pts[::-1]
    # lower crossing of x = x_start: segments that cross it, lowest one, exact crossing point inserted
    a, b = pts, np.roll(pts, -1, axis=0)
    cr = np.flatnonzero((a[:, 0] - x_start) * (b[:, 0] - x_start) <= 0)
    zc = a[cr, 1] + (x_start - a[cr, 0]) * (b[cr, 1] - a[cr, 1]) / np.where(b[cr, 0] == a[cr, 0], 1e-12, b[cr, 0] - a[cr, 0])
    k = cr[np.argmin(zc)]
    p0 = np.array([x_start, zc.min()])
    pts = np.vstack([p0, np.roll(pts, -(k + 1), axis=0), p0])
    seg = np.hypot(*np.diff(pts, axis=0).T)
    sl = np.concatenate([[0], np.cumsum(seg)])
    t = np.linspace(0, sl[-1], N_SEC_PTS + 1)[:-1]
    out = np.column_stack([np.interp(t, sl, pts[:, 0]), np.full(len(t), y), np.interp(t, sl, pts[:, 1])])
    return out, n_closed


def _section_ys(y0, y1, dense, d0=0.01, ratio=1.7, base=2.5, fine=1.0):
    """Section positions between the tips y0 < y1: geometric from each tip (d0, d0 r, ... up to `base`: the blunt
    ends need sections very close to the tip), then every `base` mm, every `fine` mm over the bowl (dense = (ya, yb))."""
    tip = []
    dd = d0
    while dd < base:
        tip.append(dd)
        dd *= ratio
    ya, yb = y0 + tip[-1], y1 - tip[-1]
    mid = [ya]
    while mid[-1] < yb:
        y = mid[-1]
        st = fine if dense[0] - fine < y < dense[1] else base
        mid.append(y + st)
    mid = np.linspace(ya, yb, len(mid))           # even out the last step
    return np.concatenate([[y0 + d for d in tip], mid[1:-1], [y1 - d for d in tip[::-1]]])


def _coarse_top(F, X, Y, zr=(-1.0, 70.0), dz=0.25):
    """Highest z with F > 0 per column (coarse scan from the top, then bisection)."""
    zs = np.arange(zr[1], zr[0], -dz)
    n = X.size
    found = np.full(n, np.nan)
    for z in zs:
        m = np.isnan(found)
        if not m.any():
            break
        v = F(np.column_stack([X.ravel()[m], Y.ravel()[m], np.full(m.sum(), z)]))
        idx = np.flatnonzero(m)[v > 0]
        found[idx] = z
    lo = found.copy()
    hi = found + dz
    ok = np.isfinite(found)
    for _ in range(25):
        mid = 0.5 * (lo + hi)
        v = np.zeros(n, bool)
        v[ok] = F(np.column_stack([X.ravel()[ok], Y.ravel()[ok], mid[ok]])) > 0
        lo = np.where(ok & v, mid, lo)
        hi = np.where(ok & ~v, mid, hi)
    return np.where(ok, lo, np.nan).reshape(X.shape)


def _rays(f, C, dirs, rmax=130.0, step=0.3):
    """Outermost crossing of f = 0 along C + r d (f > 0 inside), and the number of sign changes along each ray."""
    rs = np.arange(0.0, rmax, step)
    R = np.empty(len(dirs))
    ncross = np.empty(len(dirs), int)
    for a in range(0, len(dirs), 400):
        D = dirs[a:a + 400]
        P = C[None, None, :] + rs[None, :, None] * D[:, None, :]
        v = f(P.reshape(-1, 3)).reshape(len(D), len(rs)) > 0
        ncross[a:a + 400] = (np.diff(v.astype(np.int8), axis=1) != 0).sum(1)
        last = len(rs) - 1 - np.argmax(v[:, ::-1], axis=1)
        lo, hi = rs[last], rs[last] + step
        for _ in range(32):
            mid = 0.5 * (lo + hi)
            ins = f(C[None, :] + mid[:, None] * D) > 0
            lo, hi = np.where(ins, mid, lo), np.where(ins, hi, mid)
        R[a:a + 400] = 0.5 * (lo + hi)
    return R, ncross


def _dirs(th, ph):
    """Unit directions: azimuth th about +z (from +x), polar ph from -z (0 = straight down, pi = straight up)."""
    th, ph = np.broadcast_arrays(th, ph)
    return np.stack([np.sin(ph) * np.cos(th), np.sin(ph) * np.sin(th), -np.cos(ph)], -1)


def _equalise(x, w, n, periodic=False):
    """n samples of x (dense, increasing) equally spaced in the cumulative weight w."""
    cw = np.concatenate([[0], np.cumsum(0.5 * (w[1:] + w[:-1]) * np.diff(x))])
    q = np.linspace(0, cw[-1], n + (1 if periodic else 0))
    out = np.interp(q, cw, x)
    return out[:-1] if periodic else out


def radial_grid(F, zlo, bbox, wheel):
    """The outer skin (bowl included, closed at zlo below the bottom cut) as R(theta, phi) from a centre C just above
    the bottom: star-shaped from there (checked: one crossing per ray). theta is spaced by arc length around the
    outline, phi by arc length along the meridians, with extra density through the bottom crease and over the bowl."""
    X0, X1, Y0, Y1 = bbox
    f = lambda P: smin(F(P), P[:, 2] - zlo, 1.0)
    C = np.array([0.5 * (X0 + X1), 0.5 * (Y0 + Y1), 1.0])
    # theta: arc length of the outline 3 mm up (just above the crease), 70 % arc / 30 % uniform
    tf = np.linspace(0, 2 * np.pi, 4001)[:-1]
    Rt, _ = _rays(f, C, _dirs(tf, np.full_like(tf, np.arctan2(1.0, 2.0) + np.pi / 2)))
    xy = np.column_stack([Rt * np.cos(tf), Rt * np.sin(tf)])
    ds = np.hypot(*np.diff(np.vstack([xy, xy[:1]]), axis=0).T)
    w = 0.7 * ds / ds.mean() + 0.3
    th = _equalise(np.append(tf, 2 * np.pi), np.append(w, w[0]), N_THETA, periodic=True)
    # phi: average meridian arc length (16 meridians + the one through the bowl, weighted), + crease band
    pf = np.linspace(1e-3, np.pi - 1e-3, 3001)
    tb = np.arctan2(wheel[1] - C[1], wheel[0] - C[0])
    lens = []
    for t_ in list(np.linspace(0, 2 * np.pi, 17)[:-1]) + [tb] * 8:
        Rm, _ = _rays(f, C, _dirs(np.full_like(pf, t_), pf))
        P = np.column_stack([Rm * np.sin(pf), -Rm * np.cos(pf)])
        dl = np.hypot(*np.diff(P, axis=0).T)
        lens.append(dl / np.diff(pf))
    w = np.mean(lens, 0)
    w = w / w.mean()
    pm = 0.5 * (pf[1:] + pf[:-1])
    w += 3.0 * np.exp(-0.5 * ((pm - np.pi / 2) / np.radians(3.0)) ** 2)      # crease (z ~ 0..3 seen from z = 1)
    ph = _equalise(pm, w, N_PHI - 2)
    ph = np.concatenate([[0.0], ph, [np.pi]])
    TH, PH = np.meshgrid(th, ph, indexing="ij")
    R, nc = _rays(f, C, _dirs(TH.ravel(), PH.ravel()))
    R = R.reshape(TH.shape)
    nc = nc.reshape(TH.shape)
    G = C + R[..., None] * _dirs(TH, PH)
    G[:, 0] = G[0, 0]
    G[:, -1] = G[0, -1]
    multi = int((nc > 1).sum())
    _log(f"radial grid {G.shape[0]} x {G.shape[1]}, rays with more than one crossing: {multi}")
    if multi:
        bad = G[nc > 1]
        _log("  e.g. at", np.round(bad[:5], 2).tolist())
    return G, C, dict(theta=th, phi=ph, multi_crossing_rays=multi)


def offset_grid(F, G, d, floor, k, z_a=3.5, r_b=1.0):
    """Skin offset d inward, on the outer grid's parametrisation (so wall errors stay correlated with the outer skin).
    Above z_a (on the outer skin): every point moved d along the inward field normal, then projected onto the level set
    F = d. Below z_a (the straight skirt and the bottom): each meridian's points are laid, at the same arc-length
    fractions as on the outer meridian, along floor -> rounded corner (radius r_b) -> vertical wall -> its first offset
    point; the floor is at `floor` (part of the skin: no flat cut at the floor)."""
    nt, nphi, _ = G.shape
    out = np.empty_like(G)
    P = G.reshape(-1, 3).copy()
    up = P[:, 2] >= z_a
    def grad(Q):
        e = 0.02
        g = np.column_stack([(F(Q + [e, 0, 0]) - F(Q - [e, 0, 0])), (F(Q + [0, e, 0]) - F(Q - [0, e, 0])),
                             (F(Q + [0, 0, e]) - F(Q - [0, 0, e]))]) / (2 * e)
        return g / np.maximum(np.linalg.norm(g, axis=1), 1e-9)[:, None]
    Q = P[up]
    Q = Q + d * grad(Q)                     # pure normal offset of the outer points: the wall is d point by point
    err = float(np.abs(F(Q) - d).max())     # (reported only: F is not a true distance in the smooth blends)
    P[up] = Q
    P = P.reshape(G.shape)
    pole = np.array([G[0, 0, 0], G[0, 0, 1], floor])       # bottom pole straight above the outer one
    for i in range(nt):
        ja = int(np.argmax(G[i, :, 2] >= z_a))
        Pa = P[i, ja]
        # path: pole -> floor -> arc (radius r_b) -> wall straight up to Pa
        u = Pa[:2] - pole[:2]
        L = np.linalg.norm(u)
        u = u / L
        corner = np.array([*(pole[:2] + u * (L - r_b)), floor])
        arc = [np.array([*(pole[:2] + u * (L - r_b + r_b * np.sin(a))), floor + r_b - r_b * np.cos(a)])
               for a in np.linspace(0, np.pi / 2, 9)]
        path = np.vstack([pole, corner, *arc[1:], [Pa[0], Pa[1], floor + r_b], Pa])
        sl = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))])
        so = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(G[i, :ja + 1], axis=0), axis=1))])
        fr = so / so[-1] * sl[-1]
        P[i, :ja + 1] = np.column_stack([np.interp(fr, sl, path[:, c]) for c in range(3)])
    P[:, 0] = pole
    P[:, -1] = P[0, -1]
    return P, err


def _core_key():
    import inspect
    fns = (voxelise, clean, build_field, skirt, _mesh_samples, exact_sd, smin, _exact_top, _dilate, _erode, _fields_key,
           distance_field, fit_scale, internal_points, Field)
    with open(REF_STL, "rb") as f:
        stl = hashlib.sha1(f.read()).hexdigest()
    consts = (REF_H, PAD, BELOW, R_CLOSE, R_OPEN, R_CREASE, LIP_TOP, FLARE_R, SMOOTH, WELL_BOX, GAP, BAND)
    return hashlib.sha1(repr((stl, consts, [inspect.getsource(fn) for fn in fns])).encode()).hexdigest()[:16]


def core():
    """The cleaned field (skirt included), origin, extents and the scale fits (cleaned and raw), cached on disk."""
    path = os.path.join(CACHE, f"ref_core-{_core_key()}.pkl")
    if os.path.exists(path):
        with open(path, "rb") as f:
            c = pickle.load(f)
        c["phi"] = c["phi"].astype(np.float32) / 8000.0
        return c
    t0 = time.time()
    M, origin, ext = voxelise()
    Mc, raw = clean(M, origin)
    _log(f"cleaned {time.time() - t0:.0f} s")
    phi, phi_raw = build_field(Mc, raw, origin)
    del Mc
    _log(f"field {time.time() - t0:.0f} s")
    P, labels = internal_points()
    F = Field(phi, origin, coef=ndimage.spline_filter(phi, order=3, output=np.float32))
    fit = fit_scale(F, P, labels)
    _log(f"scale {fit['s']:.4f} cx {fit['cx']:.3f} cy {fit['cy']:.3f} limiting {fit['limiting'][:2]}  {time.time() - t0:.0f} s")
    del F
    Fr = Field(phi_raw, origin, coef=ndimage.spline_filter(phi_raw, order=3, output=np.float32))
    fit_raw = fit_scale(Fr, P, labels)          # same fit on the raw (uncleaned) reference, for the report
    del Fr, phi_raw
    phi = skirt(phi)
    c = dict(phi=np.round(np.clip(phi, -4.0, 4.0) * 8000.0).astype(np.int16), origin=origin, ext=ext, fit=fit,
             fit_raw=fit_raw, P=P, labels=labels)
    os.makedirs(CACHE, exist_ok=True)
    with open(path + ".tmp", "wb") as f:
        pickle.dump(c, f, protocol=4)
    os.replace(path + ".tmp", path)
    c["phi"] = np.clip(phi, -4.0, 4.0)
    return c


def build_tables(wheel=(0.0, 20.25, 37.866858), verbose=True):
    t0 = time.time()
    c = core()
    phi, origin, ext, fit, fit_raw, P = c["phi"], c["origin"], c["ext"], c["fit"], c["fit_raw"], c["P"]
    F = Field(phi, origin, coef=ndimage.spline_filter(phi, order=3, output=np.float32))
    F.s, F.cx, F.cy = fit["s"], fit["cx"], fit["cy"]
    _log(f"core ready {time.time() - t0:.0f} s")
    bowl, zf = make_bowl(*wheel)
    F.bowl = bowl
    # gap check on the final field (bowl included)
    gap_min = float((F(P) - 0.0).min())
    tables = dict(fit=fit, fit_raw=fit_raw, bowl=dict(floor_z=zf, A=BOWL_A, B=BOWL_B, KX=BOWL_KX, KY=BOWL_KY,
                                                       blend=BOWL_BLEND, wheel=wheel),
                  internals_min_depth_below_outer_skin=gap_min, offsets={})
    X0, X1, Y0, Y1 = _bbox(F, 0.0)
    zlo = -4.0
    grid, C, info = radial_grid(F, zlo, (X0, X1, Y0, Y1), wheel)
    tables["centre"] = C
    tables["grid_info"] = info
    tables["offsets"][0.0] = dict(grid=grid, floor=None)
    for d in OFFSETS[1:]:
        fl, k = (FLOOR, FLOOR_BLEND) if d == CAVITY_D else (FLOOR, EMB_FLOOR_BLEND)
        g, err = offset_grid(F, grid, d, fl, k)
        tables["offsets"][d] = dict(grid=g, floor=fl, level_set_err=err)
        _log(f"offset {d}: level-set error after projection {err:.4f} mm  {time.time() - t0:.0f} s")
    # top height map (outer skin) and the top-view outline
    xs = np.arange(X0, X1 + 1e-9, 0.25)
    ys = np.arange(Y0, Y1 + 1e-9, 0.25)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    Z = _coarse_top(F, X, Y)
    tables["top_map"] = dict(xs=xs, ys=ys, z=Z)
    from skimage import measure
    sil = np.isfinite(Z)
    c = max(measure.find_contours(np.pad(sil.astype(float), 1), 0.5), key=len)
    tables["outline"] = np.column_stack([X0 + (c[:, 0] - 1) * 0.25, Y0 + (c[:, 1] - 1) * 0.25])
    # peak
    i = np.nanargmax(Z)
    tables["peak"] = (float(X.ravel()[i]), float(Y.ravel()[i]), float(np.nanmax(Z)))
    tables["ref_extents"] = tuple(map(float, ext))
    # preview mesh of the final outer skin (marching cubes, 0.35 mm), for renders and the silhouette check
    from skimage import measure
    st = 0.35
    gx, gy, gz = np.arange(X0, X1, st), np.arange(Y0, Y1, st), np.arange(-0.5, tables["peak"][2] + 2, st)
    G = np.stack(np.meshgrid(gx, gy, gz, indexing="ij"), -1).reshape(-1, 3)
    v = np.concatenate([F(G[i:i + 2000000]) for i in range(0, len(G), 2000000)]).reshape(len(gx), len(gy), len(gz))
    v[:, :, 0] = -1.0                                   # closed at z ~ 0
    vv, ff, _, _ = measure.marching_cubes(v, 0.0, spacing=(st, st, st))
    tables["preview"] = dict(v=(vv + [gx[0], gy[0], gz[0]]).astype(np.float32), f=ff[:, ::-1].astype(np.int32))
    _log(f"done {time.time() - t0:.0f} s")
    return tables


def src_hash():
    with open(__file__, "rb") as f:
        a = f.read()
    with open(REF_STL, "rb") as f:
        b = f.read()
    return hashlib.sha1(a + b + f"{N_THETA}x{N_PHI}".encode()).hexdigest()[:16]


def tables():
    path = os.path.join(CACHE, f"ref_skin-{src_hash()}.pkl")
    if os.path.exists(path):
        with open(path, "rb") as f:
            return pickle.load(f)
    t = build_tables()
    os.makedirs(CACHE, exist_ok=True)
    with open(path + ".tmp", "wb") as f:
        pickle.dump(t, f)
    os.replace(path + ".tmp", path)
    return t


if __name__ == "__main__":
    t = tables()
    print(json.dumps(dict(fit=t["fit"], fit_raw=t["fit_raw"], bowl=t["bowl"], peak=t["peak"],
                          gap=t["internals_min_depth_below_outer_skin"],
                          grid=t["grid_info"]["multi_crossing_rays"],
                          level_set_err={d: v.get("level_set_err") for d, v in t["offsets"].items()}), indent=1, default=str))
