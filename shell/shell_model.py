#!/usr/bin/env python3
"""
Parametric demo mouse shell for the ESP32-S31 Vision Pro Rev C board.

Run:   python3 shell_model.py            -> builds parts, runs checks, writes STL/STEP + checks.json
Needs: cadquery >= 2.4, trimesh, numpy, manifold3d

Shell frame (all numbers in mm):
  X = across the mouse width, +X = right
  Y = along the mouse length, +Y = FRONT (key / wheel end)
  Z = up, z = 0 is the bottom of the shell

Board placement (fixed by the brief): rotation 0, board shifted +2.5 mm in X.
  shell_x = (board_x - 30) + BOARD_SHIFT_X          board_x/board_y: origin board top-left, Y down
  shell_y = BOARD_FRONT_Y - board_y                 (board top edge, board_y = 0, faces the shell front)

The board geometry is read from the KiCad PCB inside ../mouse-revc-src.zip (read only, never written).
"""
from __future__ import annotations

import json
import math
import os
import re
import sys
import zipfile
from dataclasses import dataclass, field, asdict

import cadquery as cq
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.TopExp import TopExp
from OCP.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE

HERE = os.path.dirname(os.path.abspath(__file__))
ZIP_PATH = os.path.join(HERE, "..", "mouse-revc-src.zip")
PCB_IN_ZIP = "mouse-revc-src/ESP32-S31-Vision-Pro-RevC.kicad_pcb"


# ----------------------------------------------------------------------------------------------
# Parameters
# ----------------------------------------------------------------------------------------------
@dataclass
class P:
    # outer shell
    W: float = 70.0                 # outer width (X)
    L: float = 114.0                # outer length (Y)
    H_BASE: float = 14.0            # base height = split plane between base.stl and top.stl
    H_TOP: float = 34.0             # top surface
    TAPER_DEG: float = 24.0         # front / rear taper of the hump, from vertical
    WALL: float = 1.5
    FLOOR: float = 1.5
    R_PLAN_FRONT: float = 8.0       # plan corner radius, front (<= 8 keeps 1.0 mm board clearance)
    R_PLAN_REAR: float = 18.0       # plan corner radius, rear
    R_TOP_EDGE: float = 3.0         # fillet where the top / slopes meet the side walls
    R_CREASE: float = 10.0          # profile fillet at z = H_BASE, front and rear
    R_TOP_FRONT: float = 12.0       # profile fillet top / front slope
    R_TOP_REAR: float = 25.0        # profile fillet top / rear slope (palm hump)
    R_BOTTOM: float = 1.0           # bottom outer edge
    R_MIN: float = 0.5              # minimum edge radius everywhere else

    # board
    BOARD_W: float = 60.0
    BOARD_H: float = 58.0
    BOARD_T: float = 1.6
    BOARD_R: float = 2.0
    BOARD_SHIFT_X: float = 2.5
    BOARD_FRONT_Y: float = 51.0     # shell Y of the board top edge (board_y = 0)
    BOARD_Z_BOT: float = 13.5       # board bottom face

    # battery (assumed pouch cell, not verified)
    BAT_T: float = 6.0
    BAT_W: float = 60.0
    BAT_L: float = 90.0
    BAT_X: float = 0.0              # centre X
    BAT_Y: float = -1.7             # centre Y
    BAT_RIB_H: float = 2.0          # rib standoffs under the cell (floor top -> cell bottom)
    BAT_CLEAR: float = 1.0          # minimum clearance around the cell

    # screws (M2.5)
    SCREW_HEAD_D: float = 5.0       # ISO 7045 M2.5 pan head dk max 5.0 (rear board screws)
    SCREW_HEAD_K: float = 1.75      # ISO 7045 M2.5 pan head k max 1.75
    SCREW_CLEAR_D: float = 2.8
    SCREW_PILOT_D: float = 2.1      # thread-forming screw for plastics
    CSK_D: float = 5.0              # countersink for ISO 7046 / ISO 10642 M2.5 flat head (dk max 4.7)
    STANDOFF_D: float = 5.8         # base columns above the pad (wall 1.5 around 2.8 hole)
    BOSS_D: float = 5.0             # top bosses that clamp the board top (inside the M2.5 pad)
    BOSS_D_REAR: float = 6.0        # top bosses for the two rear cover screws
    PILOT_DEPTH: float = 8.0
    REAR_SCREW_X: float = 17.0
    REAR_SCREW_Y: float = -51.0
    PILOT_DEPTH_REAR: float = 6.0

    # keys
    KEY_W: float = 21.0
    KEY_L: float = 18.0
    SLOT: float = 0.6              # JLC3DP minimum clearance between moving parts (MJF/SLS nylon)
    HINGE_T: float = 1.0            # living hinge thickness
    HINGE_LEN: float = 2.0          # hinge band length along Y
    RELIEF_D: float = 1.0           # stress-relief hole at each slot end (radius 0.5)
    KEY_SOCKET_D: float = 6.0       # boss under each tongue
    KEY_SOCKET_H: float = 3.0
    KEY_HOLE_D: float = 2.6
    KEY_HOLE_DEPTH: float = 2.5
    PLUNGER_D: float = 4.5
    PLUNGER_TIP_D: float = 3.0      # PTS645 round actuator is 3.5 mm (C&K datasheet)
    PLUNGER_PEG_D: float = 2.4
    PLUNGER_PEG_L: float = 2.3
    KEY_GAP: float = 0.2            # plunger tip to switch actuator, at rest

    # switch (C&K PTS645SK43SMTR92LFS datasheet)
    SW_BODY: float = 6.0
    SW_BODY_H: float = 3.45
    SW_H: float = 4.3
    SW_ACT_D: float = 3.5

    # wheel
    WHEEL_D: float = 20.0
    WHEEL_W: float = 8.0
    AXLE_D: float = 2.0
    AXLE_STUB: float = 2.5
    WHEEL_PROUD: float = 3.5        # wheel top above the top surface
    WHEEL_OPEN_CLR: float = 0.6     # opening clearance at each wheel side
    WHEEL_OPEN_END: float = 0.6     # opening clearance at front / rear (at the top surface chord)
    DETENTS: int = 24
    DETENT_RING_W: float = 1.5
    DETENT_R_PEAK: float = 9.6      # wave valley radius 0.61 >= R_MIN so the ring edges can be rounded
    DETENT_R_VALLEY: float = 9.1
    BRACKET_T: float = 1.6
    BRACKET_LEN: float = 7.0
    AXLE_SEAT_D: float = 2.15
    AXLE_THROAT: float = 1.7
    DETENT_ANGLE_DEG: float = 35.0  # contact direction below the rear horizontal
    DETENT_NUB_R: float = 0.7
    DETENT_ARM_T: float = 1.3
    DETENT_ARM_W: float = 1.4
    DETENT_REST_CLR: float = 0.1    # nub to ring clearance with the wheel resting on a detent

    # battery clips
    CLIP_T: float = 1.3             # leaves room for the root fillet beside the floor-wall fillet
    CLIP_W: float = 5.0
    CLIP_HOOK: float = 0.9          # overlap over the cell edge (arm can flex 1.0 before the wall)
    CLIP_HOOK_T: float = 1.5
    CLIP_Y: tuple = (-38.0, -11.0)  # behind the board, so the hooks can be 1.5 thick

    # USB-C plug overmold envelope: USB-IF Type-C Compliance Document rev 1.2, critical dimensions table:
    # plug overmold/insulator width 12.35 max, height 6.5 max (also on the GCT USB4110 mating drawing)
    PLUG_W: float = 12.35
    PLUG_H: float = 6.5
    PLUG_OPEN_CLR: float = 0.4
    PLUG_STOP: float = 0.5          # overmold stops this far before the receptacle face

    @property
    def taper_dx(self):
        return (self.H_TOP - self.H_BASE) * math.tan(math.radians(self.TAPER_DEG))

    @property
    def board_z_top(self):
        return self.BOARD_Z_BOT + self.BOARD_T


p = P()


# ----------------------------------------------------------------------------------------------
# KiCad board data (read from the zip, never written)
# ----------------------------------------------------------------------------------------------
def _tokenize(s):
    i, n = 0, len(s)
    while i < n:
        c = s[i]
        if c in "()":
            yield c; i += 1
        elif c.isspace():
            i += 1
        elif c == '"':
            j = i + 1; buf = []
            while s[j] != '"':
                if s[j] == "\\":
                    buf.append(s[j + 1]); j += 2
                else:
                    buf.append(s[j]); j += 1
            yield ("S", "".join(buf)); i = j + 1
        else:
            j = i
            while j < n and not s[j].isspace() and s[j] not in "()":
                j += 1
            yield s[i:j]; i = j


def _parse(tokens):
    stack = [[]]
    for t in tokens:
        if t == "(":
            stack.append([])
        elif t == ")":
            x = stack.pop(); stack[-1].append(x)
        else:
            stack[-1].append(t[1] if isinstance(t, tuple) else t)
    return stack[0][0]


def _find(node, key):
    return [c for c in node if isinstance(c, list) and c and c[0] == key]


def read_board():
    with zipfile.ZipFile(ZIP_PATH) as z:
        txt = z.read(PCB_IN_ZIP).decode()
    tree = _parse(_tokenize(txt))
    # outline
    xs, ys = [], []
    for g in tree:
        if isinstance(g, list) and g and g[0] in ("gr_line", "gr_arc"):
            if _find(g, "layer")[0][1] != "Edge.Cuts":
                continue
            for k in ("start", "end", "mid"):
                for e in _find(g, k):
                    xs.append(float(e[1])); ys.append(float(e[2]))
    outline = (min(xs), min(ys), max(xs), max(ys))
    fps = {}
    for fp in _find(tree, "footprint"):
        at = _find(fp, "at")[0]
        x, y = float(at[1]), float(at[2]); rot = float(at[3]) if len(at) > 3 else 0.0
        ref = next((q[2] for q in _find(fp, "property") if q[1] == "Reference"), "")
        a = math.radians(rot)
        def tr(px, py):  # KiCad footprint local -> page
            return (x + px * math.cos(a) + py * math.sin(a), y - px * math.sin(a) + py * math.cos(a))
        pts = {"F.CrtYd": [], "F.Fab": []}
        for g in fp:
            if isinstance(g, list) and g and g[0] in ("fp_line", "fp_rect", "fp_poly", "fp_circle", "fp_arc"):
                L = _find(g, "layer")
                if not L or L[0][1] not in pts:
                    continue
                if g[0] == "fp_poly":
                    for xy in _find(_find(g, "pts")[0], "xy"):
                        pts[L[0][1]].append(tr(float(xy[1]), float(xy[2])))
                elif g[0] == "fp_circle":
                    c = _find(g, "center")[0]; e = _find(g, "end")[0]
                    cx, cy = float(c[1]), float(c[2]); r = math.hypot(float(e[1]) - cx, float(e[2]) - cy)
                    for dx, dy in ((-r, -r), (r, r), (-r, r), (r, -r)):
                        pts[L[0][1]].append(tr(cx + dx, cy + dy))
                else:
                    for k in ("start", "end", "mid"):
                        for e in _find(g, k):
                            pts[L[0][1]].append(tr(float(e[1]), float(e[2])))
        def bb(v):
            if not v:
                return None
            return (min(q[0] for q in v), min(q[1] for q in v), max(q[0] for q in v), max(q[1] for q in v))
        fps[ref] = dict(ref=ref, fp=fp[1], x=x, y=y, rot=rot, crtyd=bb(pts["F.CrtYd"]), fab=bb(pts["F.Fab"]),
                        pads=[(q[1], *tr(float(_find(q, "at")[0][1]), float(_find(q, "at")[0][2])))
                              for q in _find(fp, "pad")])
    return outline, fps


OUTLINE, FPS = read_board()
BX0, BY0 = OUTLINE[0], OUTLINE[1]   # board top-left in KiCad page coordinates


def b2s(bx, by):
    """board frame (origin top-left, X right, Y down) -> shell X, Y"""
    return (bx - p.BOARD_W / 2 + p.BOARD_SHIFT_X, p.BOARD_FRONT_Y - by)


def page2s(px, py):
    return b2s(px - BX0, py - BY0)


def board_facts():
    """Verify the fixed inputs of the brief against the PCB file. Raises on mismatch."""
    f = {}
    ow, oh = OUTLINE[2] - OUTLINE[0], OUTLINE[3] - OUTLINE[1]
    f["outline_mm"] = (round(ow, 3), round(oh, 3))
    assert abs(ow - p.BOARD_W) < 1e-6 and abs(oh - p.BOARD_H) < 1e-6, f["outline_mm"]
    for ref, exp in (("SW_L1", (6.30, 31.50)), ("SW_R1", (42.15, 30.00))):
        q = FPS[ref]; got = (round(q["x"] - BX0, 3), round(q["y"] - BY0, 3))
        f[ref] = got
        assert abs(got[0] - exp[0]) < 1e-3 and abs(got[1] - exp[1]) < 1e-3, (ref, got)
        assert abs(q["rot"]) < 1e-6
    holes = sorted((round(FPS[h]["x"] - BX0, 3), round(FPS[h]["y"] - BY0, 3)) for h in ("H1", "H2", "H3", "H4"))
    f["holes"] = holes
    for hx, hy in holes:
        assert abs(abs(hx - 30) - 26.6) < 1e-3 and abs(abs(hy - 29) - 25.6) < 1e-3, holes
    j5 = FPS["J5"]
    f["J5_origin"] = (round(j5["x"] - BX0, 3), round(j5["y"] - BY0, 3), j5["rot"])
    # mouth = Fab outline edge with the smallest board X (signal pads sit at larger X -> opening faces -X)
    fab = j5["fab"]
    sig_x = [x for (n, x, y) in j5["pads"] if n and n[0] in "AB"]
    f["J5_fab_x"] = (round(fab[0] - BX0, 3), round(fab[2] - BX0, 3))
    f["J5_signal_pad_x"] = round(min(sig_x) - BX0, 3)
    assert min(sig_x) > fab[0] + 3, "J5 opening is not on its -X side"
    f["J5_mouth_board_x"] = round(fab[0] - BX0, 3)
    f["J5_centre_board_y"] = round((fab[1] + fab[3]) / 2 - BY0, 3)
    return f


# ----------------------------------------------------------------------------------------------
# Derived layout
# ----------------------------------------------------------------------------------------------
SW = {k: page2s(FPS[k]["x"], FPS[k]["y"]) for k in ("SW_L1", "SW_R1")}
HOLES = {k: page2s(FPS[k]["x"], FPS[k]["y"]) for k in ("H1", "H2", "H3", "H4")}
J5_MOUTH_X = page2s(FPS["J5"]["fab"][0], 0)[0]
J5_Y = page2s(0, (FPS["J5"]["fab"][1] + FPS["J5"]["fab"][3]) / 2)[1]
J5_Z = p.board_z_top + 1.68            # USB4110 drawing: opening centre 1.68 above the PCB
SW_TOP_Z = p.board_z_top + p.SW_H
WHEEL_X = (SW["SW_L1"][0] + SW["SW_R1"][0]) / 2
WHEEL_Y = (SW["SW_L1"][1] + SW["SW_R1"][1]) / 2
WHEEL_Z = p.H_TOP + p.WHEEL_PROUD - p.WHEEL_D / 2
FRONT_HOLES = ("H1", "H2")             # board_y = 3.4 -> front
REAR_HOLES = ("H3", "H4")
CEIL_Z = p.H_TOP - p.WALL


# ----------------------------------------------------------------------------------------------
# Geometry helpers
# ----------------------------------------------------------------------------------------------
def offset_polygon(pts, d):
    """Inward offset of a convex CCW polygon by d (straight edges only)."""
    n = len(pts)
    lines = []
    for i in range(n):
        (x0, y0), (x1, y1) = pts[i], pts[(i + 1) % n]
        L = math.hypot(x1 - x0, y1 - y0)
        nx, ny = -(y1 - y0) / L, (x1 - x0) / L
        lines.append(((x0 + nx * d, y0 + ny * d), (x1 - x0, y1 - y0)))
    out = []
    for i in range(n):
        (p1, d1), (p2, d2) = lines[i - 1], lines[i]
        den = d1[0] * d2[1] - d1[1] * d2[0]
        t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / den
        out.append((p1[0] + d1[0] * t, p1[1] + d1[1] * t))
    return out


def rounded_poly(wp: cq.Workplane, pts, radii):
    """Closed polygon with a fillet radius per vertex (0 = sharp), drawn on wp's plane."""
    n = len(pts)
    tang = []
    for i in range(n):
        P0 = cq.Vector(*pts[i - 1], 0); P1 = cq.Vector(*pts[i], 0); P2 = cq.Vector(*pts[(i + 1) % n], 0)
        r = radii[i]
        if r <= 0:
            tang.append((P1, None, P1)); continue
        u1 = (P0 - P1).normalized(); u2 = (P2 - P1).normalized()
        th = math.acos(max(-1, min(1, u1.dot(u2)))) / 2
        t = r / math.tan(th)
        b = (u1 + u2).normalized()
        C = P1 + b * (r / math.sin(th))
        tang.append((P1 + u1 * t, C - b * r, P1 + u2 * t))
    cur = tang[0][2]
    w = wp.moveTo(cur.x, cur.y)
    for i in list(range(1, n)) + [0]:
        a, m, c = tang[i]
        if (a - cur).Length > 1e-9:
            w = w.lineTo(a.x, a.y)
        cur = a
        if m is not None:
            w = w.threePointArc((m.x, m.y), (c.x, c.y))
            cur = c
    return w.close()


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl_z(x, y, z0, z1, d):
    return cq.Workplane("XY").circle(d / 2).extrude(z1 - z0).translate((x, y, z0))


def cyl_x(x0, x1, y, z, d):
    return cq.Workplane("YZ").circle(d / 2).extrude(x1 - x0).translate((x0, y, z))


def solid(wp):
    """Unwrap a Workplane / single-solid compound to a cq.Solid (fillet() can return a Solid-typed
    wrapper around a TopoDS compound, which later booleans reject)."""
    v = wp.val() if isinstance(wp, cq.Workplane) else wp
    if v.ShapeType() == "Compound":
        s = v.Solids()
        if len(s) == 1:
            return cq.Solid(s[0].wrapped)
    return v


def face_normal_angle(edge, f1, f2):
    pmid = edge.positionAt(0.5)
    try:
        n1 = f1.normalAt(pmid); n2 = f2.normalAt(pmid)
    except Exception:
        return 0.0
    return math.degrees(n1.getAngle(n2))


def edge_faces(shape):
    m = TopTools_IndexedDataMapOfShapeListOfShape()
    TopExp.MapShapesAndAncestors_s(shape.wrapped, TopAbs_EDGE, TopAbs_FACE, m)
    out = []
    for i in range(1, m.Extent() + 1):
        e = cq.Edge(m.FindKey(i))
        fl = [cq.Face(f) for f in m.FindFromIndex(i)]
        out.append((e, fl))
    return out


def sharp_edges(shape, tol_deg=5.0, zmin=-1e9):
    """Edges between two faces that meet at an angle (not tangent)."""
    res = []
    for e, fl in edge_faces(shape):
        if len(fl) < 2:
            continue
        if e.BoundingBox().zmin < zmin:
            continue
        if fl[0].isSame(fl[1]):      # seam edge
            continue
        if face_normal_angle(e, fl[0], fl[1]) > tol_deg:
            res.append(e)
    return res


def _fillet_is_safe(shape, edges, r):
    """OCC can segfault inside fillet(); try it in a forked child first (Linux)."""
    if not hasattr(os, "fork"):
        return True
    pid = os.fork()
    if pid == 0:
        code = 1
        try:
            out = shape.fillet(r, edges)
            code = 0 if out.isValid() else 1
        except BaseException:
            code = 1
        os._exit(code)
    _, status = os.waitpid(pid, 0)
    return os.WIFEXITED(status) and os.WEXITSTATUS(status) == 0


def fillet_edges(shape, edges, r, log=None, name=""):
    """Fillet edges; on failure bisect the list. Returns (shape, failed_edges)."""
    if not edges:
        return shape, []
    try:
        if _fillet_is_safe(shape, edges, r):
            out = solid(shape.fillet(r, edges))
            if out.isValid():
                return out, []
    except Exception:
        pass
    if len(edges) == 1:
        return shape, edges
    h = len(edges) // 2
    shape, f1 = fillet_edges(shape, edges[:h], r, log, name)
    # edges of the second half may have changed identity; re-find by midpoint
    rest = rematch(shape, edges[h:])
    shape, f2 = fillet_edges(shape, rest, r, log, name)
    return shape, f1 + f2


def rematch(shape, edges):
    mids = [e.positionAt(0.5) for e in edges]
    cur = shape.Edges()
    out = []
    for m in mids:
        best = None; bd = 1e9
        for e in cur:
            d = (e.positionAt(0.5) - m).Length
            if d < bd:
                bd, best = d, e
        if best is not None and bd < 1e-3:
            out.append(best)
    return out


def round_all(shape, r, tol_deg=5.0, rounds=3, name=""):
    """Try to give every sharp edge at least radius r."""
    shape = solid(shape)
    failed = []
    for _ in range(rounds):
        es = sharp_edges(shape, tol_deg)
        if not es:
            break
        shape, failed = fillet_edges(shape, es, r, name=name)
        if len(failed) == len(es):
            break
    return shape, sharp_edges(shape, tol_deg)


def min_dist(a, b):
    """Distance between the boundaries (faces) of a and b. Overlap is checked separately by volume,
    because the solid-inside test in BRepExtrema can misclassify points near spline faces."""
    fa = cq.Compound.makeCompound(a.Faces())
    fb = cq.Compound.makeCompound(b.Faces())
    d = BRepExtrema_DistShapeShape(fa.wrapped, fb.wrapped)
    d.Perform()
    return d.Value()


def common_volume(a, b):
    try:
        c = a.intersect(b)
        return c.Volume()
    except Exception:
        return float("nan")


# ----------------------------------------------------------------------------------------------
# Outer body
# ----------------------------------------------------------------------------------------------
def _body(off=0.0, z_bottom=0.0):
    """Outer body (off = 0) or the cavity (off = WALL, bottom pushed below z = 0)."""
    hl, dx = p.L / 2, p.taper_dx
    # side profile in the YZ plane (local x = Y, local y = Z)
    prof = [(-hl, z_bottom), (hl, z_bottom), (hl, p.H_BASE), (hl - dx, p.H_TOP), (-hl + dx, p.H_TOP), (-hl, p.H_BASE)]
    radii = [0, 0, p.R_CREASE, p.R_TOP_FRONT, p.R_TOP_REAR, p.R_CREASE]
    if off:
        prof = offset_polygon(prof, off)
        radii = [max(r - off, 0) for r in radii]
    w = rounded_poly(cq.Workplane("YZ"), prof, radii)
    loaf = w.extrude(p.W / 2 + 1, both=True)
    hw = p.W / 2
    plan = [(-hw, -hl), (hw, -hl), (hw, hl), (-hw, hl)]
    pr = [p.R_PLAN_REAR, p.R_PLAN_REAR, p.R_PLAN_FRONT, p.R_PLAN_FRONT]
    if off:
        plan = offset_polygon(plan, off)
        pr = [r - off for r in pr]
    w2 = rounded_poly(cq.Workplane("XY").workplane(offset=z_bottom), plan, pr)
    prism = w2.extrude(p.H_TOP + 5 - z_bottom)
    body = solid(loaf.intersect(prism))
    top_edges = sharp_edges(body, 5.0, zmin=z_bottom + off + 0.5)
    # cavity fillet radius 2.2 (> 3 - 1.5): wall across the 3 mm fillet >= 1.5 (1.79 at 45 deg), and its
    # tangent line (x = -31.3) does not coincide with the left tongue edge (x = -31.7)
    body = body.fillet(p.R_TOP_EDGE - off + (0.7 if off else 0.0), top_edges)
    return body


def outer_body():
    return _body(0.0, 0.0)


def cavity_body():
    return _body(p.WALL, -5.0)


def plan_inner_offset(d):
    """plan outline offset inward by d, as a wire workplane (for floor plates etc.)"""
    hw, hl = p.W / 2 - d, p.L / 2 - d
    plan = [(-hw, -hl), (hw, -hl), (hw, hl), (-hw, hl)]
    pr = [p.R_PLAN_REAR - d, p.R_PLAN_REAR - d, p.R_PLAN_FRONT - d, p.R_PLAN_FRONT - d]
    return rounded_poly(cq.Workplane("XY"), plan, pr)
# ----------------------------------------------------------------------------------------------
# Rounding-aware boolean helpers
# ----------------------------------------------------------------------------------------------
FILLET_FAILS = {}
REDUCED = []          # (tag, radius, midpoint, length) for edges that only took a smaller fillet
FALLBACK_R = (0.4, 0.3)


def _near(e, bb, pad):
    c = e.positionAt(0.5)
    return (bb.xmin - pad <= c.x <= bb.xmax + pad and bb.ymin - pad <= c.y <= bb.ymax + pad
            and bb.zmin - pad <= c.z <= bb.zmax + pad)


def pre_round(f, r=None, skip=None):
    """Round a feature's own edges before it is joined (skip = edge filter to leave sharp)."""
    r = r or p.R_MIN
    f = solid(f)
    es = [e for e in f.Edges() if not (skip and skip(e))]
    try:
        if _fillet_is_safe(f, es, r):
            g = solid(f.fillet(r, es))
            if g.isValid():
                return g
    except Exception:
        pass
    g, _ = fillet_edges(f, es, r)
    return g


def round_near(body, bb, tag, r=None, pad=0.6, rounds=3):
    r = r or p.R_MIN
    prev = None
    failed = []
    for _ in range(rounds):
        es = [e for e in sharp_edges(body) if _near(e, bb, pad)]
        if not es or (prev is not None and len(es) >= prev):
            break
        prev = len(es)
        body, failed = fillet_edges(body, es, r)
    # leftovers: one edge at a time (re-detected after every success), R_MIN first, then smaller radii
    for rr in (r,) + FALLBACK_R:
        tried = set()
        while True:
            es = [e for e in sharp_edges(body) if _near(e, bb, pad)]
            es = [e for e in es if tuple(round(c, 3) for c in e.positionAt(0.5).toTuple()) not in tried]
            if not es:
                break
            e = es[0]
            key = tuple(round(c, 3) for c in e.positionAt(0.5).toTuple())
            tried.add(key)
            if _fillet_is_safe(body, [e], rr):
                try:
                    nb = solid(body.fillet(rr, [e]))
                    if nb.isValid():
                        body = nb
                        if rr < r:
                            REDUCED.append((tag, rr, tuple(round(c, 2) for c in key), round(e.Length(), 2)))
                except Exception:
                    pass
    failed = [e for e in sharp_edges(body) if _near(e, bb, pad)]
    if failed:
        FILLET_FAILS.setdefault(tag, []).extend(failed)
    return solid(body)


def add(body, feat, tag, r=None, pre=True):
    """Fuse a (pre-rounded) feature and round the new junction edges."""
    f = pre_round(feat, r) if pre else solid(feat)
    body = solid(body.fuse(f))
    return round_near(body, f.BoundingBox(), tag, r)


def sub(body, tool, tag, r=None):
    """Cut and round the new edges around the cut."""
    t = solid(tool)
    body = solid(body.cut(t))
    return round_near(body, t.BoundingBox(), tag, r)


# ----------------------------------------------------------------------------------------------
# Shared features
# ----------------------------------------------------------------------------------------------
def hollow():
    """Outer body minus the cavity above the floor (floor-wall corner rounded R_MIN)."""
    ob = outer_body()
    cav = solid(cavity_body().cut(box(-60, 60, -80, 80, -10, p.FLOOR).val()))
    bot = [e for e in cav.Edges() if abs(e.BoundingBox().zmax - p.FLOOR) < 1e-6 and abs(e.BoundingBox().zmin - p.FLOOR) < 1e-6]
    cav = cav.fillet(p.R_MIN, bot)
    return ob, solid(ob.cut(cav))


def usb_opening():
    """Pocket through the left wall so the USB-C plug overmold reaches J5 (stops PLUG_STOP short)."""
    w = p.PLUG_W + 2 * p.PLUG_OPEN_CLR
    h = p.PLUG_H + 2 * p.PLUG_OPEN_CLR
    r = h / 2 - 0.01
    x0, x1 = -p.W / 2 - 5, -p.W / 2 + p.WALL + 0.5   # through the wall only; inside is open cavity
    return (cq.Workplane("YZ").workplane(offset=x0).center(J5_Y, J5_Z)
            .rect(w, h).extrude(x1 - x0).edges("|X").fillet(r))


def plug_envelope():
    """USB-C plug overmold at full insertion (for the clearance check)."""
    x1 = J5_MOUTH_X - p.PLUG_STOP
    return (cq.Workplane("YZ").workplane(offset=-p.W / 2 - 20).center(J5_Y, J5_Z)
            .rect(p.PLUG_W, p.PLUG_H).extrude(x1 + p.W / 2 + 20).edges("|X").fillet(p.PLUG_H / 2 - 0.05))


def round_split(part, z):
    """Round the edges of the split face (z = H_BASE) and anything else left sharp there."""
    es = [e for e in sharp_edges(part) if abs(e.positionAt(0.5).z - z) < 0.05]
    part, failed = fillet_edges(part, es, p.R_MIN)
    if failed:
        FILLET_FAILS.setdefault("split", []).extend(failed)
    return solid(part)


# ----------------------------------------------------------------------------------------------
# Base (removable bottom cover, holds the battery)
# ----------------------------------------------------------------------------------------------
BAT_Z0 = p.FLOOR + p.BAT_RIB_H
BAT_BOX = (p.BAT_X - p.BAT_W / 2, p.BAT_X + p.BAT_W / 2, p.BAT_Y - p.BAT_L / 2, p.BAT_Y + p.BAT_L / 2,
           BAT_Z0, BAT_Z0 + p.BAT_T)
COVER_SCREWS = [HOLES["H1"], HOLES["H2"], (-p.REAR_SCREW_X, p.REAR_SCREW_Y), (p.REAR_SCREW_X, p.REAR_SCREW_Y)]
EMBED = 0.6   # features are sunk this far into the floor / ceiling so their own rounded edges are buried


def base_part(hol):
    ob = outer_body()
    body = solid(hol.intersect(box(-60, 60, -80, 80, -1, p.H_BASE).val()))
    body = round_split(body, p.H_BASE)
    x0, x1, y0, y1, z0, z1 = BAT_BOX
    c = p.BAT_CLEAR
    fz = p.FLOOR - EMBED
    # rib standoffs under the cell
    for rx in (-22, -8, 8, 22):
        body = add(body, box(rx - 1, rx + 1, y0 + 2, y1 - 2, fz, z0), "rib")
    # rear end stop
    body = add(body, box(-12, 12, y0 - c - 1.5, y0 - c, fz, z0 + 3.5), "stop")
    # front end stop with a lip over the cell's front edge (slide the cell under it, then press the rear down)
    ya, yb = y1 + c, y1 + c + 1.5
    zl0, zl1 = z1 + c, z1 + c + p.CLIP_HOOK_T   # lip: BAT_CLEAR above the cell
    prof = [(ya, fz), (yb, fz), (yb, zl1), (y1 - p.CLIP_HOOK, zl1), (y1 - p.CLIP_HOOK, zl0), (ya, zl0)]
    lip = rounded_poly(cq.Workplane("YZ"), prof, [0, 0, 0.5, 0.5, 0.5, 1.0]).extrude(14, both=True)
    lip = pre_round(lip, p.R_MIN, skip=lambda e: e.BoundingBox().zmax < p.FLOOR)
    body = add(body, lip, "stop", pre=False)
    # cover screw columns (front: up to the board bottom, rear: up to the split plane)
    for i, (sx, sy) in enumerate(COVER_SCREWS):
        ztop = p.BOARD_Z_BOT if i < 2 else p.H_BASE
        body = add(body, cyl_z(sx, sy, fz, ztop, p.STANDOFF_D), "column")
    # battery clips (4): flexing arms beside the long edges, hook BAT_CLEAR above the cell
    for sgn in (-1, 1):
        for cy in p.CLIP_Y:
            xin = sgn * (p.BAT_W / 2 + c)
            xo = xin + sgn * p.CLIP_T
            zh0 = z1 + c
            zh1 = zh0 + p.CLIP_HOOK_T
            xh = sgn * (p.BAT_W / 2 - p.CLIP_HOOK)
            prof = [(xo, fz), (xin, fz), (xin, zh0), (xh, zh0), (xh, zh1), (xo, zh1)]
            radii = [0, 0, 1.0, 0.5, 0.5, 0.5]
            if sgn > 0:
                prof, radii = prof[::-1], radii[::-1]
            clip = (rounded_poly(cq.Workplane("XZ"), prof, radii).extrude(p.CLIP_W / 2, both=True)
                    .translate((0, cy, 0)))
            clip = pre_round(clip, p.R_MIN, skip=lambda e: e.BoundingBox().zmax < p.FLOOR)
            body = add(body, clip, "clip", pre=False)
    # holes: countersink from below + clearance hole up through the column
    for i, (sx, sy) in enumerate(COVER_SCREWS):
        csk = cq.Solid.makeCone(p.CSK_D / 2 + 1.0, 0.0, p.CSK_D / 2 + 1.0, cq.Vector(sx, sy, -1.0))
        body = sub(body, csk, "csk")
        body = sub(body, cyl_z(sx, sy, -1, p.H_BASE + 1, p.SCREW_CLEAR_D), "hole")
    body = sub(body, usb_opening(), "usb")
    # outer bottom edge
    bot = [e for e in sharp_edges(body) if e.BoundingBox().zmax < 1e-3]
    outer_bot = [e for e in bot if e.BoundingBox().xmax - e.BoundingBox().xmin > 3 or e.BoundingBox().ymax - e.BoundingBox().ymin > 3]
    body, failed = fillet_edges(body, [e for e in outer_bot if not _hole_edge(e)], p.R_BOTTOM)
    if failed:
        FILLET_FAILS.setdefault("bottom", []).extend(failed)
    return solid(body)


def _hole_edge(e):
    c = e.positionAt(0.5)
    return any(math.hypot(c.x - sx, c.y - sy) < p.CSK_D / 2 + 1.5 for sx, sy in COVER_SCREWS)


# ----------------------------------------------------------------------------------------------
# Top (hump, key tongues, wheel mounts, board bosses)
# ----------------------------------------------------------------------------------------------
def tongue_rect(k):
    sx, sy = SW[k]
    return (sx - p.KEY_W / 2, sx + p.KEY_W / 2, sy - p.KEY_L / 2, sy + p.KEY_L / 2)


def wheel_open():
    half_chord = math.sqrt((p.WHEEL_D / 2) ** 2 - (CEIL_Z - WHEEL_Z) ** 2)
    hy = half_chord + p.WHEEL_OPEN_END
    hx = p.WHEEL_W / 2 + p.WHEEL_OPEN_CLR
    return (WHEEL_X - hx, WHEEL_X + hx, WHEEL_Y - hy, WHEEL_Y + hy)


def detent_geometry():
    """Ring profile r(t) = R0 - A cos(n t) and the nub position that leaves DETENT_REST_CLR at rest."""
    n = p.DETENTS
    A = (p.DETENT_R_PEAK - p.DETENT_R_VALLEY) / 2
    R0 = (p.DETENT_R_PEAK + p.DETENT_R_VALLEY) / 2
    phi = math.radians(p.DETENT_ANGLE_DEG)

    def r(t):
        return R0 - A * math.cos(n * t)

    def min_gap(d):
        best = 1e9
        for i in range(-400, 401):
            t = i * (math.pi / 6) / 400
            best = min(best, math.hypot(r(t) * math.cos(t) - d, r(t) * math.sin(t)) - p.DETENT_NUB_R)
        return best
    lo, hi = p.DETENT_R_VALLEY, p.DETENT_R_PEAK + 2
    for _ in range(50):
        mid = (lo + hi) / 2
        if min_gap(mid) < p.DETENT_REST_CLR:
            lo = mid
        else:
            hi = mid
    d = hi
    k = n / R0
    return dict(R0=R0, A=A, d=d, nub_y=WHEEL_Y - d * math.cos(phi), nub_z=WHEEL_Z - d * math.sin(phi),
                interference_on_peak=p.DETENT_R_PEAK + p.DETENT_NUB_R - d, phi=phi,
                valley_radius=1 / (A * k * k))


HINGE_GROOVE_R = 1.25


def ring_x():
    x_left = WHEEL_X - p.WHEEL_W / 2
    return x_left, x_left + p.DETENT_RING_W


def top_part(hol):
    ob = outer_body()
    t = solid(hol.intersect(box(-60, 60, -80, 80, p.H_BASE, 50).val()))
    # USB notch first, then round its edges together with the split edges (one fillet operation)
    u = solid(usb_opening())
    t = solid(t.cut(u))
    es = [e for e in sharp_edges(t) if _near(e, u.BoundingBox(), 0.6) or abs(e.positionAt(0.5).z - p.H_BASE) < 0.05]
    t, failed = fillet_edges(t, es, p.R_MIN)
    if failed:
        FILLET_FAILS.setdefault("split+usb", []).extend(failed)
    ox0, ox1, oy0, oy1 = wheel_open()
    # ---- cuts in the skin first (slots, hinge grooves, wheel opening)
    for k in SW:
        x0, x1, y0, y1 = tongue_rect(k)
        s = p.SLOT
        zc0, zc1 = CEIL_Z - 1.0, p.H_TOP + 1
        rg = HINGE_GROOVE_R
        ye = y0 - rg                          # side slots run past the hinge groove
        slot = (box(x0 - s, x0, ye, y1 + s, zc0, zc1).union(box(x1, x1 + s, ye, y1 + s, zc0, zc1))
                .union(box(x0 - s, x1 + s, y1, y1 + s, zc0, zc1))
                .union(cyl_z(x0 - s / 2, ye, zc0, zc1, p.RELIEF_D)).union(cyl_z(x1 + s / 2, ye, zc0, zc1, p.RELIEF_D)))
        t = sub(t, slot, "slot")
        # living hinge: cylindrical groove under the tongue's rear edge, leaves HINGE_T of skin
        zc = p.H_TOP - p.HINGE_T - rg
        t = sub(t, cyl_x(x0, x1, y0, zc, 2 * rg), "hinge")
    t = sub(t, box(ox0, ox1, oy0, oy1, CEIL_Z - 1.0, p.H_TOP + 1).edges("|Z").fillet(1.0), "wheel_open")
    # ---- features hanging from the ceiling
    for k in SW:
        sx, sy = SW[k]
        t = add(t, cyl_z(sx, sy, CEIL_Z - p.KEY_SOCKET_H, CEIL_Z + EMBED, p.KEY_SOCKET_D), "socket")
    for h, (hx, hy) in HOLES.items():
        boss = solid(cyl_z(hx, hy, p.board_z_top, p.H_TOP + 2, p.BOSS_D).intersect(cq.Workplane().add(ob)))
        t = add(t, boss, "boss", pre=False)
    for sx in (-p.REAR_SCREW_X, p.REAR_SCREW_X):
        boss = pre_round(cyl_z(sx, p.REAR_SCREW_Y, p.H_BASE, p.H_TOP - 6, p.BOSS_D_REAR), p.R_MIN)
        boss = solid(boss.intersect(ob))
        t = add(t, boss, "boss", pre=False)
    bz0 = WHEEL_Z - 2.5
    for xa, xb in ((ox0 - p.BRACKET_T, ox0), (ox1, ox1 + p.BRACKET_T)):
        t = add(t, box(xa, xb, WHEEL_Y - p.BRACKET_LEN / 2, WHEEL_Y + p.BRACKET_LEN / 2, bz0, CEIL_Z + EMBED),
                "bracket", r=0.5)
    # detent: bracket block behind the opening, cantilever arm, nub
    dg = detent_geometry()
    rx0, rx1 = ring_x()
    ax0, ax1 = rx0 + 0.1, rx1 - 0.1
    arm_y0 = oy0 - 0.3 - 1.6 - 5.0
    ny, nz, rn = dg["nub_y"], dg["nub_z"], p.DETENT_NUB_R
    top_z = nz + rn                        # the nub is the arm's rounded upper-front corner (centre ny, nz)
    bot_z = top_z - p.DETENT_ARM_T
    prof = [(arm_y0, CEIL_Z + EMBED), (arm_y0, bot_z), (ny + rn, bot_z), (ny + rn, top_z),
            (arm_y0 + 1.6, top_z), (arm_y0 + 1.6, CEIL_Z + EMBED)]
    arm = (rounded_poly(cq.Workplane("YZ").workplane(offset=ax0), prof, [0, 0.5, 0.5, rn, 1.0, 0])
           .extrude(ax1 - ax0))
    arm = pre_round(arm, p.R_MIN, skip=lambda e: e.BoundingBox().zmin > CEIL_Z)
    t = add(t, arm, "detent", pre=False)
    # ---- holes
    for xa, xb in ((ox0 - p.BRACKET_T - 1, ox0 + 0.01), (ox1 - 0.01, ox1 + p.BRACKET_T + 1)):
        seat = cyl_x(xa, xb, WHEEL_Y, WHEEL_Z, p.AXLE_SEAT_D).union(
            box(xa, xb, WHEEL_Y - p.AXLE_THROAT / 2, WHEEL_Y + p.AXLE_THROAT / 2, bz0 - 1, WHEEL_Z))
        t = sub(t, seat, "axle_seat")
    for k in SW:
        sx, sy = SW[k]
        t = sub(t, cyl_z(sx, sy, CEIL_Z - p.KEY_SOCKET_H - 1, CEIL_Z - p.KEY_SOCKET_H + p.KEY_HOLE_DEPTH,
                         p.KEY_HOLE_D), "socket_hole")
    for h, (hx, hy) in HOLES.items():
        t = sub(t, cyl_z(hx, hy, p.board_z_top - 1, p.board_z_top + p.PILOT_DEPTH, p.SCREW_PILOT_D), "pilot")
    for sx in (-p.REAR_SCREW_X, p.REAR_SCREW_X):
        t = sub(t, cyl_z(sx, p.REAR_SCREW_Y, p.H_BASE - 1, p.H_BASE + p.PILOT_DEPTH_REAR, p.SCREW_PILOT_D), "pilot")
    return solid(t)


# ----------------------------------------------------------------------------------------------
# Wheel and keys (plungers)
# ----------------------------------------------------------------------------------------------
def revolve_x(profile_rz, radii, x_shift, y, z):
    """Revolve a rounded (x, r) profile about the X axis through (y, z)."""
    w = rounded_poly(cq.Workplane("XY"), profile_rz, radii)
    s = w.revolve(360, (0, 0, 0), (1, 0, 0))
    return solid(s).translate(cq.Vector(x_shift, y, z))


def wheel_part():
    """Tire (revolved, rounded) + hub + wavy detent ring + 2 mm axle stubs, one piece."""
    dg = detent_geometry()
    xl, xr = WHEEL_X - p.WHEEL_W / 2, WHEEL_X + p.WHEEL_W / 2
    rx0, rx1 = ring_x()
    gap = 1.2
    R = p.WHEEL_D / 2
    ra = p.AXLE_D / 2
    stub = p.AXLE_STUB
    hub_r = 3.0
    # one revolved profile: left stub, hub through the ring and gap, tire, right stub
    prof = [(xl - stub, 0), (xr + stub, 0), (xr + stub, ra), (xr, ra), (xr, R), (rx1 + gap, R),
            (rx1 + gap, hub_r), (xl, hub_r), (xl, ra), (xl - stub, ra)]
    radii = [0, 0, 0.5, 0.5, 1.0, 1.0, 0.5, 0, 0.5, 0.5]
    body = revolve_x([(x, r) for x, r in prof], radii, 0, WHEEL_Y, WHEEL_Z)
    # detent ring: valleys face the nub direction
    n, pts = p.DETENTS, []
    base = math.pi + dg["phi"]
    for i in range(n * 16):
        a = 2 * math.pi * i / (n * 16)
        rr = dg["R0"] - dg["A"] * math.cos(n * a)
        pts.append((WHEEL_Y + rr * math.cos(base + a), WHEEL_Z + rr * math.sin(base + a)))
    ring = solid(cq.Workplane("YZ").workplane(offset=rx0).spline(pts, periodic=True).close().extrude(rx1 - rx0))
    ring = ring.fillet(p.R_MIN, [e for e in ring.Edges() if e.geomType() != "LINE"])
    w = solid(body.fuse(ring))
    w = round_near(w, w.BoundingBox(), "wheel")
    return w


def plunger(k):
    """Revolved key plunger: tip on the switch actuator, 45 deg taper, body, shoulder, peg into the socket."""
    sx, sy = SW[k]
    zt = SW_TOP_Z + p.KEY_GAP
    zb = CEIL_Z - p.KEY_SOCKET_H
    rt, rb, rp = p.PLUNGER_TIP_D / 2, p.PLUNGER_D / 2, p.PLUNGER_PEG_D / 2
    prof = [(0, zt), (rt, zt), (rt, zt + 0.8), (rb, zt + 0.8 + (rb - rt)), (rb, zb), (rp, zb),
            (rp, zb + p.PLUNGER_PEG_L), (0, zb + p.PLUNGER_PEG_L)]
    radii = [0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0]
    w = rounded_poly(cq.Workplane("XZ"), prof, radii)
    s = solid(w.revolve(360, (0, 0, 0), (0, 1, 0)))
    return s.translate(cq.Vector(sx, sy, 0))


def keys_part():
    return cq.Compound.makeCompound([plunger("SW_L1"), plunger("SW_R1")])


# ----------------------------------------------------------------------------------------------
# Reference models (board, switches, battery, screws, plug) - for checks and assembly.step only
# ----------------------------------------------------------------------------------------------
# Component heights above the board top (mm). Source per entry.
HEIGHTS = [
    # (footprint regex, height, source)
    (r"PTS645", 4.3, "C&K PTS645 datasheet (switches are modelled separately)"),
    (r"USB_C_Receptacle_GCT_USB4110", 3.26, "GCT USB4110 drawing rev B4"),
    (r"PinHeader_1x04_P2.54mm_Vertical", 8.54, "KiCad 3D model"),
    (r"JST_GH_SM0\dB", 4.25, "KiCad 3D model"),
    (r"JST_PH_S2B-PH-SM4", 5.5, "Distributor data: Digi-Key insulation height 5.51 mm, JLCPCB Z-height 5.5 mm "
                               "(JST drawing gives no height; no KiCad 3D model exists for this part)"),
    (r"Hirose_FH12", 2.0, "KiCad 3D model"),
    (r"microSD_HC_Hirose_DM3AT", 1.68, "KiCad 3D model"),
    (r"SKRTLAE010", 3.3, "Alps Alpine product page SKRTLAE010: product height 3.3 mm (W x D x H 4.5 x 3.4 x 3.3)"),
    (r"ESP32-S31-WROOM-1U", 3.2, "Espressif ESP32-S31-WROOM-1/1U datasheet: -1U module 18.0 x 20.5 x 3.2 mm"),
    (r"SOT-23-6", 1.55, "KiCad 3D model"),
    (r"SOT-23", 1.2, "KiCad 3D model"),
    (r"SOT-363", 1.1, "KiCad 3D model"),
    (r"L_1008", 1.2, "KiCad 3D model"),
    (r"D_SOD-323", 1.11, "KiCad 3D model"),
    (r"C_0603", 0.8, "KiCad 3D model"),
    (r"C_0402", 0.5, "KiCad 3D model"),
    (r"R_0\d0\d", 0.45, "KiCad 3D model"),
    (r"TestPoint|SolderJumper|MountingHole", 0.1, "pad only"),
    (r".", 1.0, "ASSUMED: small QFN/DFN/SON, 1.0 used"),
]


def comp_height(fp):
    for rx, h, src in HEIGHTS:
        if re.search(rx, fp):
            return h, src
    return 1.0, "default"


def board_model():
    """Board slab with holes + every component as a box: F.Fab body outline (or courtyard) x height."""
    x0, y0 = b2s(0, 0)
    slab = (rounded_poly(cq.Workplane("XY").workplane(offset=p.BOARD_Z_BOT),
                         [(x0, y0 - p.BOARD_H), (x0 + p.BOARD_W, y0 - p.BOARD_H), (x0 + p.BOARD_W, y0), (x0, y0)],
                         [p.BOARD_R] * 4).extrude(p.BOARD_T))
    for hx, hy in HOLES.values():
        slab = slab.cut(cyl_z(hx, hy, p.BOARD_Z_BOT - 1, p.board_z_top + 1, 2.7))
    parts = [solid(slab)]
    for ref, q in FPS.items():
        if ref.startswith(("H", "SW_L", "SW_R", "TP")) or q["crtyd"] is None:
            continue
        h, _ = comp_height(q["fp"])
        cx0, cy0, cx1, cy1 = q["fab"] or q["crtyd"]
        a = page2s(cx0, cy0); b = page2s(cx1, cy1)
        parts.append(solid(box(min(a[0], b[0]), max(a[0], b[0]), min(a[1], b[1]), max(a[1], b[1]),
                               p.board_z_top, p.board_z_top + h)))
    return cq.Compound.makeCompound(parts)


def switches_model():
    out = []
    for k, (sx, sy) in SW.items():
        b = box(sx - 3, sx + 3, sy - 3, sy + 3, p.board_z_top, p.board_z_top + p.SW_BODY_H)
        a = cyl_z(sx, sy, p.board_z_top + p.SW_BODY_H - 0.01, SW_TOP_Z, p.SW_ACT_D)
        out.append(solid(b).fuse(solid(a)))
    return cq.Compound.makeCompound(out)


def battery_model():
    x0, x1, y0, y1, z0, z1 = BAT_BOX
    return solid(box(x0, x1, y0, y1, z0, z1).edges("|Z").fillet(1.0))


def hardware_model():
    """Screw heads only (shanks thread into the bosses by design): 4 countersunk cover screws, 2 pan-head board screws."""
    out = []
    for sx, sy in COVER_SCREWS:
        # ISO 7046 M2.5 flat head (dk max 4.7, 90 deg), seated flush with the bottom
        out.append(cq.Solid.makeCone(2.35, 1.25, 1.1, cq.Vector(sx, sy, 0.0)))
    for h in REAR_HOLES:
        hx, hy = HOLES[h]
        out.append(solid(cyl_z(hx, hy, p.BOARD_Z_BOT - p.SCREW_HEAD_K, p.BOARD_Z_BOT, p.SCREW_HEAD_D)))
    return cq.Compound.makeCompound(out)


# ----------------------------------------------------------------------------------------------
# Build, checks, export
# ----------------------------------------------------------------------------------------------
def build(round_edges=True):
    ob, hol = hollow()
    parts = {
        "base": base_part(hol),
        "top": top_part(hol),
        "wheel": wheel_part(),
        "key_L": plunger("SW_L1"),
        "key_R": plunger("SW_R1"),
    }
    sharp = {}
    if round_edges:
        for k in list(parts):
            parts[k], left = round_all(parts[k], p.R_MIN, rounds=1, name=k)
            sharp[k] = left
    refs = {
        "board": board_model(),
        "switches": switches_model(),
        "battery": battery_model(),
        "screw_heads": hardware_model(),
    }
    return ob, parts, refs, sharp


# design contacts: pairs that touch on purpose (distance 0 allowed, overlap must still be 0)
CONTACT_OK = {
    frozenset(("board", "top")): "4 bosses clamp the board top at the M2.5 holes",
    frozenset(("board", "base")): "2 front columns support the board bottom at H1/H2",
    frozenset(("battery", "base")): "cell rests on 4 rib standoffs",
    frozenset(("key_L", "top")): "plunger seated in the tongue socket",
    frozenset(("key_R", "top")): "plunger seated in the tongue socket",
    frozenset(("screw_heads", "base")): "cover screw heads seat in the counterbores",
    frozenset(("screw_heads", "board")): "rear board screw heads seat on the board bottom",
    frozenset(("board", "switches")): "switches are soldered to the board",
    frozenset(("base", "top")): "base and top meet at the z = 14 split line",
}


def wall_thickness(part, outer, n=6000, seed=1):
    """Ray-cast local wall thickness from points on the OUTER skin, inward along -normal."""
    import numpy as np
    import trimesh
    def mesh(s):
        vs, fs = s.tessellate(0.05, 0.2)
        return trimesh.Trimesh([(v.x, v.y, v.z) for v in vs], fs, process=True)
    mp, mo = mesh(part), mesh(outer)
    pts, fi = trimesh.sample.sample_surface_even(mp, n, seed=seed)
    # keep samples that lie on the outer envelope
    _, dist, _ = trimesh.proximity.closest_point(mo, pts)
    keep = dist < 0.02
    pts, nrm = pts[keep], mp.face_normals[fi[keep]]
    origins = pts - nrm * 1e-3
    loc, idx_ray, _ = mp.ray.intersects_location(origins, -nrm, multiple_hits=True)
    import collections
    hits = collections.defaultdict(list)
    for l, r in zip(loc, idx_ray):
        hits[r].append(np.linalg.norm(l - origins[r]))
    th = []
    for r in range(len(origins)):
        h = [d for d in hits.get(r, []) if d > 1e-3]
        if h:
            th.append((min(h), tuple(np.round(origins[r], 2))))
    th.sort()
    return th


def in_hinge(pt, tol=0.3):
    for k in SW:
        x0, x1, y0, y1 = tongue_rect(k)
        if x0 - tol <= pt[0] <= x1 + tol and y0 - p.HINGE_LEN / 2 - tol <= pt[1] <= y0 + p.HINGE_LEN / 2 + tol:
            return True
    return False


def near_cut_edge(pt, d=0.75):
    """Sample points on the rim of a cut (slot, relief hole, wheel opening, USB opening) or on the
    parting-line groove measure the edge, not the wall: skip them."""
    x, y, z = pt
    if abs(z - p.H_BASE) < 1.0:     # parting line: both parts carry 0.5 rounds there (small V-groove)
        return True
    for k in SW:
        x0, x1, y0, y1 = tongue_rect(k)
        s_ = p.SLOT
        ye = y0 - HINGE_GROOVE_R
        if x0 - s_ - d <= x <= x1 + s_ + d and ye - p.RELIEF_D / 2 - d <= y <= y1 + s_ + d and z > CEIL_Z - 0.5:
            # inside the tongue (not near its slots) is real skin
            if not (x0 + d <= x <= x1 - d and y0 + d <= y <= y1 - d):
                return True
    ox0, ox1, oy0, oy1 = wheel_open()
    if ox0 - d <= x <= ox1 + d and oy0 - d <= y <= oy1 + d and z > CEIL_Z - 0.5:
        return True
    if x < -p.W / 2 + p.WALL + 0.5 and abs(y - J5_Y) < (p.PLUG_W / 2 + p.PLUG_OPEN_CLR + d) and \
            abs(z - J5_Z) < (p.PLUG_H / 2 + p.PLUG_OPEN_CLR + d):
        return True
    return False


def wall_report(parts, ob):
    """Thickness of the closed shell (base + top fused, so the split line is not an edge)."""
    sh = parts["base"].fuse(parts["top"])
    th = wall_thickness(sh, ob, n=12000)
    hinge = [t for t in th if in_hinge(t[1])]
    other = [t for t in th if not in_hinge(t[1]) and not near_cut_edge(t[1])]
    thin = [t for t in other if t[0] < p.WALL - 0.02]
    f3 = lambda v: tuple(round(float(c), 2) for c in v)
    return dict(samples=len(th), samples_used=len(other), min_excluding_hinges=round(float(other[0][0]), 3),
                at=f3(other[0][1]), hinge_min=round(float(min(t[0] for t in hinge)), 3) if hinge else None,
                below_1p48=[(round(float(t[0]), 3), f3(t[1])) for t in thin[:20]], n_below_1p48=len(thin))


def run_checks(ob, parts, refs, sharp):
    R = {}
    allp = {**parts, **refs}
    names = list(allp)
    # 1 + 6: overlap and clearance between every pair
    pairs = []
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = names[i], names[j]
            d = min_dist(allp[a], allp[b])
            v = common_volume(allp[a], allp[b])
            pairs.append(dict(a=a, b=b, min_clearance=round(d, 3), overlap_mm3=round(v, 4),
                              note=CONTACT_OK.get(frozenset((a, b)), "")))
    R["pairs"] = pairs
    R["overlaps"] = [q for q in pairs if q["overlap_mm3"] > 1e-3]
    R["unexpected_contacts"] = [q for q in pairs if q["min_clearance"] < 1e-6 and not q["note"]]
    # battery clearance to everything except its rib standoffs
    bat = allp["battery"]
    ribs_removed = parts["base"].cut(solid(box(-40, 40, -60, 60, -1, BAT_BOX[4] + 1e-3)))
    R["battery_clearance_excl_ribs"] = {
        "base (walls, clips, stops, columns)": round(min_dist(bat, ribs_removed), 3),
        **{k: round(min_dist(bat, allp[k]), 3) for k in allp if k not in ("battery", "base")},
    }
    # 2: key centring
    R["key_centring"] = {}
    for k in SW:
        x0, x1, y0, y1 = tongue_rect(k)
        R["key_centring"][k] = dict(tongue_centre=(round((x0 + x1) / 2, 3), round((y0 + y1) / 2, 3)),
                                    switch_centre=(round(SW[k][0], 3), round(SW[k][1], 3)),
                                    offset=round(math.hypot((x0 + x1) / 2 - SW[k][0], (y0 + y1) / 2 - SW[k][1]), 3))
    # 3: height
    zmax = max(s.BoundingBox().zmax for s in allp.values())
    zmin = min(s.BoundingBox().zmin for s in (parts["base"],))
    R["height"] = dict(shell_bottom=round(zmin, 3), key_top=p.H_TOP,
                       wheel_top=round(parts["wheel"].BoundingBox().zmax, 3), overall_top=round(zmax, 3), limit=38.0)
    # 4: USB
    pe = solid(plug_envelope())
    R["usb"] = dict(j5_mouth_x=round(J5_MOUTH_X, 3), j5_centre_y=round(J5_Y, 3), j5_centre_z=round(J5_Z, 3),
                    opening_centre=(round(-p.W / 2, 3), round(J5_Y, 3), round(J5_Z, 3)),
                    opening_wall="left (-X)", opening_size=(p.PLUG_W + 2 * p.PLUG_OPEN_CLR, p.PLUG_H + 2 * p.PLUG_OPEN_CLR),
                    plug_clearance={k: round(min_dist(pe, allp[k]), 3) for k in allp},
                    recess_depth_outer_skin_to_j5=round(J5_MOUTH_X + p.W / 2, 3))
    # 5: walls and edges
    xL0 = tongue_rect("SW_L1")[0]
    R["left_key_rim"] = dict(slot_outer_edge_x=round(xL0 - p.SLOT, 3), outer_side_x=-p.W / 2,
                             wall_outside_slot=round(xL0 - p.SLOT + p.W / 2, 3), required=1.5)
    R["wall_thickness"] = wall_report(parts, ob)
    # exact B-rep minimum distance between the outer skin and the cavity skin (walls + floor + top)
    cav = cavity_body().cut(box(-60, 60, -80, 80, -10, p.FLOOR).val())
    R["wall_thickness"]["shell_skin_exact_min"] = round(min_dist(ob, cav), 4)
    R["sharp_edges_left"] = {k: [dict(at=tuple(round(c, 2) for c in e.positionAt(0.5).toTuple()),
                                      len=round(e.Length(), 2)) for e in v] for k, v in sharp.items()}
    R["reduced_radius_edges"] = [dict(feature=t, radius=r, at=m, len=L) for (t, r, m, L) in REDUCED]
    R["detent"] = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in detent_geometry().items()}
    R["layout"] = dict(switches={k: tuple(round(c, 3) for c in v) for k, v in SW.items()},
                       holes={k: tuple(round(c, 3) for c in v) for k, v in HOLES.items()},
                       wheel_centre=(round(WHEEL_X, 3), round(WHEEL_Y, 3), round(WHEEL_Z, 3)),
                       wheel_opening=tuple(round(c, 3) for c in wheel_open()),
                       battery_box=tuple(round(c, 3) for c in BAT_BOX),
                       board_z=(p.BOARD_Z_BOT, p.board_z_top), switch_top_z=round(SW_TOP_Z, 3),
                       cover_screws=[tuple(round(c, 3) for c in s) for s in COVER_SCREWS])
    R["board_facts"] = board_facts()
    R["closest_components"] = closest_components(parts)
    return R


def closest_components(parts, n=3):
    """Nearest board components (F.Fab box x height) to each printed part, excluding the board slab."""
    comps = []
    for ref, q in FPS.items():
        if ref.startswith(("H", "SW_L", "SW_R", "TP")) or q["crtyd"] is None:
            continue
        h, _ = comp_height(q["fp"])
        cx0, cy0, cx1, cy1 = q["fab"] or q["crtyd"]
        a = page2s(cx0, cy0); b = page2s(cx1, cy1)
        comps.append((ref, solid(box(min(a[0], b[0]), max(a[0], b[0]), min(a[1], b[1]), max(a[1], b[1]),
                                     p.board_z_top, p.board_z_top + h))))
    out = {}
    for name in ("top", "base", "wheel", "key_L", "key_R"):
        d = sorted((min_dist(parts[name], c), ref) for ref, c in comps)[:n]
        out[name] = [(ref, round(x, 3)) for x, ref in d]
    return out


def close_mesh_cracks(path):
    """OCC meshes adjacent fillet faces independently; where their edge discretisation differs the STL
    has hairline cracks (T-junctions). manifold3d's MeshGL.merge() welds them; volume is unchanged."""
    import numpy as np
    import trimesh
    import manifold3d as mf
    m = trimesh.load(path)
    mg = mf.Mesh(vert_properties=np.asarray(m.vertices, dtype=np.float32),
                 tri_verts=np.asarray(m.faces, dtype=np.uint32))
    mg.merge()
    M = mf.Manifold(mg)
    if M.status() != mf.Error.NoError:
        raise RuntimeError(f"{path}: mesh repair failed: {M.status()}")
    o = M.to_mesh()
    t = trimesh.Trimesh(o.vert_properties[:, :3], o.tri_verts, process=True)
    assert abs(t.volume - m.volume) < 1e-3 * abs(m.volume) + 1.0, (path, t.volume, m.volume)
    t.export(path)


def export(parts, refs, outdir=HERE):
    # keys.* holds both plungers (2 bodies); key_L.* / key_R.* are single-body files for print services
    # that reject multi-body uploads (JLC3DP flags them as an "assembly").
    out = {"base": parts["base"], "top": parts["top"], "wheel": parts["wheel"],
           "keys": cq.Compound.makeCompound([parts["key_L"], parts["key_R"]]),
           "key_L": parts["key_L"], "key_R": parts["key_R"]}
    for k, s in out.items():
        cq.exporters.export(s, os.path.join(outdir, f"{k}.step"))
        stl = os.path.join(outdir, f"{k}.stl")
        cq.exporters.export(s, stl, tolerance=0.01, angularTolerance=0.1)
        close_mesh_cracks(stl)
    asm = cq.Assembly(name="mouse_shell_demo")
    colors = {"base": (0.25, 0.25, 0.28), "top": (0.85, 0.85, 0.88), "wheel": (0.1, 0.1, 0.1),
              "key_L": (0.9, 0.4, 0.1), "key_R": (0.9, 0.4, 0.1), "board": (0.1, 0.45, 0.2),
              "switches": (0.2, 0.2, 0.2), "battery": (0.75, 0.7, 0.3), "screw_heads": (0.6, 0.6, 0.6)}
    for k, s in {**parts, **refs}.items():
        asm.add(s, name=k, color=cq.Color(*colors[k]))
    asm.save(os.path.join(outdir, "assembly.step"))


def stl_report():
    import trimesh
    out = {}
    for k in ("base", "top", "keys", "key_L", "key_R", "wheel"):
        m = trimesh.load(os.path.join(HERE, f"{k}.stl"))
        parts = m.split(only_watertight=False)
        out[k] = dict(watertight=bool(m.is_watertight), bodies=len(parts), triangles=int(len(m.faces)),
                      volume_cm3=round(float(m.volume) / 1000, 3), size_mm=[round(float(v), 2) for v in m.extents])
    return out


def main():
    import time
    t0 = time.time()
    board_facts()
    ob, parts, refs, sharp = build(round_edges="--fast" not in sys.argv)
    print(f"built in {time.time() - t0:.1f} s")
    for k, s in parts.items():
        assert s.isValid(), k
    R = run_checks(ob, parts, refs, sharp)
    with open(os.path.join(HERE, "checks.json"), "w") as f:
        json.dump(R, f, indent=1, default=str)
    export(parts, refs)
    R["stl"] = stl_report()
    with open(os.path.join(HERE, "checks.json"), "w") as f:
        json.dump(R, f, indent=1, default=str)
    print(json.dumps(R["stl"], indent=1))
    print(json.dumps({k: R[k] for k in ("overlaps", "unexpected_contacts", "key_centring", "height",
                                         "left_key_rim", "wall_thickness")}, indent=1, default=str))
    print("sharp edges left:", {k: len(v) for k, v in R["sharp_edges_left"].items()})
    print(f"done in {time.time() - t0:.1f} s")


if __name__ == "__main__":
    main()
