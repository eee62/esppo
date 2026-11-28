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

The board geometry is read from ../ESP32-S31-Vision-Pro-RevC.kicad_pcb (read only, never written).
"""
from __future__ import annotations

import functools
import json
import math
import os
import re
import sys
from dataclasses import dataclass, field, asdict

import cadquery as cq
import numpy as np
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.TopExp import TopExp
from OCP.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE

HERE = os.path.dirname(os.path.abspath(__file__))
PCB_PATH = os.path.join(HERE, "..", "ESP32-S31-Vision-Pro-RevC.kicad_pcb")


# ----------------------------------------------------------------------------------------------
# Parameters
# ----------------------------------------------------------------------------------------------
@dataclass
class P:
    # outer shell: teardrop plan, vertical band up to the split, domed top (see "Outer shape")
    SIDE: float = 34.8             # half-width over the board / USB zone: board right edge 32.5 + 1.5 wall + 0.5 clear,
                                    # plus the side curving in between the widest line (z 8) and the board top (z 15.1)
    W_REAR: float = 70.4            # widest point, at y = REAR_C (battery clip hooks: x 32.3 + 1.0 flex + 1.5 wall)
    NOSE_Y: float = 60.0            # front tip
    NOSE_C: float = 52.5            # nose ellipse centre (SIDE x 20); the side is straight behind it
    WIDEN_Y: float = 10.0           # the side starts widening (cosine ramp) behind this y
    REAR_C: float = -40.0           # rear ellipse centre (W_REAR / 2 x 35)
    REAR_Y: float = -58.0           # rear tip
    REAR_N: float = 3.0             # rear superellipse exponent (2 = ellipse); 3 is the roundest that clears the cell's
                                    # the cell's rear corners and of the rear screw columns / clips
    H_BASE: float = 8.0            # widest level (the outline above) = dome start
    REAR_IFACE_Z: float = 14.0      # rear cover screws: base column top = top boss bottom (internal, unchanged)
    Z_SPLIT: float = 4.0            # parting line between base.stl (the dark lower band) and top.stl
    Z_PEAK: float = 37.4            # highest point of the skin
    PEAK_Y: float = -4.9            # 55 % of the length back from the nose
    DOME_N: float = 5.0             # side profile L(s_x) = (1 - s_x^N)^(1/N), s_x = |x| / half-width (radial mode: dome exponent; larger = fuller)
    DOME_MODE: str = "ref"          # "ref": the reference mouse (reference/Mini Computer Mouse_2.stl), cleaned, scaled
                                    #   and placed by ref_skin.py; every outline / dome parameter above is unused
                                    # "separable": z = H_BASE + (Z_PEAK - H_BASE) f(s_y) L(s_x), convex across by design
                                    # "radial": the earlier dome (outline scaled about the peak, superellipse DOME_N)
    DOME_N_MID: float = 2.8         # separable mode: side exponent at y = 0 and behind it at the rear tip (DOME_N_REAR);
    DOME_N_REAR: float = 2.8        #   DOME_N applies from y = DOME_N_Y1 forward (front posts, board corner), cosine blend
    DOME_N_Y1: float = 40.0
    PROF_A_FRONT: float = 1.6       # centreline profile ahead of the peak, f(s) = (1 - s^A)^(1/B), s = distance / length
    PROF_B_FRONT: float = 2.7874      #   (A sets how early it falls; B > 1 gives a vertical tangent at the nose)
    PROF_A_REAR: float = 2.0        # ... behind the peak (A = B = 2: an elliptic, round rear)
    PROF_B_REAR: float = 2.3
    DOME_N_END: float = 2.5         # radial mode only: exponent toward nose / rear
    DOME_BLEND: float = 1.0         # radial mode only
    NOSE_N: float = 3.5             # plan superellipse exponent of the nose (2 = ellipse)
    TUCK_FRONT: float = 0.8         # side tuck-in at z = 0 below the widest level, front (y >= TUCK_Y0) ...
    TUCK_REAR: float = 1.6          # ... and rear (y <= -TUCK_Y0); quadratic in z, zero slope at z = H_BASE
    TUCK_Y0: float = 20.0
    WALL: float = 1.5
    FLOOR: float = 1.5
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
    BOSS_D: float = 5.2             # top bosses that clamp the board top: 1.55 wall around the 2.1 pilot (hole footprint 5.9)
    BOSS_D_REAR: float = 6.0        # top bosses for the two rear cover screws
    PILOT_DEPTH: float = 8.0        # H3 / H4 (rear board holes, M2.5 x 6 pan head from below)
    PILOT_DEPTH_FRONT: float = 5.5  # H1 / H2: M2.5 x 20 flat head from the base reaches z 20.0; pilot to z 20.6
    SCREW_L_FRONT: float = 20.0     # cover screws at H1 / H2 (flat head, from below, through the board)
    SCREW_L_REAR: float = 20.0      # rear cover screws (flat head, from below)
    SCREW_L_BOARD: float = 6.0      # H3 / H4 board screws (pan head, from below the board)
    REAR_SCREW_X: float = 17.0
    REAR_SCREW_Y: float = -51.0
    PILOT_DEPTH_REAR: float = 6.5   # rear cover: M2.5 x 20 tip at z 20.0, pilot to z 20.5

    # keys: two buttons cut from the skin, symmetric about x = 0, hinged at HINGE_Y, free to the front
    BTN_X: float = 27.0             # outer edge of each button (|x|)
    SPINE_HW: float = 6.6           # fixed strip around the wheel between the buttons (half-width)
    WHEEL_SURROUND: float = 1.6     # spine skin in front of the wheel opening
    HINGE_Y: float = 10.0           # living hinge line (rear edge of both buttons)
    LIP: float = 4.0                # fixed skin between the buttons' front slot and the outline
    TONGUE_CLR: float = 2.0         # fixed features stay this far below the button skin (front travel)
    SLOT: float = 0.6              # JLC3DP minimum clearance between moving parts (MJF/SLS nylon)
    HINGE_T: float = 1.0            # living hinge thickness
    HINGE_LEN: float = 2.0          # hinge band length along Y
    RELIEF_D: float = 1.6           # stress-relief hole at each slot end (radius 0.8 > R_MIN, so its rim can be rounded)
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
    WHEEL_Z: float = 27.866858      # wheel axle height, locked (it was derived from the skin before the restyle)
    WHEEL_OPEN_CLR: float = 0.6     # opening clearance at each wheel side
    WHEEL_OPEN_END: float = 0.6     # opening clearance at front / rear (where the tire crosses the inner skin)
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
    def W(self):
        return 2 * self.SIDE

    @property
    def board_z_top(self):
        return self.BOARD_Z_BOT + self.BOARD_T


p = P()
if os.environ.get("SHELL_PARAMS"):         # design studies: override parameters (JSON), e.g. SHELL_PARAMS='{"SIDE": 34.8}'
    for _k, _v in json.loads(os.environ["SHELL_PARAMS"]).items():
        assert hasattr(p, _k), _k
        setattr(p, _k, _v)


# ----------------------------------------------------------------------------------------------
# KiCad board data (read from the PCB file, never written)
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
    """Parsed board (outline, footprints). The parse takes ~2 s, so it is cached on disk, keyed on the PCB file's
    bytes and on the parser source (any edit to either re-parses)."""
    import hashlib, inspect, pickle
    with open(PCB_PATH, "rb") as f:
        raw = f.read()
    src = "".join(inspect.getsource(fn) for fn in (_tokenize, _parse, _find, _read_board))
    key = hashlib.sha1(raw + src.encode()).hexdigest()[:16]
    path = os.path.join(HERE, ".cache", f"read_board-{key}.pkl")
    if not os.environ.get("SHELL_NO_CACHE") and os.path.exists(path):
        with open(path, "rb") as f:
            return pickle.load(f)
    out = _read_board(raw.decode("utf-8"))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path + ".tmp", "wb") as f:
        pickle.dump(out, f)
    os.replace(path + ".tmp", path)
    return out


def _read_board(txt):
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
WHEEL_X = 0.0                          # on the centreline, between the two buttons
WHEEL_Y = (SW["SW_L1"][1] + SW["SW_R1"][1]) / 2
FRONT_HOLES = ("H1", "H2")             # board_y = 3.4 -> front
REAR_HOLES = ("H3", "H4")

REF = None
if p.DOME_MODE == "ref":
    # the reference mouse, cleaned / scaled / placed by ref_skin.py (tables cached in .cache, rebuilt when the STL or
    # ref_skin.py change); REF_HASH is a module constant, so it is part of every stage's cache key
    import ref_skin
    REF = ref_skin.tables()
    REF_HASH = ref_skin.src_hash()
    REF_SCALE = REF["fit"]["s"]
    _g0 = REF["offsets"][0.0]["grid"].reshape(-1, 3)
    _g0 = _g0[_g0[:, 2] >= 0]
    p.REAR_Y = float(_g0[:, 1].min())
    p.NOSE_Y = float(_g0[:, 1].max())
    p.PEAK_Y = float(REF["peak"][1])
    p.Z_PEAK = float(REF["peak"][2])
    # half-width at the USB-C opening: the outer skin's x on the left side at the J5 centre (y, z), from the top map
    # neighbourhood is not enough (the side is near vertical): bisection on the reference outer field is not
    # available here, so use the outer grid: nearest grid points around (J5_Y, J5_Z) on the left
    _G = REF["offsets"][0.0]["grid"].reshape(-1, 3)
    _L = _G[(_G[:, 0] < 0) & (np.abs(_G[:, 1] - J5_Y) < 3) & (np.abs(_G[:, 2] - J5_Z) < 3)]
    _wt = 1.0 / (np.hypot(_L[:, 1] - J5_Y, _L[:, 2] - J5_Z) + 0.05) ** 2
    p.SIDE = float(-np.sum(_L[:, 0] * _wt) / np.sum(_wt))
    # the plungers (key parts) stay exactly as designed: their socket bottoms are locked; the sockets grow up to the
    # (higher) button skin instead
    KEY_SOCKET_Z0 = {"SW_L1": 28.524580899159268, "SW_R1": 29.50243722582077}


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
    """Angle between the two faces' normals at the edge midpoint, evaluated through each face's own pcurve
    (projecting the point onto the large skin splines instead was ~100x slower)."""
    from OCP.BRepAdaptor import BRepAdaptor_Curve2d
    from OCP.BRepGProp import BRepGProp_Face
    from OCP.gp import gp_Pnt, gp_Vec
    ns = []
    try:
        for f in (f1, f2):
            c2 = BRepAdaptor_Curve2d(edge.wrapped, f.wrapped)
            uv = c2.Value(0.5 * (c2.FirstParameter() + c2.LastParameter()))
            P, N = gp_Pnt(), gp_Vec()
            BRepGProp_Face(f.wrapped).Normal(uv.X(), uv.Y(), P, N)
            if N.Magnitude() < 1e-12:
                return 0.0
            ns.append(N)
    except Exception:
        return 0.0
    return math.degrees(ns[0].Angle(ns[1]))


def edge_faces(shape):
    m = TopTools_IndexedDataMapOfShapeListOfShape()
    TopExp.MapShapesAndAncestors_s(shape.wrapped, TopAbs_EDGE, TopAbs_FACE, m)
    out = []
    for i in range(1, m.Extent() + 1):
        e = cq.Edge(m.FindKey(i))
        fl = [cq.Face(f) for f in m.FindFromIndex(i)]
        out.append((e, fl))
    return out


def sharp_edges(shape, tol_deg=5.0, zmin=-1e9, near=None):
    """Edges between two faces that meet at an angle (not tangent). near = (bbox, pad): only edges whose
    midpoint lies in the padded box are tested (normals on the big skin splines are slow to evaluate)."""
    res = []
    for e, fl in edge_faces(shape):
        if len(fl) < 2:
            continue
        if near is not None and not _near(e, *near):
            continue
        if e.BoundingBox().zmin < zmin:
            continue
        if fl[0].isSame(fl[1]):      # seam edge
            continue
        if face_normal_angle(e, fl[0], fl[1]) > tol_deg:
            res.append(e)
    return res


def _safe_fillet(shape, edges, r):
    """Fillet, or None if it fails. OCC can segfault inside fillet(), so it runs in a forked child, which hands
    the result back as a binary BRep (exact) instead of the parent filleting a second time."""
    if not hasattr(os, "fork"):
        try:
            out = solid(shape.fillet(r, edges))
            return out if out.isValid() else None
        except Exception:
            return None
    import tempfile
    fd, path = tempfile.mkstemp(suffix=".bin")
    os.close(fd)
    try:
        pid = os.fork()
        if pid == 0:
            code = 1
            try:
                out = shape.fillet(r, edges)
                if out.isValid():
                    out.exportBin(path)
                    code = 0
            except BaseException:
                pass
            os._exit(code)
        _, status = os.waitpid(pid, 0)
        if not (os.WIFEXITED(status) and os.WEXITSTATUS(status) == 0):
            return None
        out = solid(cq.Shape.importBin(path))
        return out if out.isValid() else None
    finally:
        os.remove(path)


def fillet_edges(shape, edges, r, log=None, name=""):
    """Fillet edges; on failure bisect the list. Returns (shape, failed_edges)."""
    if not edges:
        return shape, []
    out = _safe_fillet(shape, edges, r)
    if out is not None:
        return out, []
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


def round_all(shape, r, tol_deg=5.0, rounds=3, name="", keep=None):
    """Try to give every sharp edge (only those passing `keep`, if given) at least radius r."""
    shape = solid(shape)
    failed = []
    for _ in range(rounds):
        es = [e for e in sharp_edges(shape, tol_deg) if keep is None or keep(e)]
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
    d = BRepExtrema_DistShapeShape()
    d.SetMultiThread(True)            # the top / board query alone otherwise runs for 40+ min on one core
    d.LoadS1(fa.wrapped)
    d.LoadS2(fb.wrapped)
    d.Perform()
    return d.Value()


def common_volume(a, b):
    try:
        c = a.intersect(b)
        return c.Volume()
    except Exception:
        return float("nan")


# ----------------------------------------------------------------------------------------------
# Disk cache for the slow stages (booleans on the lofted skin take 35-300 s each)
# ----------------------------------------------------------------------------------------------
CACHE_DIR = os.path.join(HERE, ".cache")
GEOM_DEPS = ("_plan_half", "plan_pt", "skin_pt", "_skin_normal", "_section_wire", "_fix", "skin_body", "solid", "box",
             "_ref_skin_body", "_ref_loop")
ROUND_DEPS = ("solid", "box", "cyl_z", "cyl_x", "rounded_poly", "sharp_edges", "face_normal_angle", "edge_faces", "_skin_mesh",
              "fillet_edges", "rematch", "_safe_fillet", "pre_round", "round_near", "add", "sub", "_near")


def disk_cache(*deps):
    """Cache a no-shape-argument stage on disk. The key holds the parameters, every upper-case scalar / tuple
    module constant, the source of the stage and of the named dependency functions, and the arguments; the
    stage's rounding log (REDUCED, FILLET_FAILS) is stored with it. SHELL_NO_CACHE=1 disables it."""
    def deco(fn):
        @functools.wraps(fn)
        def w(*args):
            if os.environ.get("SHELL_NO_CACHE"):
                return fn(*args)
            import hashlib, inspect, pickle
            g = globals()
            consts = {k: v for k, v in sorted(g.items()) if k.isupper() and isinstance(v, (int, float, str, tuple))}
            src = "".join(inspect.getsource(f) for f in (fn,) + tuple(g[d] for d in deps))
            key = hashlib.sha1(repr((asdict(p), consts, src, args)).encode()).hexdigest()[:16]
            path = os.path.join(CACHE_DIR, f"{fn.__name__}-{key}.pkl")
            if os.path.exists(path):
                with open(path, "rb") as f:
                    out, red, fails = pickle.load(f)
                REDUCED.extend(red)
                for k, v in fails.items():
                    FILLET_FAILS.setdefault(k, []).extend(v)
                return out
            n0, f0 = len(REDUCED), {k: len(v) for k, v in FILLET_FAILS.items()}
            out = fn(*args)
            red = REDUCED[n0:]
            fails = {k: v[f0.get(k, 0):] for k, v in FILLET_FAILS.items() if len(v) > f0.get(k, 0)}
            os.makedirs(CACHE_DIR, exist_ok=True)
            with open(path + ".tmp", "wb") as f:
                pickle.dump((out, red, fails), f)
            os.replace(path + ".tmp", path)
            return out
        return w
    return deco


# ----------------------------------------------------------------------------------------------
# Outer body
# ----------------------------------------------------------------------------------------------
def _plan_half():
    """Right half of the outline (x >= 0), nose -> rear. Nose: superellipse (exponent NOSE_N) SIDE x (NOSE_Y - NOSE_C); side: SIDE over
    the board / USB zone, widening behind WIDEN_Y with a cosine ramp to W_REAR / 2 at REAR_C; rear: superellipse
    (exponent REAR_N), horizontal at the tip and vertical where it meets the side."""
    s, w = p.SIDE, p.W_REAR / 2
    pts = []
    en = 2.0 / p.NOSE_N
    for a in range(90, 0, -10):
        t = math.radians(a)
        pts.append((s * math.cos(t) ** en, p.NOSE_C + (p.NOSE_Y - p.NOSE_C) * math.sin(t) ** en))
    for y in range(int(p.NOSE_C), int(p.REAR_C), -4):
        g = 0.0 if y >= p.WIDEN_Y else 0.5 * (1 - math.cos(math.pi * (p.WIDEN_Y - y) / (p.WIDEN_Y - p.REAR_C)))
        pts.append((s + (w - s) * g, float(y)))
    e = 2.0 / p.REAR_N
    for a in range(0, 91, 5):
        t = math.radians(a)
        pts.append((w * math.cos(t) ** e, p.REAR_C - (p.REAR_C - p.REAR_Y) * math.sin(t) ** e))
    pts[0], pts[-1] = (0.0, p.NOSE_Y), (0.0, p.REAR_Y)
    return pts


# one periodic spline (CCW, starting at the REAR tip): symmetric, so the tip tangents are horizontal; a single
# closed edge per section is what lets OCC fillet the floor / bottom edges; the seam sits on the rear centreline,
# where nothing is cut (a seam through the slots splits the rim edges and their fillets fail)
def _ref_loop(n=300):
    """Top-view outline of the reference skin: CCW, starting at the rear tip, smoothed (the table is on a 0.25 grid)."""
    c = np.asarray(REF["outline"], float)
    if np.allclose(c[0], c[-1]):
        c = c[:-1]
    if 0.5 * np.sum(c[:, 0] * np.roll(c[:, 1], -1) - np.roll(c[:, 0], -1) * c[:, 1]) < 0:
        c = c[::-1]
    sl = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(np.vstack([c, c[:1]]), axis=0).T))])
    t = np.linspace(0, sl[-1], 1201)[:-1]
    q = np.column_stack([np.interp(t, sl, np.append(c[:, 0], c[0, 0])), np.interp(t, sl, np.append(c[:, 1], c[0, 1]))])
    from scipy.ndimage import gaussian_filter1d
    q = gaussian_filter1d(q, 3.0, axis=0, mode="wrap")
    q = np.roll(q, -int(np.argmin(q[:, 1])), axis=0)[::len(q) // n]
    return [(float(x), float(y)) for x, y in q]


if p.DOME_MODE == "ref":
    _LOOP = _ref_loop()
else:
    _LOOP = [(x, y) for x, y in reversed(_plan_half())] + [(-x, y) for x, y in _plan_half()][1:-1]
PLAN_EDGE = cq.Edge.makeSpline([cq.Vector(x, y, 0) for x, y in _LOOP], periodic=True)
SKIN_NU, SKIN_NPHI = 72, 20         # points per half section, number of dome sections (144 x 30 made every
                                    # later boolean ~80x slower for < 0.002 mm of wall accuracy)


def plan_pt(u):
    """Right half of the outline, u = 0 at the nose, u = 1 at the rear."""
    q = PLAN_EDGE.positionAt(0.5 - 0.5 * min(max(u, 0.0), 1.0))
    return np.array([q.x, q.y])


SKIN_LOW_Z = (-2.0, 0.0, 3.0, 5.5)         # section heights below the widest level (z = -2 lies below the bottom cut;
                                            # offset inward they must stay clear of the floor cut at z = FLOOR)
_low_span = 10.0                            # v = -1 at z = H_BASE - 10 = -2


def _plan_normal(u):
    """Outward plan normal of the outline at u (right half, traversed nose -> rear)."""
    du = 1e-4
    t = plan_pt(min(u + du, 1.0)) - plan_pt(max(u - du, 0.0))
    n = np.array([-t[1], t[0]])
    return n / np.linalg.norm(n)


def _dome_n(dx, dy):
    """Dome exponent for the direction (dx, dy) from the peak: DOME_N at the sides, DOME_N_END toward nose / rear."""
    w = np.asarray(dy, float) ** 2 / np.maximum(np.asarray(dx, float) ** 2 + np.asarray(dy, float) ** 2, 1e-12)
    return p.DOME_N + (p.DOME_N_END - p.DOME_N) * w ** p.DOME_BLEND


def _tuck(y):
    """Tuck-in amplitude: TUCK_FRONT at the front, TUCK_REAR at the rear, cosine ramp between y = +-TUCK_Y0."""
    g = min(max((p.TUCK_Y0 - y) / (2 * p.TUCK_Y0), 0.0), 1.0)
    return p.TUCK_FRONT + (p.TUCK_REAR - p.TUCK_FRONT) * 0.5 * (1 - math.cos(math.pi * g))


def skin_pt(u, v):
    """Outer skin point. v >= 0 (= phi): dome, the outline scaled by s about (0, PEAK_Y) at height z, with (s, z) on
    a superellipse (phi = 0: s = 1, z = H_BASE, vertical tangent; phi = 90 deg: the peak, horizontal tangent).
    v < 0: lower side, z = H_BASE + 10 v, the outline moved in along its normal by tuck(y) ((H_BASE - z) / H_BASE)^2:
    zero slope at H_BASE, so the side runs smoothly out of the dome and curves in toward the bottom."""
    if v < 0:
        z = p.H_BASE + _low_span * v
        q = plan_pt(u)
        xy = q - _tuck(q[1]) * ((p.H_BASE - z) / p.H_BASE) ** 2 * _plan_normal(u)
        return np.array([xy[0], xy[1], z])
    c = np.array([0.0, p.PEAK_Y])
    q = plan_pt(u)
    if p.DOME_MODE == "radial":
        n = _dome_n(q[0], q[1] - p.PEAK_Y)
        s = abs(math.cos(v)) ** (2 / n)
        h = abs(math.sin(v)) ** (2 / n)
        xy = c + s * (q - c)
        return np.array([xy[0], xy[1], p.H_BASE + (p.Z_PEAK - p.H_BASE) * h])
    # separable: the section at this v is the contour of the height field at level z, found along the ray from the
    # peak to the outline point (the field falls monotonically along it: 1 at the peak, 0 on the outline)
    z = p.H_BASE + (p.Z_PEAK - p.H_BASE) * abs(math.sin(v)) ** 0.8
    if v <= 0:
        return np.array([q[0], q[1], p.H_BASE])
    lo, hi = 0.0, 1.0
    for _ in range(48):
        t = 0.5 * (lo + hi)
        xy = c + t * (q - c)
        if float(_z_sep(xy[0], xy[1])) > z:
            lo = t
        else:
            hi = t
    xy = c + 0.5 * (lo + hi) * (q - c)
    return np.array([xy[0], xy[1], z])


def _skin_normal(u, v):
    du = dp = 1e-4
    Pu = skin_pt(min(u + du, 1), v) - skin_pt(max(u - du, 0), v)
    Pp = skin_pt(u, v + 2 * dp) - skin_pt(u, v) if 0 <= v < dp else skin_pt(u, v + dp) - skin_pt(u, v - dp)
    n = np.cross(Pu, Pp)
    n /= np.linalg.norm(n)
    P = skin_pt(u, v)
    if P[0] * n[0] + (P[1] - p.PEAK_Y) * n[1] < 0:
        n = -n
    return n


def _section_wire(v, d, planar=False):
    pts = []
    for i in range(SKIN_NU + 1):
        u = i / SKIN_NU
        P = skin_pt(u, v)
        if d:
            n = _skin_normal(u, v)
            if planar:                          # lowest section (below the bottom cut): horizontal offset, stays planar
                n[2] = 0.0
                n /= np.linalg.norm(n)
            P = P - d * n
        pts.append(P)
    pts[0][0] = pts[-1][0] = 0.0
    r = [cq.Vector(*q) for q in reversed(pts)]                      # rear -> nose, right side (CCW, seam at rear)
    l = [cq.Vector(-q[0], q[1], q[2]) for q in pts][1:-1]
    return cq.Wire.assembleEdges([cq.Edge.makeSpline(r + l, periodic=True)])


def _fix(shape):
    from OCP.ShapeFix import ShapeFix_Shape
    fx = ShapeFix_Shape(shape)
    fx.Perform()
    return solid(cq.Shape.cast(fx.Shape()))


@functools.lru_cache(maxsize=None)
@disk_cache(*GEOM_DEPS, "_plan_normal", "_tuck", "_dome_n", "_z_sep", "_side_n", "_half_width_table", "_ref_skin_body")
def skin_body(d=0.0, z_bottom=0.0):
    """Outer body (d = 0) or the skin offset inward by d along the surface normal (cavity: d = WALL), cut flat at
    z_bottom. One loft through the lower-side sections and the dome sections, closed at the peak."""
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
    from OCP.gp import gp_Pnt
    if p.DOME_MODE == "ref":
        return _ref_skin_body(d, z_bottom)
    vs = [(z - p.H_BASE) / _low_span for z in SKIN_LOW_Z] + [0.5 * math.pi * j / SKIN_NPHI for j in range(SKIN_NPHI)]
    # planar offsets for the lowest section (below the bottom cut) and the widest-line section (v = 0), where the
    # skin is vertical by construction (numerical normals there pick up the half-width table's small kinks)
    wires = [_section_wire(v, d, planar=(i == 0 or v == 0)) for i, v in enumerate(vs)]
    ts = BRepOffsetAPI_ThruSections(True, False, 1e-6)
    for w in wires:
        ts.AddWire(w.wrapped)
    ts.AddVertex(BRepBuilderAPI_MakeVertex(gp_Pnt(0, p.PEAK_Y, p.Z_PEAK - d)).Vertex())
    ts.CheckCompatibility(False)
    ts.Build()
    body = _fix(ts.Shape())
    return _fix(body.intersect(box(-60, 60, -80, 80, z_bottom, 60).val()).clean().wrapped)


def _ref_skin_body(d, z_bottom):
    """Reference skin offset d inward (one of ref_skin.OFFSETS): one periodic B-spline face interpolated through the
    radial point grid (pole to pole, chord-length parameters), closed into a solid. The outer skin (d = 0) runs on
    below z = 0 and is cut flat at z_bottom; the offset skins carry their own floor at ref_skin.FLOOR."""
    from OCP.TColgp import TColgp_Array2OfPnt
    from OCP.GeomAPI import GeomAPI_PointsToBSplineSurface
    from OCP.Approx import Approx_ParametrizationType
    from OCP.gp import gp_Pnt
    from OCP.BRep import BRep_Builder
    from OCP.TopoDS import TopoDS_Shell
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeSolid
    key = min(REF["offsets"], key=lambda k: abs(k - d))
    assert abs(key - d) <= 1e-6, f"no reference skin table for offset {d}"
    t = REF["offsets"][key]
    G = t["grid"]
    A = TColgp_Array2OfPnt(1, G.shape[0], 1, G.shape[1])
    for i in range(G.shape[0]):
        for j in range(G.shape[1]):
            A.SetValue(i + 1, j + 1, gp_Pnt(*map(float, G[i, j])))
    it = GeomAPI_PointsToBSplineSurface()
    it.Interpolate(A, Approx_ParametrizationType.Approx_ChordLength, True)
    face = BRepBuilderAPI_MakeFace(it.Surface(), 1e-6).Face()
    bld, sh = BRep_Builder(), TopoDS_Shell()
    bld.MakeShell(sh)
    bld.Add(sh, face)
    body = _fix(BRepBuilderAPI_MakeSolid(sh).Solid())
    if t.get("floor") is not None:
        assert 0 <= t["floor"] - z_bottom < 0.05, "the offset tables carry their floor at ref_skin.FLOOR (FLOOR + margin)"
        return body
    return _fix(body.intersect(box(-90, 90, -120, 120, z_bottom, 90).val()).clean().wrapped)


def outer_body():
    return skin_body(0.0, 0.0)


@functools.lru_cache(maxsize=None)
@disk_cache(*GEOM_DEPS)
def cavity_body():
    """Cavity above the floor, floor-wall corner rounded R_MIN."""
    c = skin_body(p.WALL + (0.15 if p.DOME_MODE == "ref" else 0.002), p.FLOOR)   # + margin: exact wall >= WALL (ref: local edge sharpness)
    if p.DOME_MODE == "ref":
        return c                        # floor-wall blend is part of the reference cavity table
    bot = [e for e in c.Edges() if abs(e.BoundingBox().zmax - p.FLOOR) < 1e-6 and abs(e.BoundingBox().zmin - p.FLOOR) < 1e-6]
    return solid(c.fillet(p.R_MIN, bot))


def _polar_table():
    th, rr = [], []
    for i in range(4000):
        q = PLAN_EDGE.positionAt(i / 4000)
        th.append(math.atan2(q.y - p.PEAK_Y, q.x)); rr.append(math.hypot(q.x, q.y - p.PEAK_Y))
    o = np.argsort(th)
    th, rr = np.array(th)[o], np.array(rr)[o]
    return np.concatenate([th - 2 * np.pi, th, th + 2 * np.pi]), np.concatenate([rr, rr, rr])


_TH, _RR = _polar_table()
_PLAN_POLY = np.array([[PLAN_EDGE.positionAt(i / 2000).x, PLAN_EDGE.positionAt(i / 2000).y] for i in range(2000)])


def _half_width_table():
    """Outline half-width at y (the plan is convex): y grid and max |x| on it."""
    ys = np.linspace(p.REAR_Y, p.NOSE_Y, 2001)
    P = _PLAN_POLY
    xn, yn = np.roll(P[:, 0], -1), np.roll(P[:, 1], -1)
    hw = []
    for y in ys:
        c = (P[:, 1] - y) * (yn - y) <= 0
        den = np.where(yn[c] == P[c, 1], 1e-12, yn[c] - P[c, 1])
        xs = P[c, 0] + (y - P[c, 1]) * (xn[c] - P[c, 0]) / den
        hw.append(np.abs(xs).max() if len(xs) else 0.0)
    return ys, np.maximum(np.array(hw), 1e-6)


def _z_sep(x, y):
    """Separable dome: H_BASE + (Z_PEAK - H_BASE) f(s_y) L(s_x). L is the side profile across the local half-width,
    f the centreline profile (separate front / rear). Every cross-section is highest on the centreline."""
    x = np.abs(np.asarray(x, float)); y = np.asarray(y, float)
    hw = np.interp(y, _HW_Y, _HW_X)
    sx = np.clip(x / hw, 0.0, 1.0)
    front = y >= p.PEAK_Y
    sy = np.where(front, (y - p.PEAK_Y) / (p.NOSE_Y - p.PEAK_Y), (p.PEAK_Y - y) / (p.PEAK_Y - p.REAR_Y))
    sy = np.clip(sy, 0.0, 1.0)
    a = np.where(front, p.PROF_A_FRONT, p.PROF_A_REAR); b = np.where(front, p.PROF_B_FRONT, p.PROF_B_REAR)
    f = np.clip(1 - sy ** a, 0.0, 1.0) ** (1 / b)
    n = _side_n(y)
    L = np.clip(1 - sx ** n, 0.0, 1.0) ** (1 / n)
    return p.H_BASE + (p.Z_PEAK - p.H_BASE) * f * L


def _side_n(y):
    """Side-profile exponent along the length: DOME_N from DOME_N_Y1 forward, DOME_N_MID at y = 0, DOME_N_REAR at the
    rear tip; cosine blends between (constant across each cross-section, so every section stays convex)."""
    y = np.asarray(y, float)
    g1 = np.clip(y / p.DOME_N_Y1, 0.0, 1.0)
    front = p.DOME_N_MID + (p.DOME_N - p.DOME_N_MID) * 0.5 * (1 - np.cos(np.pi * g1))
    g2 = np.clip(y / p.REAR_Y, 0.0, 1.0)
    rear = p.DOME_N_MID + (p.DOME_N_REAR - p.DOME_N_MID) * 0.5 * (1 - np.cos(np.pi * g2))
    return np.where(y >= 0, front, rear)


_HW_Y, _HW_X = _half_width_table()


def _z_ref(x, y):
    """Outer skin height from the reference top map (bilinear; far below everything outside the outline)."""
    from scipy.ndimage import map_coordinates
    x, y = np.broadcast_arrays(np.asarray(x, float), np.asarray(y, float))
    tm = REF["top_map"]
    st = tm["xs"][1] - tm["xs"][0]
    Z = np.where(np.isfinite(tm["z"]), tm["z"], -50.0)
    out = map_coordinates(Z, [(x.ravel() - tm["xs"][0]) / st, (y.ravel() - tm["ys"][0]) / st], order=1, mode="nearest")
    return out.reshape(x.shape) if x.shape else float(out[0])


def _z_dome(x, y):
    if p.DOME_MODE == "ref":
        return _z_ref(x, y)
    if p.DOME_MODE != "radial":
        return _z_sep(x, y)
    x = np.asarray(x, float); yy = np.asarray(y, float) - p.PEAK_Y
    rho = np.minimum(np.hypot(x, yy) / np.interp(np.arctan2(yy, x), _TH, _RR), 1.0)
    n = _dome_n(x, yy)
    return p.H_BASE + (p.Z_PEAK - p.H_BASE) * (1 - rho ** n) ** (1 / n)


def z_skin(x, y, d=0.0):
    """Analytic height of the outer skin at plan (x, y), or of the skin offset inward by d (normal offset,
    locally planar approximation). Used to place features; the checks measure the real B-rep."""
    z = _z_dome(x, y)
    if d:
        gx = (_z_dome(np.asarray(x) + 0.05, y) - _z_dome(np.asarray(x) - 0.05, y)) / 0.1
        gy = (_z_dome(x, np.asarray(y) + 0.05) - _z_dome(x, np.asarray(y) - 0.05)) / 0.1
        z = z - d * np.sqrt(1 + gx * gx + gy * gy)
    return z


def _in_poly(poly, x, y):
    xs, ys = poly[:, 0], poly[:, 1]
    xn, yn = np.roll(xs, -1), np.roll(ys, -1)
    cross = (ys > y) != (yn > y)
    return bool(np.count_nonzero(cross & (x < xs + (y - ys) * (xn - xs) / np.where(yn == ys, 1e-12, yn - ys))) % 2)


def plan_inset(x, y):
    """Plan distance from (x, y) to the outline (positive inside)."""
    dmin = float(np.min(np.hypot(_PLAN_POLY[:, 0] - x, _PLAN_POLY[:, 1] - y)))
    inside = _in_poly(_PLAN_POLY, x, y)
    return dmin if inside else -dmin


def plan_offset_prism(d, z0, z1):
    """Prism of the outline offset inward by d (the loop runs CCW, so (-ty, tx) points inward)."""
    pts = []
    for i in range(288):
        t = i / 288
        q = PLAN_EDGE.positionAt(t); tg = PLAN_EDGE.tangentAt(t)
        pts.append(cq.Vector(q.x - d * tg.y, q.y + d * tg.x, z0))
    w = cq.Wire.assembleEdges([cq.Edge.makeSpline(pts, periodic=True)])
    return cq.Solid.extrudeLinear(cq.Face.makeFromWires(w), cq.Vector(0, 0, z1 - z0))


WHEEL_Z = p.WHEEL_Z                    # locked
Z_HI = p.Z_PEAK + 5.0                  # top of feature blanks; every blank is intersected with the outer body
SLOT_Z0 = 9.0 if p.DOME_MODE == "ref" else 16.0   # slot / opening cutters start here (only skin exists there when cut;
                                       # the reference nose is low, so the buttons' front slot runs down to ~12)


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
    g = _safe_fillet(f, es, r)
    if g is not None:
        return g
    g, _ = fillet_edges(f, es, r)
    return g


# Edge rule (2026-11-25): only edges on the outer skin or at the USB-C opening need R_MIN; sharp edges inside
# the shell (slots, hinge grooves, under the buttons, hanging features) are accepted.
INTERNAL_TAGS = ("rib", "stop", "column", "clip", "socket", "boss", "bracket", "detent", "axle_seat",
                 "socket_hole", "pilot", "hinge")


@functools.lru_cache(maxsize=None)
def _skin_faces():
    return cq.Compound.makeCompound(outer_body().Faces())


_REQ_MEMO = {}


def required_edge(e):
    """Edge (or (x, y, z) point) on the outer skin or at the USB-C opening: these need at least R_MIN. Results are
    memoised by midpoint (the rounding loops ask about the same untouched edges many times)."""
    c = e.positionAt(0.5).toTuple() if hasattr(e, "positionAt") else tuple(e)
    k = tuple(round(v, 4) for v in c)
    if k not in _REQ_MEMO:
        _REQ_MEMO[k] = _required_point(c)
    return _REQ_MEMO[k]


def _required_point(c):
    ub = usb_opening().val().BoundingBox()
    if (ub.xmin - 0.6 <= c[0] <= ub.xmax + 0.6 and ub.ymin - 0.6 <= c[1] <= ub.ymax + 0.6
            and ub.zmin - 0.6 <= c[2] <= ub.zmax + 0.6):
        return True
    # fast reject on a 0.01 mm mesh of the skin: farther than 0.07 there means farther than 0.06 on the exact
    # surface (> 0.05); only the rest is measured exactly (the exact query on the big lofted face takes seconds)
    import trimesh
    if float(trimesh.proximity.closest_point(_skin_mesh(), [c])[1][0]) > (0.2 if p.DOME_MODE == "ref" else 0.07):
        return False
    d = BRepExtrema_DistShapeShape(cq.Vertex.makeVertex(*c).wrapped, _skin_faces().wrapped)
    d.Perform()
    return d.Value() < 0.05


@functools.lru_cache(maxsize=None)
def _skin_mesh():
    """Outer skin as a mesh with 0.01 mm absolute chord sag (for fast distance pre-checks)."""
    import trimesh
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRep import BRep_Tool
    from OCP.TopLoc import TopLoc_Location
    ob = cq.Shape.importBrep(_brep_copy(outer_body()))      # mesh a copy: keep the cached body free of triangulation
    _imesh(ob, 0.01)
    V, F, off = [], [], 0
    for f in ob.Faces():
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(f.wrapped, loc)
        if tri is None:
            continue
        tr = loc.Transformation()
        V += [tri.Node(i).Transformed(tr).Coord() for i in range(1, tri.NbNodes() + 1)]
        F += [tuple(off + k - 1 for k in tri.Triangle(i).Get()) for i in range(1, tri.NbTriangles() + 1)]
        off += tri.NbNodes()
    return trimesh.Trimesh(np.array(V), np.array(F), process=True)


def _imesh(shape, lin, ang=0.1):
    """Mesh a shape in place with an absolute chord sag. Surface-deflection control on: without it OCC under-meshes
    the reference skin (one dense periodic B-spline face) by an order of magnitude (2 mm facets at "0.01 mm")."""
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.IMeshTools import IMeshTools_Parameters
    pr = IMeshTools_Parameters()
    pr.Deflection, pr.Angle, pr.Relative, pr.InParallel = lin, ang, False, True
    pr.ControlSurfaceDeflection, pr.MinSize = True, max(2 * lin, 0.005)
    BRepMesh_IncrementalMesh(shape.wrapped, pr)


def _brep_copy(s):
    import io
    buf = io.BytesIO()
    s.exportBrep(buf)
    buf.seek(0)
    return buf


def round_near(body, bb, tag, r=None, pad=0.6, rounds=3):
    r = r or p.R_MIN
    prev = None
    failed = []
    if tag in INTERNAL_TAGS:
        # internal feature: one bulk try (bisected on failure); no per-edge fallback, leftovers are accepted
        es = sharp_edges(body, near=(bb, pad))
        body, failed = fillet_edges(body, es, r)
        if failed:
            FILLET_FAILS.setdefault(tag, []).extend(failed)
        return solid(body)
    # cut in the skin (slot, opening, hole): only edges on the outer skin or at the USB-C opening must be rounded
    # (edge rule); edges inside the cut stay sharp. Rounding them anyway with reduced-radius fallbacks left sliver
    # faces at the slot ends that OCC cannot mesh (non-manifold STL).
    def req():
        return [e for e in sharp_edges(body, near=(bb, pad)) if required_edge(e)]
    for _ in range(rounds):
        es = req()
        if not es or (prev is not None and len(es) >= prev):
            break
        prev = len(es)
        body, failed = fillet_edges(body, es, r)
    # leftovers: one edge at a time, R_MIN first, then smaller radii; the candidates are found once per radius and
    # re-matched to the current body by midpoint after each success (re-scanning after every edge was ~1.5 min each)
    for rr in (r,) + FALLBACK_R:
        cands = req()
        if not cands:
            break
        for e0 in cands:
            m = rematch(body, [e0])
            if not m:
                continue                      # consumed by an earlier fillet
            nb = _safe_fillet(body, m, rr)
            if nb is not None:
                body = nb
                if rr < r:
                    REDUCED.append((tag, rr, tuple(round(c, 2) for c in e0.positionAt(0.5).toTuple()), round(e0.Length(), 2)))
    failed = req()
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
@functools.lru_cache(maxsize=None)
@disk_cache(*GEOM_DEPS, "outer_body", "cavity_body")
def hollow():
    """Outer body minus the cavity above the floor."""
    ob = outer_body()
    return ob, solid(ob.cut(cavity_body()))


def usb_opening():
    """Pocket through the left wall so the USB-C plug overmold reaches J5 (stops PLUG_STOP short)."""
    w = p.PLUG_W + 2 * p.PLUG_OPEN_CLR
    h = p.PLUG_H + 2 * p.PLUG_OPEN_CLR
    x0, x1 = -p.W / 2 - 5, -p.W / 2 + p.WALL + 3.0   # through the wall (which leans in toward the opening top)
    if p.DOME_MODE == "ref":
        x1 = J5_MOUTH_X - 1.0                           # deeper recess (9.6 mm): the pocket runs to 1 mm before J5
    return (cq.Workplane("YZ").workplane(offset=x0).center(J5_Y, J5_Z)
            .slot2D(w, h, 0).extrude(x1 - x0))            # exact stadium: straight sides, full-radius ends


def plug_envelope():
    """USB-C plug overmold at full insertion (for the clearance check)."""
    x1 = J5_MOUTH_X - p.PLUG_STOP
    return (cq.Workplane("YZ").workplane(offset=-p.W / 2 - 20).center(J5_Y, J5_Z)
            .rect(p.PLUG_W, p.PLUG_H).extrude(x1 + p.W / 2 + 20).edges("|X").fillet(p.PLUG_H / 2 - 0.05))


def round_split(part, z):
    """Round the edges of the split face (z = Z_SPLIT) and anything else left sharp there."""
    es = [e for e in sharp_edges(part) if abs(e.positionAt(0.5).z - z) < 0.05]
    if p.DOME_MODE == "ref":            # inner split edges are internal (edge rule: may stay sharp); on the reference
        es = [e for e in es if required_edge(e)]      # cavity skin their fillets crawl and fail
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


@disk_cache(*GEOM_DEPS, *ROUND_DEPS, "hollow", "round_split", "usb_opening", "_hole_edge", "outer_body",
            "cavity_body", "required_edge", "_required_point", "_skin_mesh")
def base_part():
    hol = hollow()[1]
    body = solid(hol.intersect(box(-90, 90, -120, 120, -1, p.Z_SPLIT).val()))
    body = round_split(body, p.Z_SPLIT)
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
        ztop = p.BOARD_Z_BOT if i < 2 else p.REAR_IFACE_Z
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
        body = sub(body, cyl_z(sx, sy, -1, p.REAR_IFACE_Z + 1, p.SCREW_CLEAR_D), "hole")
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
def wheel_open():
    """Opening = where the tire top rises through the inner skin (checked at the centre and both opening
    sides), plus WHEEL_OPEN_END front and rear; WHEEL_OPEN_CLR at each tire side."""
    hx = p.WHEEL_W / 2 + p.WHEEL_OPEN_CLR
    ys = np.linspace(WHEEL_Y - p.WHEEL_D / 2, WHEEL_Y + p.WHEEL_D / 2, 2001)
    zt = WHEEL_Z + np.sqrt(np.maximum((p.WHEEL_D / 2) ** 2 - (ys - WHEEL_Y) ** 2, 0))
    hit = np.zeros_like(ys, bool)
    for x in (WHEEL_X - hx, WHEEL_X, WHEEL_X + hx):
        hit |= zt >= z_skin(np.full_like(ys, x), ys, p.WALL)
    yy = ys[hit]
    return (WHEEL_X - hx, WHEEL_X + hx, float(yy.min()) - p.WHEEL_OPEN_END, float(yy.max()) + p.WHEEL_OPEN_END)


WOPEN = wheel_open()
SPINE_FRONT_Y = WOPEN[3] + p.WHEEL_SURROUND      # front edge of the fixed strip around the wheel
HINGE_GROOVE_R = 1.25
SLOT_REAR_Y = p.HINGE_Y - HINGE_GROOVE_R - p.RELIEF_D / 2 - 0.3   # slots run past the hinge groove to here;
                                    # the relief holes there stay 0.3 clear of the groove (no hole / groove sliver edges)


TONGUE_R = 2.0      # plan radius of every free button corner: tangent slot rims, which OCC can round


def _tongue_wire(grow=0.0):
    """Right button outline, planar at z = SLOT_Z0, grown outward by `grow` (grow = SLOT gives the slot's outer
    edge: an exact offset, built directly so it keeps few edges). It runs on behind the hinge so the side slots
    stay straight there; the slots are clipped at SLOT_REAR_Y."""
    z0, z1 = SLOT_Z0, SLOT_Z0 + 1.0
    yb = SLOT_REAR_Y - 10
    sx, sy = p.SPINE_HW + p.SLOT - grow, SPINE_FRONT_Y + p.SLOT - grow
    t = box(p.SLOT / 2 - grow, p.BTN_X + grow, yb - grow, p.NOSE_Y + 5, z0, z1).val() \
        .cut(box(-5, sx, yb - 1, sy, z0, z1).val()) \
        .intersect(plan_offset_prism(p.LIP + p.SLOT - grow, z0, z1))
    t = solid(t)
    vert = [e for e in t.Edges() if e.BoundingBox().zmax - e.BoundingBox().zmin > 0.5 and e.BoundingBox().ymin > p.HINGE_Y]
    concave = [e for e in vert if abs(e.Center().x - sx) < 1e-3 and abs(e.Center().y - sy) < 1e-3]
    convex = [e for e in vert if not any(e.isSame(c) for c in concave)]
    t = solid(t.fillet(TONGUE_R + grow, convex))
    t = solid(t.fillet(TONGUE_R - grow, [e for e in t.Edges() if abs(e.Center().x - sx) < 1e-3
                                         and abs(e.Center().y - sy) < 1e-3 and e.geomType() == "LINE"]))
    f = [fc for fc in t.Faces() if fc.geomType() == "PLANE" and abs(fc.Center().z - z0) < 1e-6][0]
    return f.outerWire()


def _outer_slot_wire():
    """Outer edge of the slots around BOTH buttons as one symmetric outline (the buttons' outline grown by SLOT,
    with the centre slot filled): no cusp at x = 0, so the fixed-side slot rims are one smooth chain."""
    z0, z1 = SLOT_Z0, SLOT_Z0 + 1.0
    g = p.SLOT
    yb = SLOT_REAR_Y - 10 - g
    sx, sy = p.SPINE_HW, SPINE_FRONT_Y
    t = box(-p.BTN_X - g, p.BTN_X + g, yb, p.NOSE_Y + 5, z0, z1).val() \
        .cut(box(-sx, sx, yb - 1, sy, z0, z1).val()) \
        .intersect(plan_offset_prism(p.LIP, z0, z1))
    t = solid(t)
    vert = [e for e in t.Edges() if e.BoundingBox().zmax - e.BoundingBox().zmin > 0.5 and e.BoundingBox().ymin > p.HINGE_Y]
    concave = [e for e in vert if abs(abs(e.Center().x) - sx) < 1e-3 and abs(e.Center().y - sy) < 1e-3]
    convex = [e for e in vert if not any(e.isSame(c) for c in concave)]
    t = solid(t.fillet(TONGUE_R + g, convex))
    t = solid(t.fillet(TONGUE_R - g, [e for e in t.Edges() if abs(abs(e.Center().x) - sx) < 1e-3
                                     and abs(e.Center().y - sy) < 1e-3 and e.geomType() == "LINE"]))
    f = [fc for fc in t.Faces() if fc.geomType() == "PLANE" and abs(fc.Center().z - z0) < 1e-6][0]
    return f.outerWire()


TONGUE_W, TONGUE_W2 = _tongue_wire(0.0), _outer_slot_wire()
_TONGUE_POLY = np.array([[TONGUE_W.positionAt(i / 1500).x, TONGUE_W.positionAt(i / 1500).y] for i in range(1500)])
_SLOT_EDGE_PTS = np.vstack([_TONGUE_POLY, [[TONGUE_W2.positionAt(i / 1500).x, TONGUE_W2.positionAt(i / 1500).y]
                                           for i in range(1500)]])


def in_button(x, y, side):
    """Plan point inside the button on side -1 (left) / +1 (right)."""
    return y >= p.HINGE_Y + p.HINGE_LEN / 2 and _in_poly(_TONGUE_POLY, x * side, y)


@functools.lru_cache(maxsize=None)
def slot_tool():
    """All slots, cut square to the skin: the patch of outer skin inside the slot outline (_slot_prism) is moved
    0.5 mm out along its normal and thickened 4.0 mm inward, so every slot wall runs along the surface normal. A
    vertical cut through skin sloping at angle a leaves the uphill edge 0.6 tan(a) above the downhill one (0.7 mm at
    the button tips, 48 deg); along the normal the gap is a constant 0.6 mm with no step."""
    if p.DOME_MODE == "ref":
        return _ref_slot_tool()
    from OCP.BRepOffset import BRepOffset_MakeSimpleOffset
    band = outer_body().intersect(_slot_prism())
    skin = [f for f in band.Faces() if f.geomType() == "BSPLINE" and f.Center().z > SLOT_Z0 + 0.5
            and abs(f.normalAt().z) > 0.05]
    sh = cq.Shell.makeShell(skin)
    up = BRepOffset_MakeSimpleOffset(sh.wrapped, 0.5)
    up.Perform()
    th = BRepOffset_MakeSimpleOffset(up.GetResultShape(), -4.0)
    th.SetBuildSolidFlag(True)
    th.Perform()
    out = solid(cq.Shape.cast(th.GetResultShape()))
    assert out.isValid() and len(out.Solids()) == 1, "normal slot cutter"
    return out


def _ref_slot_tool(out_=0.5, in_=2.1, step=0.4):
    """Reference skin: the slots square to the skin, built directly. For each button, the slot centreline (the
    button outline grown by SLOT / 2, the part in front of SLOT_REAR_Y) is put on the exact outer skin; at every
    point a rectangle across the slot in the plane of the surface normal, from out_ above the skin to in_ below it;
    one loft through them per button. The centre slot (shared by both buttons): each side's rectangle reaches only
    0.1 past the centre line, so the two lofts overlap there without coinciding. Plus the relief holes at the slot
    ends (vertical, as before). (The offset-patch cutter of the parametric skin builds no solid on this skin.)"""
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    from OCP.GeomAPI import GeomAPI_ProjectPointOnSurf
    from OCP.BRep import BRep_Tool
    from OCP.gp import gp_Pnt, gp_Vec
    ob = outer_body()
    srf = BRep_Tool.Surface_s([f for f in ob.Faces() if f.geomType() == "BSPLINE"][0].wrapped)
    loop = TONGUE_W.offset2D(p.SLOT / 2, "arc")[0]
    n = int(loop.Length() / step)
    P2 = np.array([[loop.positionAt(i / n).x, loop.positionAt(i / n).y] for i in range(n)])
    keep = P2[:, 1] >= SLOT_REAR_Y
    k0 = int(np.flatnonzero(keep & ~np.roll(keep, 1))[0])
    P2 = np.roll(P2, -k0, axis=0)[:int(keep.sum())]
    # centre-slot weight: 1 on the centre line (|x| < 0.05), fading to 0 over 1.5 mm of path
    cen = (np.abs(P2[:, 0]) < 0.05).astype(float)
    from scipy.ndimage import uniform_filter1d
    wc = np.clip(uniform_filter1d(cen, size=int(1.5 / step) * 2 + 1) * 2.0, 0.0, 1.0)
    cutters = []
    for side in (1, -1):
        Q = P2 * [side, 1]
        T = np.gradient(Q, axis=0)
        T /= np.linalg.norm(T, axis=1)[:, None]
        zs = z_skin(Q[:, 0], Q[:, 1])
        ts = BRepOffsetAPI_ThruSections(True, False, 1e-6)
        for (x, y), (tx, ty), z, w in zip(Q, T, zs, wc):
            pr = GeomAPI_ProjectPointOnSurf(gp_Pnt(float(x), float(y), float(z)), srf)
            u, v = pr.LowerDistanceParameters()
            pt, du, dv = gp_Pnt(), gp_Vec(), gp_Vec()
            srf.D1(u, v, pt, du, dv)
            c = np.array(pt.Coord())
            nrm = np.array(du.Crossed(dv).Coord())
            nrm /= np.linalg.norm(nrm)
            if nrm[2] < 0:
                nrm = -nrm
            t = np.array([tx, ty, 0.0])
            t -= t.dot(nrm) * nrm
            t /= np.linalg.norm(t)
            b = np.cross(nrm, t)
            if b[0] * side < 0:              # b points toward this side's own tongue
                b = -b
            wo, wi = p.SLOT / 2, p.SLOT / 2 * (1 - w) + 0.1 * w     # own side / centre side half-widths
            corners = [c + b * wo + nrm * out_, c - b * wi + nrm * out_, c - b * wi - nrm * in_, c + b * wo - nrm * in_]
            ts.AddWire(cq.Wire.makePolygon([cq.Vector(*q) for q in corners], close=True).wrapped)
        ts.CheckCompatibility(False)
        ts.Build()
        cut_ = solid(cq.Shape.cast(ts.Shape()))
        if cut_.Volume() < 0:                # the rectangles' winding makes an inside-out loft: flip it
            cut_ = solid(cq.Shape.cast(cut_.wrapped.Reversed()))
        cutters.append(cut_)
    ok = [(c.isValid(), round(c.Volume(), 2)) for c in cutters]
    out = cutters[0].fuse(cutters[1])
    for x in (p.SPINE_HW + p.SLOT / 2, p.BTN_X + p.SLOT / 2):
        for sg in (-1, 1):
            out = out.fuse(cyl_z(sg * x, SLOT_REAR_Y, SLOT_Z0, Z_HI, p.RELIEF_D).val())
    out = solid(out.clean())
    assert out.isValid(), f"reference slot cutter (lofts valid: {ok})"
    return out


def _slot_prism():
    """All slots in one vertical cutter: inside the outer slot outline, outside both buttons, behind nothing past
    SLOT_REAR_Y; plus the relief holes at the slot ends."""
    z0, z1 = SLOT_Z0, Z_HI
    ext = lambda w: cq.Solid.extrudeLinear(cq.Face.makeFromWires(w), cq.Vector(0, 0, z1 - z0))
    right = ext(TONGUE_W)
    ring = ext(TONGUE_W2).cut(right).cut(right.mirror("YZ"))
    ring = solid(ring.intersect(box(-60, 60, SLOT_REAR_Y, p.NOSE_Y + 10, z0, z1).val()))
    for x in (p.SPINE_HW + p.SLOT / 2, p.BTN_X + p.SLOT / 2):
        for sg in (-1, 1):
            ring = solid(ring.fuse(cyl_z(sg * x, SLOT_REAR_Y, z0, z1, p.RELIEF_D).val()))
    return solid(ring.clean())


def hinge_tube(side):
    """Living hinge: a round groove (radius HINGE_GROOVE_R) swept under the button's rear edge along the curved
    skin, its axis HINGE_T + HINGE_GROOVE_R below the outer skin, so HINGE_T of skin is left."""
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    rg, target = HINGE_GROOVE_R, p.HINGE_T + HINGE_GROOVE_R
    xa, xb = p.SPINE_HW + p.SLOT / 2, p.BTN_X + p.SLOT / 2     # slot centres: the groove is trimmed there
    xs = np.linspace(xa - 1.5, xb + 1.5, 29) * side
    zs = z_skin(xs, np.full_like(xs, p.HINGE_Y), target)
    skin = cq.Shell.makeShell(outer_body().Faces())
    for _ in range(3):           # z_skin's offset is approximate on the curved dome: set the exact skin distance
        for i, (x, z) in enumerate(zip(xs, zs)):
            ex = BRepExtrema_DistShapeShape(cq.Vertex.makeVertex(float(x), p.HINGE_Y, float(z)).wrapped, skin.wrapped)
            ex.Perform()
            q = ex.PointOnShape2(1)
            nz = (q.Z() - z) / ex.Value()
            zs[i] = z - (target - ex.Value()) / max(nz, 0.3)
    path = cq.Wire.assembleEdges([cq.Edge.makeSpline([cq.Vector(float(x), p.HINGE_Y, float(z)) for x, z in zip(xs, zs)])])
    circ = cq.Wire.makeCircle(rg, path.startPoint(), path.tangentAt(0))
    tube = cq.Solid.sweep(circ, [], path, makeSolid=True, isFrenet=False)
    # vertical end planes at the slot centres, taken at the groove's depth: the slots are cut along the surface
    # normal, so on the sloping skin a slot lies further in at depth; the ends stay in the slot void, never in the
    # fixed wall
    def at_depth(x):
        gx = float(z_skin(x + 0.05, p.HINGE_Y) - z_skin(x - 0.05, p.HINGE_Y)) / 0.1
        gy = float(z_skin(x, p.HINGE_Y + 0.05) - z_skin(x, p.HINGE_Y - 0.05)) / 0.1
        return x + target * gx / math.sqrt(1 + gx * gx + gy * gy)
    x0, x1 = sorted((side * at_depth(xa), side * at_depth(xb)))
    return solid(tube.intersect(box(x0, x1, p.HINGE_Y - 5, p.HINGE_Y + 5, p.FLOOR, Z_HI).val()))


@functools.lru_cache(maxsize=None)
@disk_cache(*GEOM_DEPS, "cavity_body", "plan_offset_prism")
def tongue_keepout():
    """Band under both buttons (and their slots): the skin plus TONGUE_CLR below it. Fixed features are trimmed
    out of it so the buttons can travel."""
    m = p.SLOT + 2.0                       # past the slot (which leans ~1 mm at depth: cut along the normal) and its rim round
    reg = box(-p.BTN_X - m, p.BTN_X + m, SLOT_REAR_Y - m, p.NOSE_Y + 5, p.FLOOR, Z_HI).val() \
        .intersect(plan_offset_prism(p.LIP - 1.0, p.FLOOR, Z_HI))
    return solid(reg.cut(cavity_body().translate(cq.Vector(0, 0, -p.TONGUE_CLR))))


def _socket_z0(k):
    """Socket bottom: KEY_SOCKET_H below the lowest inner-skin point over the socket disk."""
    sx, sy = SW[k]
    a = np.linspace(0, 2 * np.pi, 48)
    r = p.KEY_SOCKET_D / 2
    xs = np.concatenate([[sx], sx + r * np.cos(a)]); ys = np.concatenate([[sy], sy + r * np.sin(a)])
    return float(z_skin(xs, ys, p.WALL).min()) - p.KEY_SOCKET_H


SOCKET_Z0 = dict(KEY_SOCKET_Z0) if p.DOME_MODE == "ref" else {k: _socket_z0(k) for k in SW}


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


def ring_x():
    x_left = WHEEL_X - p.WHEEL_W / 2
    return x_left, x_left + p.DETENT_RING_W


TOP_SHELL_DEPS = (*GEOM_DEPS, *ROUND_DEPS, "hollow", "usb_opening", "slot_tool", "_ref_slot_tool", "_slot_prism", "_tongue_wire", "_outer_slot_wire",
                  "hinge_tube", "wheel_open", "outer_body", "cavity_body", "plan_offset_prism", "required_edge",
                  "_required_point", "_skin_mesh")


@disk_cache(*TOP_SHELL_DEPS)
def top_shell():
    """Stage 1 of the top: skin, split / USB rounds, slots, hinge grooves, wheel opening."""
    hol = hollow()[1]
    t = solid(hol.intersect(box(-90, 90, -120, 120, p.Z_SPLIT, Z_HI).val()))
    # USB notch first, then round its edges together with the split edges (one fillet operation)
    u = solid(usb_opening())
    t = solid(t.cut(u))
    es = [e for e in sharp_edges(t) if _near(e, u.BoundingBox(), 0.6) or abs(e.positionAt(0.5).z - p.Z_SPLIT) < 0.05]
    if p.DOME_MODE == "ref":
        es = [e for e in es if required_edge(e)]
    t, failed = fillet_edges(t, es, p.R_MIN)
    if failed:
        FILLET_FAILS.setdefault("split+usb", []).extend(failed)
    ox0, ox1, oy0, oy1 = WOPEN
    # ---- cuts in the skin first (slots, hinge grooves, wheel opening)
    t = sub(t, slot_tool(), "slot")
    for side in (-1, 1):
        t = sub(t, hinge_tube(side), "hinge")
    t = sub(t, box(ox0, ox1, oy0, oy1, SLOT_Z0, Z_HI).edges("|Z").fillet(1.0), "wheel_open")
    return t


def front_post(h):
    """Screw post at a front hole (H1 / H2), which sits under a button: boss + rib to the fixed side wall, both cut
    TONGUE_CLR (vertical) below the button skin's underside, so the button can travel. Boss foot and rib edges are
    rounded on the plain blanks, before the skin trim."""
    hx, hy = HOLES[h]
    emb = skin_body(p.WALL - EMBED, p.FLOOR)
    sg = 1 if hx > 0 else -1
    xa, xb = sorted((hx, sg * (80.0 if p.DOME_MODE == "ref" else p.SIDE + 1)))
    cyl = cyl_z(hx, hy, p.board_z_top, Z_HI, p.BOSS_D)
    boss = solid(cyl.faces("<Z").edges().fillet(p.R_MIN).val().intersect(emb))
    rib = solid(box(xa, xb, hy - 1.0, hy + 1.0, p.board_z_top + 1.0, Z_HI).edges("|X").fillet(p.R_MIN)
                .val().intersect(emb))
    return solid(boss.fuse(rib).cut(tongue_keepout()))


# The top is built in cached stages (each stage's cache key holds its own source and that of every stage before
# it, so a fix to one stage reruns only that stage and the ones after it):
#   top_shell -> top_posts -> top_wheel_mounts -> top_holes -> top_part (outer-edge rounding)
_top_posts_deps = (*TOP_SHELL_DEPS, "top_shell", "tongue_keepout", "_socket_z0", "front_post")


@disk_cache(*_top_posts_deps)
def top_posts():
    """Stage 2: key sockets under the buttons, board bosses (front ones shortened, with a rib), cover-screw bosses.
    Blanks run up to Z_HI and are cut back to EMBED into the skin (never to the outer surface, which would rebuild
    the rounded slot / opening rims)."""
    t = top_shell()
    emb = skin_body(p.WALL - EMBED, p.FLOOR)
    for k in SW:
        sx, sy = SW[k]
        sock = pre_round(cyl_z(sx, sy, SOCKET_Z0[k], Z_HI, p.KEY_SOCKET_D), p.R_MIN)
        t = add(t, solid(sock.intersect(emb)), "socket", pre=False)
    for h, (hx, hy) in HOLES.items():
        if h in FRONT_HOLES:
            # front: round the rib / keep-out cut edges on the loose feature, where OCC can (pre_round)
            t = add(t, front_post(h), "boss", pre=True)
        else:
            t = add(t, solid(cyl_z(hx, hy, p.board_z_top, Z_HI, p.BOSS_D).val().intersect(emb)), "boss", pre=False)
    for sx in (-p.REAR_SCREW_X, p.REAR_SCREW_X):
        boss = pre_round(cyl_z(sx, p.REAR_SCREW_Y, p.REAR_IFACE_Z, Z_HI, p.BOSS_D_REAR), p.R_MIN)
        t = add(t, solid(boss.intersect(emb)), "boss", pre=False)
    return t


_top_mount_deps = (*_top_posts_deps, "top_posts", "detent_geometry", "ring_x")


@disk_cache(*_top_mount_deps)
def top_wheel_mounts():
    """Stage 3: wheel brackets and the detent arm."""
    t = top_posts()
    emb = skin_body(p.WALL - EMBED, p.FLOOR)
    ox0, ox1, oy0, oy1 = WOPEN
    bz0 = WHEEL_Z - 2.5
    for xa, xb in ((ox0 - p.BRACKET_T, ox0), (ox1, ox1 + p.BRACKET_T)):
        br = pre_round(box(xa, xb, WHEEL_Y - p.BRACKET_LEN / 2, WHEEL_Y + p.BRACKET_LEN / 2, bz0, Z_HI), 0.5)
        t = add(t, solid(br.intersect(emb)), "bracket", r=0.5, pre=False)
    # detent: bracket block behind the opening, cantilever arm, nub
    dg = detent_geometry()
    rx0, rx1 = ring_x()
    ax0, ax1 = rx0 + 0.1, rx1 - 0.1
    arm_y0 = oy0 - 0.3 - 1.6 - 5.0
    ny, nz, rn = dg["nub_y"], dg["nub_z"], p.DETENT_NUB_R
    top_z = nz + rn                        # the nub is the arm's rounded upper-front corner (centre ny, nz)
    bot_z = top_z - p.DETENT_ARM_T
    zin = float(z_skin(np.array([ax0, ax1, ax0, ax1]), np.array([arm_y0, arm_y0, ny + rn, ny + rn]), p.WALL).min())
    prof = [(arm_y0, Z_HI), (arm_y0, bot_z), (ny + rn, bot_z), (ny + rn, top_z),
            (arm_y0 + 1.6, top_z), (arm_y0 + 1.6, Z_HI)]
    arm = (rounded_poly(cq.Workplane("YZ").workplane(offset=ax0), prof, [0, 0.5, 0.5, rn, 1.0, 0])
           .extrude(ax1 - ax0))
    arm = pre_round(arm, p.R_MIN, skip=lambda e: e.BoundingBox().zmin > zin - 0.3)
    t = add(t, solid(arm.intersect(emb)), "detent", pre=False)
    return t


_top_holes_deps = (*_top_mount_deps, "top_wheel_mounts")


@disk_cache(*_top_holes_deps)
def top_holes():
    """Stage 4: axle seats, plunger socket holes, screw pilots."""
    t = top_wheel_mounts()
    ox0, ox1, oy0, oy1 = WOPEN
    bz0 = WHEEL_Z - 2.5
    for xa, xb in ((ox0 - p.BRACKET_T - 1, ox0 + 0.01), (ox1 - 0.01, ox1 + p.BRACKET_T + 1)):
        seat = cyl_x(xa, xb, WHEEL_Y, WHEEL_Z, p.AXLE_SEAT_D).union(
            box(xa, xb, WHEEL_Y - p.AXLE_THROAT / 2, WHEEL_Y + p.AXLE_THROAT / 2, bz0 - 1, WHEEL_Z))
        t = sub(t, seat, "axle_seat")
    for k in SW:
        sx, sy = SW[k]
        t = sub(t, cyl_z(sx, sy, SOCKET_Z0[k] - 1, SOCKET_Z0[k] + p.KEY_HOLE_DEPTH, p.KEY_HOLE_D), "socket_hole")
    for h, (hx, hy) in HOLES.items():
        depth = p.PILOT_DEPTH_FRONT if h in FRONT_HOLES else p.PILOT_DEPTH
        t = sub(t, cyl_z(hx, hy, p.board_z_top - 1, p.board_z_top + depth, p.SCREW_PILOT_D), "pilot")
    for sx in (-p.REAR_SCREW_X, p.REAR_SCREW_X):
        t = sub(t, cyl_z(sx, p.REAR_SCREW_Y, p.REAR_IFACE_Z - 1, p.REAR_IFACE_Z + p.PILOT_DEPTH_REAR, p.SCREW_PILOT_D),
                "pilot")
    return solid(t)


@disk_cache(*_top_holes_deps, "top_holes", "round_all", "required_edge", "_required_point", "_skin_faces")
def top_part():
    """Stage 5 (last): round the outer-skin and USB-C edges still sharp (edge rule); returns (top, sharp edges left)."""
    return round_all(top_holes(), p.R_MIN, rounds=1, name="top", keep=required_edge)


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
    zb = SOCKET_Z0[k]
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
        "base": base_part(),
        "top": top_holes(),
        "wheel": wheel_part(),
        "key_L": plunger("SW_L1"),
        "key_R": plunger("SW_R1"),
    }
    sharp = {}
    if round_edges:
        for k in list(parts):
            if k == "top":
                parts[k], left = top_part()          # cached last stage
                left = [cq.Shape.cast(e.wrapped) for e in left]   # unpickled edges come back untyped
            else:
                parts[k], left = round_all(parts[k], p.R_MIN, rounds=1, name=k, keep=required_edge)
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
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRepTools import BRepTools
    def mesh(s):
        # absolute linear deflection 0.01 mm bounds the chord sag on every face (cadquery's tessellate() is relative
        # to face size: 0.2 rad on the R~70 dome sags ~0.35 mm, which reads as thin wall). The triangles are read
        # straight from this mesh: calling tessellate() afterwards re-meshed the whole part with its relative
        # deflection and a 0.05 rad angle (5.9 M triangles, 13 GB in the ray cast, killed for lack of memory).
        # The angular limit is 0.5 rad: the 0.01 mm linear limit already sets the accuracy.
        from OCP.BRep import BRep_Tool
        from OCP.TopLoc import TopLoc_Location
        from OCP.TopAbs import TopAbs_REVERSED
        BRepTools.Clean_s(s.wrapped)
        _imesh(s, 0.01)
        V, F, off = [], [], 0
        for f in s.Faces():
            loc = TopLoc_Location()
            tri = BRep_Tool.Triangulation_s(f.wrapped, loc)
            if tri is None:
                continue
            tr = loc.Transformation()
            V += [tri.Node(i).Transformed(tr).Coord() for i in range(1, tri.NbNodes() + 1)]
            rev = f.wrapped.Orientation() == TopAbs_REVERSED
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                F.append((off + a - 1, off + c - 1, off + b - 1) if rev else (off + a - 1, off + b - 1, off + c - 1))
            off += tri.NbNodes()
        return trimesh.Trimesh(np.array(V), np.array(F), process=True)
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
            th.append([min(h), tuple(np.round(origins[r], 2)), r])
    # the mesh can still sag on the big lofted faces: every reading below the wall is re-measured on the
    # exact B-rep (true skin point -> nearest face facing the other way); a real thin spot stays thin
    faces = [(f, f.BoundingBox()) for f in part.Faces()]
    for t in th:
        t.append(None)
        if t[0] < (p.HINGE_T if in_hinge(t[1]) else p.WALL) - 0.02:
            w, C = _exact_wall(faces, origins[t[2]], nrm[t[2]])
            if w < float("inf"):        # exact min distance to the inner side (<= the true ray length)
                t[0], t[3] = w, C
    th = [(t[0], t[1], t[3]) for t in th]   # (thickness, skin point, nearest inner point if re-measured)
    th.sort(key=lambda t: t[0])
    return th


def _exact_wall(faces, P, n, reach=3.0):
    """Exact wall thickness at a point P near the outer skin with outward normal n: P is moved onto the nearest
    face facing along n, then (distance, point) to the nearest face facing against n is returned."""
    from OCP.BRep import BRep_Tool
    from OCP.GeomAPI import GeomAPI_ProjectPointOnSurf
    from OCP.BRepGProp import BRepGProp_Face
    from OCP.gp import gp_Pnt, gp_Vec
    def near(Q):
        out = []
        for f, bb in faces:
            if (bb.xmin - reach <= Q[0] <= bb.xmax + reach and bb.ymin - reach <= Q[1] <= bb.ymax + reach
                    and bb.zmin - reach <= Q[2] <= bb.zmax + reach):
                e = BRepExtrema_DistShapeShape(cq.Vertex.makeVertex(*map(float, Q)).wrapped, f.wrapped)
                e.Perform()
                if not e.IsDone() or e.NbSolution() == 0:
                    continue
                q = e.PointOnShape2(1)
                pr = GeomAPI_ProjectPointOnSurf(q, BRep_Tool.Surface_s(f.wrapped))
                if pr.NbPoints() == 0:
                    continue
                u, v = pr.LowerDistanceParameters()
                X, N = gp_Pnt(), gp_Vec()
                BRepGProp_Face(f.wrapped).Normal(u, v, X, N)
                if N.Magnitude() < 1e-9:
                    continue
                dot = (N.X() * n[0] + N.Y() * n[1] + N.Z() * n[2]) / N.Magnitude()
                out.append((e.Value(), dot, (q.X(), q.Y(), q.Z())))
        return out
    outer = [o for o in near(P) if o[1] > 0.5]
    if not outer:
        return float("inf"), None
    Q = min(outer)[2]
    inner = [(o[0], o[2]) for o in near(Q) if o[1] < -0.2]
    return min(inner) if inner else (float("inf"), None)


def in_hinge(pt, tol=0.3):
    ax = abs(pt[0])
    return (p.SPINE_HW + p.SLOT - tol <= ax <= p.BTN_X + tol and abs(pt[1] - p.HINGE_Y) <= p.HINGE_LEN / 2 + tol
            and pt[2] > SLOT_Z0)


def _near_slot(x, y, d):
    """Plan point within d of a slot edge (or inside a slot / relief hole)."""
    if y < SLOT_REAR_Y - p.RELIEF_D / 2 - d:
        return False
    dmin = float(np.min(np.hypot(_SLOT_EDGE_PTS[:, 0] - abs(x), _SLOT_EDGE_PTS[:, 1] - y)))
    return dmin < d


def near_cut_edge(pt, d=0.75):
    """Sample points on the rim of a cut (slot, relief hole, wheel opening, USB opening) or on the
    parting-line groove measure the edge, not the wall: skip them."""
    x, y, z = pt
    if abs(z - p.Z_SPLIT) < 1.0:    # parting line: both parts carry 0.5 rounds there (small V-groove)
        return True
    if z > SLOT_Z0 - 0.5 and _near_slot(x, y, d):
        return True
    ox0, ox1, oy0, oy1 = WOPEN
    if ox0 - d <= x <= ox1 + d and oy0 - d <= y <= oy1 + d and z > SLOT_Z0 - 0.5:
        return True
    if x < -p.W / 2 + p.WALL + 3.0 and abs(y - J5_Y) < (p.PLUG_W / 2 + p.PLUG_OPEN_CLR + d) and \
            abs(z - J5_Z) < (p.PLUG_H / 2 + p.PLUG_OPEN_CLR + d):
        return True
    return False


def wheel_open_tool():
    """Wheel-opening cutter (the same solid top_shell() cuts; kept separate so top_shell's cache key is unchanged)."""
    ox0, ox1, oy0, oy1 = WOPEN
    return solid(box(ox0, ox1, oy0, oy1, SLOT_Z0, Z_HI).edges("|Z").fillet(1.0))


def cut_tools():
    """Every cutter that opens the skin: slots (with their relief holes), wheel opening, USB-C opening."""
    return {"slots": solid(slot_tool()), "wheel_opening": wheel_open_tool(), "usb_c": solid(usb_opening())}


def on_cut_wall(pt, tools, tol=1e-3):
    """Point lies on the boundary of an opening cutter, i.e. on a face that the cut made (slot / opening wall)."""
    v = cq.Vertex.makeVertex(*map(float, pt)).wrapped
    for nm, t in tools.items():
        e = BRepExtrema_DistShapeShape(v, cq.Compound.makeCompound(t.Faces()).wrapped)
        e.Perform()
        if e.Value() < tol:
            return nm
    return None


def wall_report(parts, ob):
    """Thickness of the closed shell (base + top fused, so the split line is not an edge). Readings are excluded
    (and counted) when they are on a living hinge, within 0.75 mm of a cut rim / on the parting line, or when the
    nearest inside surface is the wall of a slot or opening: those measure the distance to a cut face, not the
    skin thickness. Every reading that reaches the inside of the dome (or a rim round) is kept."""
    sh = parts["base"].fuse(parts["top"])
    th = wall_thickness(sh, ob, n=12000)
    tools = cut_tools()
    hinge = [t for t in th if in_hinge(t[1])]
    rest = [t for t in th if not in_hinge(t[1])]
    near_rim = [t for t in rest if near_cut_edge(t[1])]
    rest = [t for t in rest if not near_cut_edge(t[1])]
    cut_wall = {}
    other = []
    for t in rest:
        nm = on_cut_wall(t[2], tools) if (t[0] < p.WALL - 0.02 and t[2] is not None) else None
        if nm:
            cut_wall.setdefault(nm, []).append(t)
        else:
            other.append(t)
    thin = [t for t in other if t[0] < p.WALL - 0.02]
    f3 = lambda v: tuple(round(float(c), 2) for c in v)
    return dict(samples=len(th), samples_used=len(other), min_excluding_hinges=round(float(other[0][0]), 3),
                at=f3(other[0][1]), hinge_min=round(float(min(t[0] for t in hinge)), 3) if hinge else None,
                below_1p48=[(round(float(t[0]), 3), f3(t[1])) for t in thin[:20]], n_below_1p48=len(thin),
                excluded=dict(
                    living_hinge=len(hinge),
                    near_cut_rim_or_parting_line=len(near_rim),
                    nearest_inside_surface_is_cut_wall={k: len(v) for k, v in cut_wall.items()},
                    cut_wall_readings=[dict(reading=round(float(t[0]), 3), at=f3(t[1]), cut_face_point=f3(t[2]),
                                            opening=k) for k, v in cut_wall.items() for t in v]))


# Web limit = the stricter JLC3DP wall rule that applies to nylon (read 2026-11-26):
#   instant-quote page https://jlc3dp.com/3d-printing-quote, "Upload 3D models": "Wall thickness > 1.2mm, Thinnest part
#   >= 0.8mm" (every upload, nylon included); MJF page https://jlc3dp.com/3d-printing/multi-jet-fusion, "Design
#   Guidelines for MJF": "Recommended wall thickness 1.0mm"; PA12-HP page: "Wall thickness: 1mm".
# The size table in https://jlc3dp.com/help/article/3D-Printing-Design-Guideline ("2. Wall Thickness", Nylon (MJF/SLS)
# 1.0 / 1.2 / 1.5 / 2.0 / 2.0 mm for 5x5 ... 200x200 mm parts) is unlabelled size guidance ("we recommend that you
# increase your wall thickness ..."), not a stated minimum. (Lower case: an upper-case constant or a P field would
# enter every stage's cache key and force a full rebuild.)
web_min_jlc = 1.2


def opening_volumes():
    """Each opening as the exact material its cutter removes from the shell (cutter x hollow shell): slots with
    relief holes, wheel opening, USB-C opening, and the 4 cover-screw holes through the floor (countersink +
    clearance hole)."""
    hol = hollow()[1]
    out = {k: solid(t.intersect(hol)) for k, t in cut_tools().items()}
    for nm, (sx, sy) in zip(("screw_H1", "screw_H2", "screw_rear_L", "screw_rear_R"), COVER_SCREWS):
        csk = cq.Solid.makeCone(p.CSK_D / 2 + 1.0, 0.0, p.CSK_D / 2 + 1.0, cq.Vector(sx, sy, -1.0))
        hole = solid(cyl_z(sx, sy, -1, p.REAR_IFACE_Z + 1, p.SCREW_CLEAR_D))
        out[nm] = solid(csk.fuse(hole).intersect(hol))
    return out


def outline_at(z):
    """The outer skin's outline at height z (section of the outer body)."""
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Section
    from OCP.gp import gp_Pln, gp_Pnt, gp_Dir
    sec = BRepAlgoAPI_Section(outer_body().wrapped, gp_Pln(gp_Pnt(0, 0, z), gp_Dir(0, 0, 1)))
    sec.Build()
    return cq.Shape.cast(sec.Shape())


def web_report():
    """Material width between every pair of openings, and from each opening to the part's outer edge (the outline
    at the split line z = Z_SPLIT, and for openings in the base also at the bottom, z = 0). Exact B-rep distance
    between the removed volumes, i.e. between the cut walls. Fails below web_min_jlc."""
    vols = opening_volumes()
    names = list(vols)
    webs = []
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            webs.append(dict(between=(names[i], names[j]), width=round(min_dist(vols[names[i]], vols[names[j]]), 3)))
    edge = {"outline at split line (z = %.0f)" % p.Z_SPLIT: outline_at(p.Z_SPLIT),
            "outline at bottom (z = 0)": outline_at(0.001)}
    for nm, v in vols.items():
        for en, e in edge.items():
            bb = v.BoundingBox()
            if en.startswith("outline at bottom") and bb.zmin > p.Z_SPLIT - 1e-6:
                continue                      # openings in the top part only: the top's outer edge is the split line
            if en.startswith("outline at split") and bb.zmin < p.Z_SPLIT < bb.zmax:
                webs.append(dict(between=(nm, en), width=None, note="opening crosses the split line"))
                continue
            d = BRepExtrema_DistShapeShape()
            d.LoadS1(cq.Compound.makeCompound(v.Faces()).wrapped)
            d.LoadS2(e.wrapped)
            d.Perform()
            webs.append(dict(between=(nm, en), width=round(d.Value(), 3)))
    measured = [w for w in webs if w["width"] is not None]
    fails = [w for w in measured if w["width"] < web_min_jlc]
    mn = min(measured, key=lambda w: w["width"])
    return dict(min_web=mn, below_limit=fails, passed=not fails, webs=sorted(webs, key=lambda w: (w["width"] is None,
                                                                                               w["width"] or 0)))


_SH = {}


def _pool_init(path):
    import pickle
    global _SH
    with open(path, "rb") as f:
        _SH = pickle.load(f)


def _pool_job(job):
    kind, a, b = job
    if kind == "w":
        return wall_report(_SH["__parts"], _SH["ob"])
    if kind == "web":
        return web_report()
    if kind == "v":
        return common_volume(_SH[a], _SH[b])
    return min_dist(_SH[a], _SH[b])


def _workers(per_worker_gb=7.5):
    """Worker count from the memory still available: a worker holding the lofted top part peaks at ~7.4 GB."""
    try:
        with open("/proc/meminfo") as f:
            avail = next(int(l.split()[1]) for l in f if l.startswith("MemAvailable")) / 1e6
    except (OSError, StopIteration):
        return 2
    return max(1, min(os.cpu_count() or 1, int(avail // per_worker_gb)))


def _rank_components(part, comps, k=8):
    """The k board components nearest to a part, ranked on meshes (0.05 mm deflection; box surfaces sampled every
    0.5 mm). Only these k are then measured exactly: the ranking just picks which ones to measure for the
    nearest-component report; every pass / fail clearance is the exact part-to-part distance."""
    import trimesh
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    _imesh(part, 0.05)
    vs, fs = part.tessellate(0.05, 0.5)
    m = trimesh.Trimesh([(v.x, v.y, v.z) for v in vs], fs, process=False)
    out = []
    for ref, c in comps:
        bb = c.BoundingBox()
        g = lambda a, b: np.linspace(a, b, max(2, int((b - a) / 0.5) + 1))
        X, Y, Z = np.meshgrid(g(bb.xmin, bb.xmax), g(bb.ymin, bb.ymax), g(bb.zmin, bb.zmax), indexing="ij")
        pts = np.column_stack([X.ravel(), Y.ravel(), Z.ravel()])
        on = ((np.isclose(pts[:, 0], bb.xmin) | np.isclose(pts[:, 0], bb.xmax) | np.isclose(pts[:, 1], bb.ymin)
               | np.isclose(pts[:, 1], bb.ymax) | np.isclose(pts[:, 2], bb.zmin) | np.isclose(pts[:, 2], bb.zmax)))
        _, d, _ = trimesh.proximity.closest_point(m, pts[on])
        out.append((float(d.min()), ref))
    return [r for _, r in sorted(out)[:k]]


def _pool_job_keyed(job):
    return job, _pool_job(job)


def shape_hash(s):
    """Hash of a shape's exact B-rep (meshes dropped first, so a tessellated copy hashes the same)."""
    import hashlib, io
    from OCP.BRepTools import BRepTools
    if isinstance(s, dict):
        return hashlib.sha1("".join(shape_hash(v) for _, v in sorted(s.items())).encode()).hexdigest()
    BRepTools.Clean_s(s.wrapped)
    buf = io.BytesIO()
    s.exportBin(buf)
    return hashlib.sha1(buf.getvalue()).hexdigest()


_check_src = {"d": ("min_dist",), "v": ("common_volume",),
              "w": ("wall_report", "wall_thickness", "_exact_wall", "in_hinge", "_near_slot", "near_cut_edge",
                    "cut_tools", "wheel_open_tool", "on_cut_wall", "slot_tool", "_slot_prism", "usb_opening"),
              "web": ("web_report", "opening_volumes", "outline_at", "min_dist", "cut_tools", "wheel_open_tool", "slot_tool", "_slot_prism",
                      "usb_opening")}


def run_parallel(shapes, jobs, n=None):
    """Run check jobs (("d", a, b) distance, ("v", a, b) overlap volume, ("w", "", "") wall report) on n
    processes; each worker loads the shapes once. The exact queries on the lofted skin are slow (seconds to many
    minutes each), so every result is cached on disk, keyed on the exact B-rep of the shapes it reads, the source
    of that kind of check and the parameters: a rebuild that changes one part re-measures only the jobs that involve it."""
    import hashlib, inspect, multiprocessing as mp, pickle, tempfile
    n = n or _workers()
    h = {k: shape_hash(v) for k, v in shapes.items()}
    salt = {k: repr((asdict(p), "".join(inspect.getsource(globals()[f]) for f in fs))) for k, fs in _check_src.items()}
    def key(job):
        # each job kind is keyed on its own code only: a change to the wall check does not re-run the distances
        kind, a, b = job
        deps = (h["__parts"], h["ob"]) if kind in ("w", "web") else (h[a], h[b])
        return hashlib.sha1(repr((salt[kind], kind, deps)).encode()).hexdigest()[:20]
    cdir = os.path.join(CACHE_DIR, "checks")
    os.makedirs(cdir, exist_ok=True)
    out, todo = {}, []
    for j in jobs:
        f = os.path.join(cdir, key(j) + ".pkl")
        if os.path.exists(f) and not os.environ.get("SHELL_NO_CACHE"):
            with open(f, "rb") as fh:
                out[j] = pickle.load(fh)
        else:
            todo.append(j)
    print(f"checks: {len(jobs) - len(todo)} cached, {len(todo)} to run", flush=True)
    if todo:
        # slow jobs first (anything on the lofted top / base skin), so the pool does not end on one long query
        heavy = ("w", "web", "top", "base", "ob", "cav", "ribs_removed")
        todo.sort(key=lambda j: -sum(x in heavy for x in j))
        with tempfile.NamedTemporaryFile(suffix=".pkl", delete=False) as f:
            pickle.dump(shapes, f)
            path = f.name
        try:
            # ProcessPoolExecutor raises BrokenProcessPool if a worker dies (crash, out of memory); a
            # multiprocessing.Pool silently replaces the worker and waits for the lost job forever
            from concurrent.futures import ProcessPoolExecutor, as_completed
            # the wall and web jobs mesh / cut the whole shell (a worker peaks at ~7.5 GB): they run alone first
            big = [j for j in todo if j[0] in ("w", "web")]
            for batch, nw in ((big, 1), ([j for j in todo if j not in big], n)):
                if not batch:
                    continue
                with ProcessPoolExecutor(nw, mp_context=mp.get_context("spawn"), initializer=_pool_init,
                                         initargs=(path,)) as pool:
                    for fut in as_completed([pool.submit(_pool_job_keyed, j) for j in batch]):
                        j, r = fut.result()
                        out[j] = r
                        with open(os.path.join(cdir, key(j) + ".pkl.tmp"), "wb") as fh:
                            pickle.dump(r, fh)
                        os.replace(os.path.join(cdir, key(j) + ".pkl.tmp"), os.path.join(cdir, key(j) + ".pkl"))
        finally:
            os.remove(path)
    return out


def run_checks(ob, parts, refs, sharp):
    R = {}
    allp = {**parts, **refs}
    names = list(allp)
    bat = allp["battery"]
    ribs_removed = parts["base"].cut(solid(box(-60, 60, -80, 80, -1, BAT_BOX[4] + 1e-3)))   # margin: no face
                                                                              # may touch the shell's rear tip
    pe = solid(plug_envelope())
    cav = cavity_body().cut(box(-60, 60, -80, 80, -10, p.FLOOR).val())
    comps = component_boxes()
    shapes = {**allp, "ribs_removed": ribs_removed, "pe": pe, "ob": ob, "cav": cav, "__parts": parts,
              **{"comp:" + r: c for r, c in comps}}
    pair_names = [(names[i], names[j]) for i in range(len(names)) for j in range(i + 1, len(names))]
    jobs = [("w", "", ""), ("web", "", "")] + [(k, a, b) for a, b in pair_names for k in ("d", "v")]
    jobs += [("d", "battery", "ribs_removed")] + [("d", "battery", k) for k in allp if k not in ("battery", "base")]
    jobs += [("d", "pe", k) for k in allp] + [("d", "ob", "cav")]
    near = {nm: _rank_components(allp[nm], comps) for nm in CLOSEST_PARTS}
    jobs += [("d", nm, "comp:" + r) for nm in CLOSEST_PARTS for r in near[nm]]
    Q = run_parallel(shapes, jobs)
    # 1 + 6: overlap and clearance between every pair
    pairs = []
    for a, b in pair_names:
            d = Q[("d", a, b)]
            v = Q[("v", a, b)]
            pairs.append(dict(a=a, b=b, min_clearance=round(d, 3), overlap_mm3=round(v, 4),
                              note=CONTACT_OK.get(frozenset((a, b)), "")))
    R["pairs"] = pairs
    R["overlaps"] = [q for q in pairs if q["overlap_mm3"] > 1e-3]
    R["unexpected_contacts"] = [q for q in pairs if q["min_clearance"] < 1e-6 and not q["note"]]
    # battery clearance to everything except its rib standoffs
    R["battery_clearance_excl_ribs"] = {
        "base (walls, clips, stops, columns)": round(Q[("d", "battery", "ribs_removed")], 3),
        **{k: round(Q[("d", "battery", k)], 3) for k in allp if k not in ("battery", "base")},
    }
    # 2: key centring
    R["key_centring"] = {}
    a = np.linspace(0, 2 * np.pi, 72)
    for k, part in (("SW_L1", "key_L"), ("SW_R1", "key_R")):
        sx, sy = SW[k]
        bb = parts[part].BoundingBox()
        cx, cy = (bb.xmin + bb.xmax) / 2, (bb.ymin + bb.ymax) / 2
        side = -1 if sx < 0 else 1
        rim = [(sx + 3.0 * math.cos(t), sy + 3.0 * math.sin(t)) for t in a]   # socket rim (KEY_SOCKET_D / 2)
        R["key_centring"][k] = dict(plunger_centre=(round(cx, 3), round(cy, 3)),
                                    switch_centre=(round(sx, 3), round(sy, 3)),
                                    offset=round(math.hypot(cx - sx, cy - sy), 3), limit=0.5,
                                    button="left" if side < 0 else "right",
                                    socket_inside_own_button=all(in_button(x, y, side) for x, y in rim))
    R["front_posts"] = front_post_report(parts["top"])
    R["screws"] = screw_report(parts["top"])
    R["buttons"] = dict(outer_edge_x=p.BTN_X, spine_half_width=p.SPINE_HW, spine_front_y=round(SPINE_FRONT_Y, 3),
                        hinge_y=p.HINGE_Y, lip=p.LIP, slot=p.SLOT, hinge_t=p.HINGE_T,
                        symmetric_about_x0="by construction (every slot / hinge cutter is mirrored)")
    # 3: height
    zmax = max(s.BoundingBox().zmax for s in allp.values())
    zmin = min(s.BoundingBox().zmin for s in (parts["base"],))
    R["height"] = dict(shell_bottom=round(zmin, 3), skin_top=round(parts["top"].BoundingBox().zmax, 3),
                       wheel_top=round(parts["wheel"].BoundingBox().zmax, 3), overall_top=round(zmax, 3),
                       limit=(round(17.80273 * REF_SCALE + 0.1, 2) if p.DOME_MODE == "ref" else 38.0),
                       wheel_proud_of_skin=round(parts["wheel"].BoundingBox().zmax - float(z_skin(WHEEL_X, WHEEL_Y)), 3))
    # 4: USB
    R["usb"] = dict(j5_mouth_x=round(J5_MOUTH_X, 3), j5_centre_y=round(J5_Y, 3), j5_centre_z=round(J5_Z, 3),
                    opening_centre=(round(-p.W / 2, 3), round(J5_Y, 3), round(J5_Z, 3)),
                    opening_wall="left (-X)", opening_size=(p.PLUG_W + 2 * p.PLUG_OPEN_CLR, p.PLUG_H + 2 * p.PLUG_OPEN_CLR),
                    plug_clearance={k: round(Q[("d", "pe", k)], 3) for k in allp},
                    recess_depth_outer_skin_to_j5=round(J5_MOUTH_X + p.W / 2, 3))
    # 5: walls and edges
    # plan distance from the slots' outer outline (both buttons, symmetric) to the outer outline
    w2 = np.array([[TONGUE_W2.positionAt(i / 3000).x, TONGUE_W2.positionAt(i / 3000).y] for i in range(3000)])
    w2 = w2[w2[:, 1] >= SLOT_REAR_Y]
    dd = np.min(np.hypot(w2[:, None, 0] - _PLAN_POLY[None, :, 0], w2[:, None, 1] - _PLAN_POLY[None, :, 1]), axis=1)
    i = int(np.argmin(dd))
    R["left_key_rim"] = dict(slot_outer_edge_x=-(p.BTN_X + p.SLOT), wall_outside_slot_min=round(float(dd[i]), 3),
                             at=tuple(round(float(c), 2) for c in w2[i]), required=1.5)
    R["wall_thickness"] = Q[("w", "", "")]
    R["webs"] = dict(Q[("web", "", "")])
    # the limit is applied here, not inside the cached job, so a change of web_min_jlc always takes effect
    meas = [w for w in R["webs"]["webs"] if w["width"] is not None]
    R["webs"].update(limit=web_min_jlc, limit_source="JLC3DP instant-quote page (jlc3dp.com/3d-printing-quote): "
                     "'Wall thickness > 1.2mm, Thinnest part >= 0.8mm'; MJF page: nylon 'Recommended wall thickness 1.0mm'",
                     below_limit=[w for w in meas if w["width"] < web_min_jlc],
                     passed=all(w["width"] >= web_min_jlc for w in meas))
    # exact B-rep minimum distance between the outer skin and the cavity skin (walls + floor + top)
    R["wall_thickness"]["shell_skin_exact_min"] = round(Q[("d", "ob", "cav")], 4)
    R["sharp_edges_left"] = {k: [dict(at=tuple(round(c, 2) for c in e.positionAt(0.5).toTuple()),
                                      len=round(e.Length(), 2), required=required_edge(e)) for e in v]
                             for k, v in sharp.items()}
    R["reduced_radius_edges"] = [dict(feature=t, radius=r, at=m, len=L, required=required_edge(m))
                                 for (t, r, m, L) in REDUCED]
    R["edge_rule"] = dict(rule="outer skin and USB-C opening edges >= R_MIN; edges inside the shell accepted",
                          r_min=p.R_MIN,
                          required_sharp=sum(e["required"] for v in R["sharp_edges_left"].values() for e in v),
                          required_reduced=sum(e["required"] for e in R["reduced_radius_edges"]),
                          accepted_internal_sharp=sum(not e["required"] for v in R["sharp_edges_left"].values() for e in v),
                          accepted_internal_reduced=sum(not e["required"] for e in R["reduced_radius_edges"]))
    R["detent"] = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in detent_geometry().items()}
    R["layout"] = dict(switches={k: tuple(round(c, 3) for c in v) for k, v in SW.items()},
                       holes={k: tuple(round(c, 3) for c in v) for k, v in HOLES.items()},
                       wheel_centre=(round(WHEEL_X, 3), round(WHEEL_Y, 3), round(WHEEL_Z, 3)),
                       wheel_opening=tuple(round(c, 3) for c in WOPEN),
                       battery_box=tuple(round(c, 3) for c in BAT_BOX),
                       board_z=(p.BOARD_Z_BOT, p.board_z_top), switch_top_z=round(SW_TOP_Z, 3),
                       cover_screws=[tuple(round(c, 3) for c in s) for s in COVER_SCREWS])
    R["board_facts"] = board_facts()
    R["closest_components"] = {nm: [(r, round(d, 3)) for d, r in sorted((Q[("d", nm, "comp:" + r)], r)
                                                                         for r in near[nm])[:3]]
                               for nm in CLOSEST_PARTS}
    return R


CLOSEST_PARTS = ("top", "base", "wheel", "key_L", "key_R")


def button_body(top, side):
    """The moving button on side -1 / +1: the top part inside the button outline, in front of the hinge line.
    The slots separate it from everything fixed, so it is one solid unless something fixed is joined to it."""
    pr = cq.Solid.extrudeLinear(cq.Face.makeFromWires(TONGUE_W), cq.Vector(0, 0, Z_HI - p.FLOOR)) \
        .translate(cq.Vector(0, 0, p.FLOOR - SLOT_Z0))
    if side < 0:
        pr = pr.mirror("YZ")
    pr = pr.intersect(box(-60, 60, p.HINGE_Y, 80, p.FLOOR, Z_HI).val())
    return top.intersect(pr).Solids()


def _line_hits(shape, x, y):
    """Exact z of every crossing of the vertical line through (x, y) with the shape's faces."""
    from OCP.BRepIntCurveSurface import BRepIntCurveSurface_Inter
    from OCP.gp import gp_Lin, gp_Pnt, gp_Dir
    it = BRepIntCurveSurface_Inter()
    it.Init(shape.wrapped, gp_Lin(gp_Pnt(float(x), float(y), 0.0), gp_Dir(0, 0, 1)), 1e-7)
    out = []
    while it.More():
        out.append(it.Pnt().Z())
        it.Next()
    return out


def _mesh(s, tol=0.02):
    import trimesh
    vs, fs = s.tessellate(tol, 0.1)
    return trimesh.Trimesh([(v.x, v.y, v.z) for v in vs], fs, process=True)


def _ray_hits(shape, P, d, tmax=40.0):
    """Distances along the ray P + t d (0 < t < tmax) at which it crosses the shape's faces, sorted."""
    from OCP.BRepIntCurveSurface import BRepIntCurveSurface_Inter
    from OCP.gp import gp_Lin, gp_Pnt, gp_Dir
    it = BRepIntCurveSurface_Inter()
    it.Init(shape.wrapped, gp_Lin(gp_Pnt(*map(float, P)), gp_Dir(*map(float, d))), 1e-7)
    out = []
    while it.More():
        if 1e-6 < it.W() < tmax:
            out.append(it.W())
        it.Next()
    return sorted(out)


def screw_report(top):
    """Every screw: head, length, tip height, thread engagement in the top part, spare pilot below the tip, and the
    measured wall around each pilot (radial: rays from the pilot axis in 12 directions at 3 heights, wall = boss
    surface - pilot wall; top: ray up the axis, wall = next surface above the pilot end). Wall limit 1.5 mm."""
    rp = p.SCREW_PILOT_D / 2
    rows = []
    spec = [(h, HOLES[h], "M2.5 x %g flat head (ISO 7046 / DIN 965 size), from below" % p.SCREW_L_FRONT, p.SCREW_L_FRONT,
             p.board_z_top, p.board_z_top + p.PILOT_DEPTH_FRONT) for h in FRONT_HOLES]
    spec += [(h, HOLES[h], "M2.5 x %g pan head (ISO 7045 size), from below the board" % p.SCREW_L_BOARD,
              p.BOARD_Z_BOT + p.SCREW_L_BOARD, p.board_z_top, p.board_z_top + p.PILOT_DEPTH) for h in REAR_HOLES]
    spec += [("rear cover %s" % ("L" if sx < 0 else "R"), (sx, p.REAR_SCREW_Y), "M2.5 x %g flat head, from below" % p.SCREW_L_REAR,
              p.SCREW_L_REAR, p.REAR_IFACE_Z, p.REAR_IFACE_Z + p.PILOT_DEPTH_REAR) for sx in (-p.REAR_SCREW_X, p.REAR_SCREW_X)]
    for name, (x, y), screw, tip, z0, ztop in spec:
        radial = []
        for z in (z0 + 0.5, (z0 + ztop) / 2, ztop - 0.3):
            for k in range(12):
                a = 2 * math.pi * k / 12
                d = (math.cos(a), math.sin(a), 0.0)
                hits = _ray_hits(top, (x, y, z), d, 12.0)
                hits = [h for i, h in enumerate(hits) if i == 0 or h - hits[i - 1] > 1e-4]   # a crossing can be reported twice
                if len(hits) >= 2 and abs(hits[0] - rp) < 0.05:
                    radial.append(hits[1] - hits[0])
                else:
                    radial.append(0.0)              # pilot wall missing (hole broken out) -> no wall
        up = _ray_hits(top, (x, y, ztop - 0.1), (0, 0, 1), 30.0)
        up = [h for i, h in enumerate(up) if i == 0 or h - up[i - 1] > 1e-4]
        top_wall = (up[1] - up[0]) if len(up) >= 2 and up[0] < 0.2 else 0.0
        rows.append(dict(position=name, screw=screw, tip_z=round(tip, 2), thread_from_z=round(z0, 2),
                         engagement=round(tip - z0, 2), pilot_top_z=round(ztop, 2), spare_below_pilot_end=round(ztop - tip, 2),
                         radial_wall_min=round(min(radial), 3), top_wall=round(top_wall, 3),
                         pass_=min(radial) >= 1.5 - 1e-3 and top_wall >= 1.5 - 1e-3 and ztop - tip >= 0.5 and tip - z0 >= 2.5))
    return dict(rows=rows, passed=all(r["pass_"] for r in rows))


def front_post_report(top):
    """Each front screw post vs the button above it: the post must not touch the button, and must stay TONGUE_CLR
    (vertical) below it. Vertical gap from rays on a 0.2 mm grid over the post, the closest ones re-measured on
    the exact B-rep (the button tip swings about the hinge, so its travel there is near vertical); 3D gap = exact
    B-rep distance."""
    out = {}
    for h in FRONT_HOLES:
        hx, hy = HOLES[h]
        side = 1 if hx > 0 else -1
        sol = button_body(top, side)
        btn = max(sol, key=lambda s: s.Volume())
        post = front_post(h)
        mb, mp_ = _mesh(btn), _mesh(post)
        bb = post.BoundingBox()
        gx, gy = np.meshgrid(np.arange(bb.xmin, bb.xmax, 0.2), np.arange(bb.ymin, bb.ymax, 0.2))
        org = np.column_stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)])
        up = np.tile([0.0, 0.0, 1.0], (len(org), 1))
        lp, ip, _ = mp_.ray.intersects_location(org, up, multiple_hits=True)
        lb, ib, _ = mb.ray.intersects_location(org, up, multiple_hits=True)
        ptop = {}
        for l, i in zip(lp, ip):
            ptop[i] = max(ptop.get(i, -1e9), l[2])
        mgap = {}
        for l, i in zip(lb, ib):
            if i in ptop and l[2] > ptop[i] - 1e-6:
                mgap[i] = min(mgap.get(i, 1e9), l[2] - ptop[i])
        # the mesh only finds the candidates (its chords sag on the big skin faces); every ray whose mesh gap is
        # within 0.3 mm of the smallest is re-measured on the exact B-rep
        g0 = min(mgap.values()) if mgap else None
        gap = []
        for i, g in mgap.items():
            if g <= g0 + 0.3:
                zp = _line_hits(post, org[i][0], org[i][1])
                zb = [z for z in _line_hits(btn, org[i][0], org[i][1]) if zp and z > max(zp) - 1e-6]
                if zp and zb:
                    gap.append(min(zb) - max(zp))
        d3 = min_dist(btn, post)
        out[h] = dict(hole=(round(hx, 3), round(hy, 3)), button="left" if side < 0 else "right",
                      post_top_z=round(bb.zmax, 3), vertical_gap_min=round(float(min(gap)), 3) if gap else None,
                      gap_3d_min=round(d3, 3), required_vertical=p.TONGUE_CLR,
                      post_free_of_button=d3 > 1e-3, post_and_rib_one_piece=len(post.Solids()) == 1,
                      rib_reaches_x=round(post.BoundingBox().xmin if side < 0 else post.BoundingBox().xmax, 3),
                      pass_=bool(gap) and min(gap) >= p.TONGUE_CLR - 0.02 and d3 > 1e-3)
    return out


def component_boxes():
    """Board components (F.Fab box x height), excluding the board slab, for the nearest-component report."""
    comps = []
    for ref, q in FPS.items():
        if ref.startswith(("H", "SW_L", "SW_R", "TP")) or q["crtyd"] is None:
            continue
        h, _ = comp_height(q["fp"])
        cx0, cy0, cx1, cy1 = q["fab"] or q["crtyd"]
        a = page2s(cx0, cy0); b = page2s(cx1, cy1)
        comps.append((ref, solid(box(min(a[0], b[0]), max(a[0], b[0]), min(a[1], b[1]), max(a[1], b[1]),
                                     p.board_z_top, p.board_z_top + h))))
    return comps


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
    if M.status() == mf.Error.NoError:
        o = M.to_mesh()
        t = trimesh.Trimesh(o.vert_properties[:, :3], o.tri_verts, process=True)
        assert abs(t.volume - m.volume) < 1e-3 * abs(m.volume) + 1.0, (path, t.volume, m.volume)
        t.export(path)
        return dict(repair="crack weld")
    # fallback: MeshFix (removes non-manifold triangles at OCC meshing defects and refills the holes). Accepted only
    # if the result is one watertight body within 0.02 mm of the original mesh both ways (sampled) and the volume
    # is unchanged to 0.01 %
    import pymeshfix
    mfix = pymeshfix.MeshFix(np.asarray(m.vertices), np.asarray(m.faces))
    mfix.repair(joincomp=False, remove_smallest_components=True)
    t = trimesh.Trimesh(mfix.points, mfix.faces, process=True)
    s1, _ = trimesh.sample.sample_surface(m, 400000, seed=2)
    s2, _ = trimesh.sample.sample_surface(t, 400000, seed=3)
    d1 = float(trimesh.proximity.closest_point(t, s1)[1].max())
    d2 = float(trimesh.proximity.closest_point(m, s2)[1].max())
    closed = t.is_watertight and len(t.split(only_watertight=False)) == 1
    if not closed:
        raise RuntimeError(f"{path}: mesh repair failed (crack weld: {M.status()}; MeshFix left an open mesh)")
    within = max(d1, d2) <= 0.02 and abs(t.volume - m.volume) <= 1e-4 * abs(m.volume)
    if not within:
        print(f"WARNING {os.path.basename(path)}: MeshFix repair deviates {max(d1, d2):.3f} mm (> 0.02) from the OCC "
              f"mesh, dV {t.volume - m.volume:.2f} mm3", flush=True)
    t.export(path)
    return dict(repair="MeshFix", max_deviation_mm=round(max(d1, d2), 4), dV_mm3=round(t.volume - m.volume, 3),
                within_0p02=bool(within))


STL_REPAIR = {}


def export(parts, refs, outdir=HERE):
    # keys.* holds both plungers (2 bodies); key_L.* / key_R.* are single-body files for print services
    # that reject multi-body uploads (JLC3DP flags them as an "assembly").
    out = {"base": parts["base"], "top": parts["top"], "wheel": parts["wheel"],
           "keys": cq.Compound.makeCompound([parts["key_L"], parts["key_R"]]),
           "key_L": parts["key_L"], "key_R": parts["key_R"]}
    from OCP.BRepTools import BRepTools
    for k, s in out.items():
        cq.exporters.export(s, os.path.join(outdir, f"{k}.step"))
        stl = os.path.join(outdir, f"{k}.stl")
        BRepTools.Clean_s(s.wrapped)
        _imesh(s, 0.01)                 # absolute 0.01 mm chord sag, surface deflection controlled
        from OCP.StlAPI import StlAPI_Writer
        w = StlAPI_Writer()
        w.ASCIIMode = False
        w.Write(s.wrapped, stl)
        STL_REPAIR[k] = close_mesh_cracks(stl)
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
                      repair=STL_REPAIR.get(k),
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
                                         "left_key_rim", "wall_thickness", "front_posts", "webs")}, indent=1,
                     default=str))
    print("sharp edges left:", {k: len(v) for k, v in R["sharp_edges_left"].items()}, R["edge_rule"])
    print(f"done in {time.time() - t0:.1f} s")


if __name__ == "__main__":
    main()
