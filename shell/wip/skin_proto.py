import math, time
import numpy as np
import cadquery as cq
from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex, BRepBuilderAPI_MakeFace
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Pnt, gp_Vec

SIDE = 35.03   # 35 + spline overshoot margin: board edge to inner wall stays >= 1.0


def _plan_half():
    """Right half of the outline, nose -> rear. Nose: ellipse (35 x 20) about y = 50; side: 35 wide over the
    board / USB zone, widening with a cosine ramp to 38 at y = -25; rear: ellipse (38 x 35) about y = -25."""
    pts = []
    for a in range(90, 0, -10):                              # nose, 90 deg = tip
        t = math.radians(a)
        pts.append((SIDE * math.cos(t), 50 + 20 * math.sin(t)))
    for y in range(50, -25, -4):
        g = 0.0 if y >= 22 else 0.5 * (1 - math.cos(math.pi * (22 - y) / 47))
        pts.append((SIDE + (38 - SIDE) * g, float(y)))
    for a in range(0, 91, 10):
        t = math.radians(a)
        pts.append((38 * math.cos(t), -25 - 35 * math.sin(t)))
    pts[0] = (0.0, 70.0); pts[-1] = (0.0, -60.0)
    return pts


PLAN_HALF = _plan_half()
Z_SH, Z_PEAK, N_DOME, PEAK_Y = 14.0, 34.7, 3.0, -8.0
NU, NPHI = 144, 30

_LOOP = [(-x, y) for x, y in PLAN_HALF] + [(x, y) for x, y in reversed(PLAN_HALF)][1:-1]
E = cq.Edge.makeSpline([cq.Vector(x, y, 0) for x, y in _LOOP], periodic=True)   # symmetric: tip and rear tangents are horizontal


def plan_pt(u):
    """Right half, u = 0 at the nose, u = 1 at the rear."""
    q = E.positionAt(1 - 0.5 * min(max(u, 0.0), 1.0))
    return np.array([q.x, q.y])


def surf(u, phi):
    """Outer skin point: plan outline scaled by s(phi) about (0, PEAK_Y), height by the superellipse."""
    c, s_ = math.cos(phi), math.sin(phi)
    s = abs(c) ** (2 / N_DOME)
    h = abs(s_) ** (2 / N_DOME)
    q = plan_pt(u)
    C = np.array([0.0, PEAK_Y])
    xy = C + s * (q - C)
    return np.array([xy[0], xy[1], Z_SH + (Z_PEAK - Z_SH) * h])


def normal(u, phi):
    du, dp = 1e-4, 1e-4
    Pu = (surf(min(u + du, 1), phi) - surf(max(u - du, 0), phi))
    if phi < dp:
        Pp = surf(u, 2 * dp) - surf(u, 0.0)
    else:
        Pp = surf(u, phi + dp) - surf(u, phi - dp)
    n = np.cross(Pu, Pp)
    n /= np.linalg.norm(n)
    P = surf(u, phi)
    if (P[0]) * n[0] + (P[1] - PEAK_Y) * n[1] + 1e-9 * n[2] < 0 and phi < math.pi / 2 - 1e-3:
        n = -n
    return n


def section_pts(phi, d):
    out = []
    for i in range(NU + 1):
        u = i / NU
        P = surf(u, phi)
        if d:
            n = normal(u, phi)
            if phi == 0.0:
                n[2] = 0.0; n /= np.linalg.norm(n)
            P = P - d * n
        out.append(P)
    out[0][0] = 0.0; out[-1][0] = 0.0
    return out


def wire_from(pts):
    """Closed CCW section wire (left half nose->rear, then right half rear->nose)."""
    l = [cq.Vector(-p[0], p[1], p[2]) for p in pts]
    r = [cq.Vector(*p) for p in reversed(pts)][1:-1]
    return cq.Wire.assembleEdges([cq.Edge.makeSpline(l + r, periodic=True)])


def skin_body(d=0.0, zb=0.0):
    phis = [0.5 * math.pi * j / NPHI for j in range(NPHI)]
    wires = [wire_from(section_pts(ph, d)) for ph in phis]
    ts = BRepOffsetAPI_ThruSections(True, False, 1e-6)
    for w in wires:
        ts.AddWire(w.wrapped)
    ts.AddVertex(BRepBuilderAPI_MakeVertex(gp_Pnt(0, PEAK_Y, Z_PEAK - d)).Vertex())
    ts.CheckCompatibility(False)
    ts.Build()
    dome = _fix(ts.Shape())
    f0 = BRepBuilderAPI_MakeFace(wires[0].wrapped, True).Face()
    prism = _fix(BRepPrimAPI_MakePrism(f0, gp_Vec(0, 0, zb - Z_SH)).Shape())
    return _fix(dome.fuse(prism).clean().wrapped)


def _fix(shape):
    from OCP.ShapeFix import ShapeFix_Shape
    fx = ShapeFix_Shape(shape); fx.Perform()
    s = cq.Shape.cast(fx.Shape())
    if s.ShapeType() == "Compound" and len(s.Solids()) == 1:
        s = s.Solids()[0]
    return s


if __name__ == "__main__":
    t = time.time()
    dome, prism = skin_body(0.0, 0.0)
    print('dome', dome.ShapeType(), dome.isValid(), round(dome.Volume()), 'prism', prism.isValid(), round(time.time() - t, 1))
    b = dome.fuse(prism).clean()
    print('fused', b.ShapeType(), len(b.Solids()), b.isValid(), round(b.Volume()), [f.geomType() for f in b.Faces()])
    bb = b.BoundingBox(); print([round(v, 2) for v in (bb.xmin, bb.xmax, bb.ymin, bb.ymax, bb.zmin, bb.zmax)])
    t = time.time()
    cd, cp = skin_body(1.5, -5.0)
    c = cd.fuse(cp).clean()
    print('cavity', c.isValid(), round(time.time() - t, 1))
    import sys; sys.path.insert(0, '/Users/e/esppo/shell')
    import shell_model as m
    print('skin min dist', round(m.min_dist(b, c.cut(m.box(-60, 60, -80, 80, -10, 1.5).val())), 4))
    h = b.cut(c)
    print('hollow', h.isValid(), round(h.Volume()))
    cq.exporters.export(h, 'skin_hollow.stl', tolerance=0.05)
