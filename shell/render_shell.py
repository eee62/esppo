#!/usr/bin/env python3
"""
Render the mouse shell to PNG (shell/renders/).

Run after shell_model.py:   python3 render_shell.py
Needs: cadquery, vtk with an offscreen backend (on headless Linux: apt-get install libosmesa6).

Views
  iso_front_left.png    assembled shell, isometric from the front-left
  iso_rear_right.png    assembled shell, isometric from the rear-right
  top.png               top view (front = up)
  side_left.png         left side (USB-C opening side)
  side_right.png        right side
  section_keys.png      cross-section through both keys: vertical plane through the SW_L1 and SW_R1 centres,
                        rear half kept, with the board, switches, battery and plungers in place
  compare_*.png         every outside view with the same view of the reference STL (reference/Mini Computer Mouse_2.stl,
                        scaled and placed as the shell model uses it) beside it, same camera and scale
One neutral colour, white background. Shapes are meshed at 0.01 mm absolute deflection and shaded with normals
taken from the exact B-rep surface at every mesh node (no normal averaging, so no shading streaks).
"""
import math
import os
import sys

import cadquery as cq
import vtk
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import shell_model as m  # noqa: E402

OUT = os.path.join(HERE, "renders")
W, H = 1600, 1200
SS = 2          # supersampling factor

NEUTRAL = (0.72, 0.72, 0.70)
COLORS = {k: NEUTRAL for k in ("top", "base", "wheel", "keys", "board", "switches", "battery")}
REF_PHOTO = os.path.join(HERE, "reference", "mouse_reference.jpg")


def load_step(name):
    return m.solid(cq.importers.importStep(os.path.join(HERE, f"{name}.step")).val())


def to_polydata(shape, lin=None, ang=0.1):
    """Triangles from an absolute-deflection mesh; point normals from the exact surface at each node's (u, v).
    0.01 mm for the parametric skin; 0.05 mm for the reference skin (one dense B-spline face: 0.01 mm runs out of
    memory, and 0.05 mm is below a pixel at these image sizes)."""
    if lin is None:
        lin = 0.05 if m.REF is not None else 0.01
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRepTools import BRepTools
    from OCP.BRepGProp import BRepGProp_Face
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopAbs import TopAbs_REVERSED
    from OCP.gp import gp_Pnt, gp_Vec
    BRepTools.Clean_s(shape.wrapped)
    m._imesh(shape, lin, ang)          # surface-deflection controlled (see shell_model._imesh)
    pts, nrm, tris = vtk.vtkPoints(), vtk.vtkFloatArray(), vtk.vtkCellArray()
    nrm.SetNumberOfComponents(3)
    off = 0
    for f in shape.Faces():
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(f.wrapped, loc)
        if tri is None:
            continue
        tr = loc.Transformation()
        rev = f.wrapped.Orientation() == TopAbs_REVERSED
        gp = BRepGProp_Face(f.wrapped)
        for i in range(1, tri.NbNodes() + 1):
            P = tri.Node(i).Transformed(tr)
            pts.InsertNextPoint(P.X(), P.Y(), P.Z())
            N = gp_Vec(0, 0, 1)
            if tri.HasUVNodes():
                uv = tri.UVNode(i)
                X = gp_Pnt()
                gp.Normal(uv.X(), uv.Y(), X, N)    # global frame, oriented by the face orientation
                if N.Magnitude() > 1e-12:
                    N.Normalize()
            nrm.InsertNextTuple3(N.X(), N.Y(), N.Z())
        for i in range(1, tri.NbTriangles() + 1):
            a_, b_, c_ = tri.Triangle(i).Get()
            ids = (a_, c_, b_) if rev else (a_, b_, c_)
            t = vtk.vtkTriangle()
            for k, idx in enumerate(ids):
                t.GetPointIds().SetId(k, off + idx - 1)
            tris.InsertNextCell(t)
        off += tri.NbNodes()
    pd = vtk.vtkPolyData(); pd.SetPoints(pts); pd.SetPolys(tris); pd.GetPointData().SetNormals(nrm)
    return pd


def mesh_actor(tm, color=NEUTRAL, specular=0.25):
    """Actor for a trimesh (the reference STL): smooth normals with a 30 deg feature angle."""
    pts, tris = vtk.vtkPoints(), vtk.vtkCellArray()
    for q in tm.vertices:
        pts.InsertNextPoint(*map(float, q))
    for f in tm.faces:
        t = vtk.vtkTriangle()
        for k in range(3):
            t.GetPointIds().SetId(k, int(f[k]))
        tris.InsertNextCell(t)
    pd = vtk.vtkPolyData(); pd.SetPoints(pts); pd.SetPolys(tris)
    nm = vtk.vtkPolyDataNormals(); nm.SetInputData(pd); nm.SetFeatureAngle(30); nm.Update()
    mp = vtk.vtkPolyDataMapper(); mp.SetInputConnection(nm.GetOutputPort())
    a = vtk.vtkActor(); a.SetMapper(mp)
    pr = a.GetProperty(); pr.SetColor(*color); pr.SetSpecular(specular); pr.SetSpecularPower(30)
    pr.SetAmbient(0.18); pr.SetDiffuse(0.82); pr.SetInterpolationToPhong()
    return a


def compare_ref(view_png, ref_png, out_png):
    """The shell render and the same view of the reference STL, side by side."""
    from PIL import ImageDraw
    a = Image.open(os.path.join(OUT, view_png)).convert("RGB")
    b = Image.open(os.path.join(OUT, ref_png)).convert("RGB")
    canvas = Image.new("RGB", (a.width + b.width, a.height), "white")
    canvas.paste(a, (0, 0)); canvas.paste(b, (a.width, 0))
    ImageDraw.Draw(canvas).text((a.width + 24, a.height - 40), "Reference STL, same scale and camera", fill=(60, 60, 60))
    canvas.save(os.path.join(OUT, out_png), optimize=True)
    print("wrote", os.path.join(OUT, out_png))


def actor(shape, color, specular=0.25):
    mp = vtk.vtkPolyDataMapper(); mp.SetInputData(to_polydata(shape))
    a = vtk.vtkActor(); a.SetMapper(mp)
    pr = a.GetProperty(); pr.SetColor(*color); pr.SetSpecular(specular); pr.SetSpecularPower(30)
    pr.SetAmbient(0.18); pr.SetDiffuse(0.82); pr.SetInterpolationToPhong()
    return a


def compare(view_png, out_png, label="Reference photo"):
    """The render with the reference photo beside it, both on white, photo scaled to the render height / 2."""
    r = Image.open(os.path.join(OUT, view_png)).convert("RGB")
    ph = Image.open(REF_PHOTO).convert("RGB")
    sc = (r.height * 0.6) / ph.height
    ph = ph.resize((int(ph.width * sc), int(ph.height * sc)), Image.LANCZOS)
    pad = 40
    canvas = Image.new("RGB", (r.width + ph.width + 2 * pad, r.height), "white")
    canvas.paste(r, (0, 0))
    canvas.paste(ph, (r.width + pad, (r.height - ph.height) // 2))
    from PIL import ImageDraw
    ImageDraw.Draw(canvas).text((r.width + pad, (r.height - ph.height) // 2 - 30), label, fill=(60, 60, 60))
    canvas.save(os.path.join(OUT, out_png), optimize=True)
    print("wrote", os.path.join(OUT, out_png))


def render(actors, fname, direction, view_up=(0, 0, 1), parallel=True, zoom=1.0, title=None, cam_state=None):
    r = vtk.vtkRenderer()
    r.SetBackground(1.0, 1.0, 1.0)
    for a in actors:
        r.AddActor(a)
    # three-point lighting
    r.RemoveAllLights()
    for pos, inten in (((-1, 1.5, 2), 0.75), ((1.5, -1, 1), 0.35), ((0, 0, -1), 0.15)):
        lt = vtk.vtkLight(); lt.SetLightTypeToSceneLight(); lt.SetPosition(*[p * 500 for p in pos])
        lt.SetFocalPoint(0, 0, 15); lt.SetIntensity(inten); r.AddLight(lt)
    if title:
        t = vtk.vtkTextActor(); t.SetInput(title); tp = t.GetTextProperty()
        tp.SetFontSize(28 * SS); tp.SetColor(0.2, 0.2, 0.2); t.SetPosition(24 * SS, (H - 50) * SS); r.AddViewProp(t)
    cam = r.GetActiveCamera()
    if cam_state:                       # same camera as an earlier render (side-by-side comparisons)
        cam.SetParallelProjection(parallel)
        cam.SetFocalPoint(*cam_state[0]); cam.SetPosition(*cam_state[1]); cam.SetViewUp(*cam_state[2])
        cam.SetParallelScale(cam_state[3]); cam.SetClippingRange(1, 5000)
    else:
        cam.SetFocalPoint(0, 0, 17)
        cam.SetPosition(*[d * 400 for d in direction])
        cam.SetViewUp(*view_up)
        cam.SetParallelProjection(parallel)
        r.ResetCamera()
        cam.Zoom(zoom)
    state = (cam.GetFocalPoint(), cam.GetPosition(), cam.GetViewUp(), cam.GetParallelScale())
    w = vtk.vtkRenderWindow(); w.SetOffScreenRendering(1); w.SetSize(W * SS, H * SS); w.AddRenderer(r)
    w.Render()
    f = vtk.vtkWindowToImageFilter(); f.SetInput(w); f.Update()
    path = os.path.join(OUT, fname)
    p = vtk.vtkPNGWriter(); p.SetFileName(path); p.SetInputConnection(f.GetOutputPort()); p.Write()
    # anti-alias by supersampling (FXAA renders black under OSMesa)
    Image.open(path).convert("RGB").resize((W, H), Image.LANCZOS).save(path, optimize=True)
    print("wrote", os.path.join(OUT, fname))
    return state


def main():
    os.makedirs(OUT, exist_ok=True)
    parts = {k: load_step(k) for k in ("top", "base", "wheel")}
    shell_actors = [actor(parts[k], COLORS[k]) for k in ("top", "base", "wheel")]

    views = [("iso_front_left", (-1.0, 1.1, 0.8), (0, 0, 1), 1.15, "Front-left"),
             ("iso_rear_right", (1.0, -1.1, 0.8), (0, 0, 1), 1.15, "Rear-right"),
             ("top", (0, 0, 1), (0, 1, 0), 0.95, "Top (front up)"),
             ("side_left", (-1, 0, 0), (0, 0, 1), 1.2, "Left side (USB-C opening)"),
             ("side_right", (1, 0, 0), (0, 0, 1), 1.2, "Right side")]
    ref_actor = None
    if m.REF is not None:
        import silhouette
        ref_actor = mesh_actor(silhouette.ref_mesh(m.REF["fit"]))
    for name, d, up, z, title in views:
        st = render(shell_actors, name + ".png", d, view_up=up, zoom=z, title=title)
        if ref_actor is not None:
            render([ref_actor], "ref_" + name + ".png", d, view_up=up, title=title + " - reference STL", cam_state=st)
            compare_ref(name + ".png", "ref_" + name + ".png", "compare_" + name + ".png")
        elif name in ("iso_front_left", "iso_rear_right", "top"):
            compare(name + ".png", "compare_" + name + ".png")

    # cross-section through the keys: vertical plane through both switch centres (SW_L1 and SW_R1 sit 1.5 mm
    # apart in Y, so the plane is turned ~2.4 deg about Z); keep the rear half and look at the cut from the front
    (xl, yl), (xr, yr) = m.SW["SW_L1"], m.SW["SW_R1"]
    ang = math.degrees(math.atan2(yr - yl, xr - xl))
    keep = (m.box(-100, 100, -100, 0, -5, 50).val()
            .rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), ang).translate(cq.Vector(xl, yl, 0)))
    keys = cq.importers.importStep(os.path.join(HERE, "keys.step")).val()
    sec = {
        "top": parts["top"], "base": parts["base"], "wheel": parts["wheel"], "keys": keys,
        "board": m.board_model(), "switches": m.switches_model(), "battery": m.battery_model(),
    }
    sec_actors = []
    for k, s in sec.items():
        c = s.intersect(keep)
        if c.Volume() > 1e-6:
            sec_actors.append(actor(c, COLORS[k], specular=0.1))
    nx, ny = -math.sin(math.radians(ang)), math.cos(math.radians(ang))     # plane normal, pointing forward
    render(sec_actors, "section_keys.png", (nx - 0.25, ny, 0.12), zoom=1.3,
           title="Section through SW_L1 and SW_R1 (keys, plungers, switches, wheel)")

if __name__ == "__main__":
    main()
