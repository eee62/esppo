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
Colours: grey top over a black base and wheel, white background.
"""
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

COLORS = {
    "top": (0.74, 0.75, 0.77), "base": (0.07, 0.07, 0.08), "wheel": (0.03, 0.03, 0.03),
    "keys": (0.95, 0.50, 0.12), "board": (0.10, 0.50, 0.25), "switches": (0.15, 0.15, 0.15),
    "battery": (0.85, 0.72, 0.25),
}


def load_step(name):
    return m.solid(cq.importers.importStep(os.path.join(HERE, f"{name}.step")).val())


def to_polydata(shape, tol=0.02, ang=0.15):
    vs, fs = shape.tessellate(tol, ang)
    pts = vtk.vtkPoints()
    for v in vs:
        pts.InsertNextPoint(v.x, v.y, v.z)
    tris = vtk.vtkCellArray()
    for a, b, c in fs:
        t = vtk.vtkTriangle()
        t.GetPointIds().SetId(0, a); t.GetPointIds().SetId(1, b); t.GetPointIds().SetId(2, c)
        tris.InsertNextCell(t)
    pd = vtk.vtkPolyData(); pd.SetPoints(pts); pd.SetPolys(tris)
    nrm = vtk.vtkPolyDataNormals(); nrm.SetInputData(pd); nrm.SetFeatureAngle(35); nrm.SplittingOn(); nrm.Update()
    return nrm.GetOutput()


def actor(shape, color, specular=0.25):
    mp = vtk.vtkPolyDataMapper(); mp.SetInputData(to_polydata(shape))
    a = vtk.vtkActor(); a.SetMapper(mp)
    pr = a.GetProperty(); pr.SetColor(*color); pr.SetSpecular(specular); pr.SetSpecularPower(30)
    pr.SetAmbient(0.15); pr.SetDiffuse(0.85)
    return a


def render(actors, fname, direction, view_up=(0, 0, 1), parallel=True, zoom=1.0, title=None):
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
    cam.SetFocalPoint(0, 0, 17)
    cam.SetPosition(*[d * 400 for d in direction])
    cam.SetViewUp(*view_up)
    cam.SetParallelProjection(parallel)
    r.ResetCamera()
    cam.Zoom(zoom)
    w = vtk.vtkRenderWindow(); w.SetOffScreenRendering(1); w.SetSize(W * SS, H * SS); w.AddRenderer(r)
    w.Render()
    f = vtk.vtkWindowToImageFilter(); f.SetInput(w); f.Update()
    path = os.path.join(OUT, fname)
    p = vtk.vtkPNGWriter(); p.SetFileName(path); p.SetInputConnection(f.GetOutputPort()); p.Write()
    # anti-alias by supersampling (FXAA renders black under OSMesa)
    Image.open(path).convert("RGB").resize((W, H), Image.LANCZOS).save(path, optimize=True)
    print("wrote", os.path.join(OUT, fname))


def main():
    os.makedirs(OUT, exist_ok=True)
    parts = {k: load_step(k) for k in ("top", "base", "wheel")}
    shell_actors = [actor(parts[k], COLORS[k]) for k in ("top", "base", "wheel")]

    render(shell_actors, "iso_front_left.png", (-1.0, 1.1, 0.8), zoom=1.15, title="Front-left")
    render(shell_actors, "iso_rear_right.png", (1.0, -1.1, 0.8), zoom=1.15, title="Rear-right")
    render(shell_actors, "top.png", (0, 0, 1), view_up=(0, 1, 0), zoom=1.2, title="Top (front up)")
    render(shell_actors, "side_left.png", (-1, 0, 0), zoom=1.2, title="Left side (USB-C opening)")
    render(shell_actors, "side_right.png", (1, 0, 0), zoom=1.2, title="Right side")

if __name__ == "__main__":
    main()
