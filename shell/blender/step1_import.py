"""Step 1a: import the reference STL, repair, decimate to ~100k faces. Result: object `ref_dec` in reference-local
units (bounding-box minimum at the origin, nose at +x), and `ref_raw` (repaired, full resolution) kept hidden for
the silhouette overlays. Run in Blender via bl.py."""
import sys
sys.path.insert(0, "/home/etanpong/mouse-shell/shell/blender")
import importlib
import json

import bmesh
import bpy
import numpy as np

import lib
importlib.reload(lib)

TARGET_FACES = 100_000

lib.clear_all()
raw = lib.import_stl(lib.REF_STL, "ref_raw", "ref")
v = lib.verts(raw)
lo = v.min(0)
raw.data.transform(__import__("mathutils").Matrix.Translation(-__import__("mathutils").Vector(lo)))
report = {"input": lib.topo(raw)}

# repair: weld duplicate vertices, drop loose / degenerate geometry, fill any holes, consistent outward normals
bm = bmesh.new()
bm.from_mesh(raw.data)
n0 = len(bm.verts)
bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
welded = n0 - len(bm.verts)
bmesh.ops.dissolve_degenerate(bm, edges=bm.edges, dist=1e-6)
loose = [v_ for v_ in bm.verts if not v_.link_faces]
bmesh.ops.delete(bm, geom=loose, context="VERTS")
holes = [e for e in bm.edges if e.is_boundary]
if holes:
    bmesh.ops.holes_fill(bm, edges=holes, sides=0)
bmesh.ops.triangulate(bm, faces=bm.faces)
bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
bm.to_mesh(raw.data)
bm.free()
raw.data.update()
report["repair"] = dict(welded_verts=welded, loose_verts=len(loose), boundary_edges_filled=len(holes))
report["repaired"] = lib.topo(raw)

dec = raw.copy()
dec.data = raw.data.copy()
dec.name = dec.data.name = "ref_dec"
lib.collection("ref").objects.link(dec)
m = dec.modifiers.new("dec", "DECIMATE")
m.decimate_type = "COLLAPSE"
m.ratio = TARGET_FACES / len(raw.data.polygons)
m.use_collapse_triangulate = True
m.use_symmetry = False
lib.apply_mods(dec)
bm = bmesh.new()
bm.from_mesh(dec.data)
bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
bm.to_mesh(dec.data)
bm.free()
report["decimated"] = lib.topo(dec)
vd = lib.verts(dec)
report["extents_ref_units"] = (vd.max(0) - vd.min(0)).round(4).tolist()
raw.hide_set(True)
raw.hide_render = True
print(json.dumps(report, indent=1))
