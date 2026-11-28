"""Shared helpers for the Blender shell build (run inside Blender via bl.py; scripts do
`import sys; sys.path.insert(0, BLENDER_DIR); import lib`)."""
import json
import os

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
SHELL = os.path.dirname(HERE)
REF_STL = os.path.join(SHELL, "reference", "Mini Computer Mouse_2.stl")
OUT = os.path.join(HERE, "out")            # STLs, .blend, renders (large intermediate files are git-ignored)
os.makedirs(OUT, exist_ok=True)


def internals():
    with open(os.path.join(HERE, "internals.json")) as f:
        return json.load(f)


# ---------------------------------------------------------------------------------------------------------- scene
def obj(name):
    return bpy.data.objects.get(name)


def remove(name):
    o = bpy.data.objects.get(name)
    if o is not None:
        me = o.data if o.type == "MESH" else None
        bpy.data.objects.remove(o, do_unlink=True)
        if me is not None and me.users == 0:
            bpy.data.meshes.remove(me)


def clear_all():
    for o in list(bpy.data.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    for m in list(bpy.data.meshes):
        if m.users == 0:
            bpy.data.meshes.remove(m)


def collection(name):
    c = bpy.data.collections.get(name)
    if c is None:
        c = bpy.data.collections.new(name)
        bpy.context.scene.collection.children.link(c)
    return c


def link(o, coll="work"):
    for c in list(o.users_collection):
        c.objects.unlink(o)
    collection(coll).objects.link(o)
    return o


def mesh_obj(name, V, F, coll="work"):
    """New mesh object from numpy vertices / faces (replaces an object of that name)."""
    remove(name)
    me = bpy.data.meshes.new(name)
    V = np.asarray(V, np.float64)
    F = np.asarray(F, np.int64)
    me.vertices.add(len(V))
    me.vertices.foreach_set("co", V.astype(np.float32).ravel())
    nv = F.shape[1]
    me.loops.add(F.size)
    me.loops.foreach_set("vertex_index", F.astype(np.int32).ravel())
    me.polygons.add(len(F))
    me.polygons.foreach_set("loop_start", (np.arange(len(F)) * nv).astype(np.int32))
    me.update(calc_edges=True)
    me.validate()
    o = bpy.data.objects.new(name, me)
    collection(coll).objects.link(o)
    return o


def import_stl(path, name, coll="work"):
    remove(name)
    bpy.ops.wm.stl_import(filepath=path)
    o = bpy.context.selected_objects[0]
    o.name = name
    o.data.name = name
    return link(o, coll)


def export_stl(o, path):
    bpy.ops.object.select_all(action="DESELECT")
    o.select_set(True)
    bpy.context.view_layer.objects.active = o
    bpy.ops.wm.stl_export(filepath=path, export_selected_objects=True, apply_modifiers=True, ascii_format=False)


def verts(o, world=True):
    me = o.data
    v = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", v)
    v = v.reshape(-1, 3)
    if world and o.matrix_world != Matrix.Identity(4):
        M = np.array(o.matrix_world)
        v = v @ M[:3, :3].T + M[:3, 3]
    return v


def faces(o):
    """Triangle indices (mesh must be triangulated)."""
    me = o.data
    me.calc_loop_triangles()
    t = np.empty(len(me.loop_triangles) * 3, np.int64)
    me.loop_triangles.foreach_get("vertices", t)
    return t.reshape(-1, 3)


def apply_mods(o):
    """Apply every modifier (evaluated mesh becomes the object's mesh)."""
    dg = bpy.context.evaluated_depsgraph_get()
    ev = o.evaluated_get(dg)
    me = bpy.data.meshes.new_from_object(ev)
    old = o.data
    o.modifiers.clear()
    o.data = me
    if old.users == 0:
        bpy.data.meshes.remove(old)
    me.name = o.name
    return o


def apply_transform(o):
    o.data.transform(o.matrix_world)
    o.matrix_world = Matrix.Identity(4)


def boolean(target, cutter, op="DIFFERENCE", solver="EXACT", apply=True):
    m = target.modifiers.new("bool", "BOOLEAN")
    m.operation = op
    m.object = cutter
    m.solver = solver
    if solver == "EXACT":
        m.use_self = False
        m.use_hole_tolerant = True
    if apply:
        apply_mods(target)
    return target


def topo(o):
    """Watertightness report: non-manifold and boundary edge counts, shells, Euler characteristic."""
    bm = bmesh.new()
    bm.from_mesh(o.data)
    nm = sum(1 for e in bm.edges if not e.is_manifold)
    bd = sum(1 for e in bm.edges if e.is_boundary)
    # connected shells
    seen = set()
    shells = 0
    for f in bm.faces:
        if f.index in seen:
            continue
        shells += 1
        stack = [f]
        seen.add(f.index)
        while stack:
            g = stack.pop()
            for e in g.edges:
                for h in e.link_faces:
                    if h.index not in seen:
                        seen.add(h.index)
                        stack.append(h)
    vol = bm.calc_volume(signed=True)
    r = dict(faces=len(bm.faces), verts=len(bm.verts), nonmanifold_edges=nm, boundary_edges=bd, shells=shells,
             euler=len(bm.verts) - len(bm.edges) + len(bm.faces), volume_mm3=round(vol, 2))
    bm.free()
    r["watertight"] = nm == 0 and bd == 0
    return r


def box_obj(name, b, coll="ref", r=None):
    """Axis box [x0,x1,y0,y1,z0,z1]."""
    x0, x1, y0, y1, z0, z1 = b
    V = np.array([[x, y, z] for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)])
    F = np.array([[0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4], [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5]])
    return mesh_obj(name, V, F, coll)


def cyl_obj(name, x, y, z0, z1, d, n=48, coll="tools", axis="Z"):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    r = d / 2
    c, s = r * np.cos(a), r * np.sin(a)
    if axis == "Z":
        V = np.vstack([np.column_stack([x + c, y + s, np.full(n, z0)]), np.column_stack([x + c, y + s, np.full(n, z1)])])
    else:   # along X from z0 to z1 (x0, x1), centre (y = x arg, z = y arg)
        V = np.vstack([np.column_stack([np.full(n, z0), x + c, y + s]), np.column_stack([np.full(n, z1), x + c, y + s])])
    F = [[i, (i + 1) % n, n + (i + 1) % n, n + i] for i in range(n)]
    F = [list(f) for f in F]
    o = mesh_poly(name, V, F + [list(range(n))[::-1], list(range(n, 2 * n))], coll)
    if axis != "Z":
        bpy.context.view_layer.update()
    # outward normals
    bm = bmesh.new(); bm.from_mesh(o.data); bmesh.ops.recalc_face_normals(bm, faces=bm.faces); bm.to_mesh(o.data); bm.free()
    return o


def mesh_poly(name, V, F, coll="work"):
    remove(name)
    me = bpy.data.meshes.new(name)
    me.from_pydata([tuple(map(float, v)) for v in V], [], [list(map(int, f)) for f in F])
    me.update()
    o = bpy.data.objects.new(name, me)
    collection(coll).objects.link(o)
    return o


# ------------------------------------------------------------------------------------------------------- renders
VIEWS = {   # camera direction (from target towards camera), up vector
    "top": ((0, 0, 1), (0, 1, 0)),
    "side": ((-1, 0, 0), (0, 0, 1)),     # from the left (-X): nose to the right of the image
    "side_r": ((1, 0, 0), (0, 0, 1)),
    "front": ((0, 1, 0), (0, 0, 1)),
    "rear": ((0, -1, 0), (0, 0, 1)),
    "bottom": ((0, 0, -1), (0, 1, 0)),
    "iso": ((-0.55, 0.75, 0.6), (0, 0, 1)),
    "iso_r": ((0.6, -0.7, 0.6), (0, 0, 1)),
}


def material(name, rgba, alpha=1.0):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.diffuse_color = (*rgba[:3], alpha)
    return m


def set_color(o, rgba, alpha=1.0):
    o.color = (*rgba[:3], alpha)


def render(path, view, objs, size=1200, ortho_pad=1.08, center=None, extent=None, xray=False, cavity=True):
    """Workbench render of `objs` only, orthographic, from a named view. Object colours via obj.color."""
    sc = bpy.context.scene
    keep = {o.name for o in objs}
    hidden = {}
    for o in sc.objects:
        hidden[o.name] = o.hide_render
        o.hide_render = o.name not in keep and o.type != "CAMERA"
    d, up = VIEWS[view]
    d = Vector(d).normalized()
    up = Vector(up)
    if center is None or extent is None:
        P = np.vstack([np.array([o.matrix_world @ Vector(c) for c in o.bound_box]) for o in objs])
        lo, hi = P.min(0), P.max(0)
        center = (lo + hi) / 2 if center is None else center
        if extent is None:
            right = up.cross(d).normalized()
            corners = P - center
            extent = 2 * max(np.abs(corners @ np.array(right)).max(), np.abs(corners @ np.array(d.cross(right).normalized())).max())
    cam = bpy.data.objects.get("shotcam")
    if cam is None:
        cam = bpy.data.objects.new("shotcam", bpy.data.cameras.new("shotcam"))
        sc.collection.objects.link(cam)
    cam.data.type = "ORTHO"
    cam.data.ortho_scale = extent * ortho_pad
    cam.data.clip_start, cam.data.clip_end = 0.1, 2000
    loc = Vector(center) + d * 500
    cam.location = loc
    z = d
    x = up.cross(z).normalized()
    y = z.cross(x)
    cam.matrix_world = Matrix((x, y, z)).transposed().to_4x4()
    cam.location = loc
    sc.camera = cam
    sc.render.engine = "BLENDER_WORKBENCH"
    sh = sc.display.shading
    sh.light = "STUDIO"
    sh.color_type = "OBJECT"
    sh.show_cavity = cavity
    sh.show_xray = xray
    sh.xray_alpha = 0.5
    sh.show_object_outline = True
    sh.show_specular_highlight = True
    sc.render.resolution_x = size
    sc.render.resolution_y = size
    sc.render.film_transparent = False
    sc.world.color = (1, 1, 1) if sc.world else None
    sc.render.image_settings.file_format = "PNG"
    sc.render.filepath = path
    bpy.ops.render.render(write_still=True)
    for n, h in hidden.items():
        if n in bpy.data.objects:
            bpy.data.objects[n].hide_render = h
    return path


def mask_render(path, view, objs, center, extent, size=800):
    """Flat silhouette (white object on black) for overlap scoring."""
    sc = bpy.context.scene
    sh = sc.display.shading
    old = (sh.light, sh.color_type, sh.single_color[:], sh.show_cavity, sh.show_object_outline, sh.show_specular_highlight)
    sh.light, sh.color_type = "FLAT", "SINGLE"
    sh.single_color = (1, 1, 1)
    sh.show_object_outline = False
    sh.show_specular_highlight = False
    w = sc.world.color[:]
    sc.world.color = (0, 0, 0)
    render(path, view, objs, size=size, ortho_pad=1.0, center=center, extent=extent, cavity=False)
    sh.light, sh.color_type, sh.single_color, sh.show_cavity, sh.show_object_outline, sh.show_specular_highlight = \
        old[0], old[1], old[2], old[3], old[4], old[5]
    sh.light, sh.color_type = old[0], old[1]
    sc.world.color = w
    return path


# ---------------------------------------------------------------------------------------------- SDF grid pipeline
# A recipe is a nested tuple, evaluated by Blender's geometry-node SDF grid nodes (OpenVDB level sets):
#   ("mesh", obj_name)               Mesh to SDF Grid of that object (world space)
#   ("union"|"diff"|"inter", a, b)   SDF Grid Boolean
#   ("offset", a, d)                 SDF Grid Offset, d > 0 grows the solid
#   ("close", a, r) / ("open", a, r) morphological closing (grow r, shrink r) / opening (shrink r, grow r)
#   ("mean", a, width, iterations)   SDF Grid Mean filter
def _sock(node, name, out=False):
    socks = node.outputs if out else node.inputs
    for s in socks:
        if s.name == name and s.enabled:
            return s
    for s in socks:
        if s.name == name:
            return s
    raise KeyError(f"{node.bl_idname}: no {'output' if out else 'input'} {name!r}: {[s.name for s in socks]}")


GROW_SIGN = 1.0     # sign of SDF Grid Offset's Distance that grows the solid (set by calibrate_offset())


def sdf_mesh(name, recipe, voxel=0.2, band=20, coll="work", adaptivity=0.0):
    """Evaluate the recipe with a geometry-node tree on a fresh object and return it as a plain mesh object."""
    remove(name)
    me = bpy.data.meshes.new(name)
    o = bpy.data.objects.new(name, me)
    collection(coll).objects.link(o)
    ng = bpy.data.node_groups.new(name + "_gn", "GeometryNodeTree")
    ng.interface.new_socket("Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
    N, L = ng.nodes, ng.links
    out = N.new("NodeGroupOutput")
    cache = {}

    def ev(r):
        k = repr(r)
        if k in cache:
            return cache[k]
        op = r[0]
        if op == "mesh":
            oi = N.new("GeometryNodeObjectInfo")
            oi.inputs["Object"].default_value = bpy.data.objects[r[1]]
            oi.transform_space = "RELATIVE"
            n = N.new("GeometryNodeMeshToSDFGrid")
            L.new(_sock(oi, "Geometry", True), _sock(n, "Mesh"))
            _sock(n, "Voxel Size").default_value = voxel
            _sock(n, "Band Width").default_value = band
        elif op in ("union", "diff", "inter"):
            n = N.new("GeometryNodeSDFGridBoolean")
            n.operation = {"union": "UNION", "diff": "DIFFERENCE", "inter": "INTERSECT"}[op]
            a, b = ev(r[1]), ev(r[2])
            if op == "diff":
                L.new(a, _sock(n, "Grid 1"))
                L.new(b, _sock(n, "Grid 2"))
            else:
                L.new(b, _sock(n, "Grid"))
                L.new(a, _sock(n, "Grid"))
        elif op == "offset":
            n = N.new("GeometryNodeSDFGridOffset")
            L.new(ev(r[1]), _sock(n, "Grid"))
            _sock(n, "Distance").default_value = GROW_SIGN * r[2]
        elif op == "close":
            s = ev(("offset", ("offset", r[1], r[2]), -r[2]))
            cache[k] = s
            return s
        elif op == "open":
            s = ev(("offset", ("offset", r[1], -r[2]), r[2]))
            cache[k] = s
            return s
        elif op == "mean":
            n = N.new("GeometryNodeSDFGridMean")
            L.new(ev(r[1]), _sock(n, "Grid"))
            _sock(n, "Width").default_value = r[2]
            _sock(n, "Iterations").default_value = r[3]
        else:
            raise ValueError(op)
        s = _sock(n, "Grid", True)
        cache[k] = s
        return s

    g = N.new("GeometryNodeGridToMesh")
    L.new(ev(recipe), _sock(g, "Grid"))
    _sock(g, "Threshold").default_value = 0.0
    _sock(g, "Adaptivity").default_value = adaptivity
    L.new(_sock(g, "Mesh", True), out.inputs[0])
    m = o.modifiers.new("sdf", "NODES")
    m.node_group = ng
    apply_mods(o)
    bpy.data.node_groups.remove(ng)
    return o


def heightfield_solid(name, xs, ys, ztop, zbot, coll="tools"):
    """Closed mesh over the rectangle xs x ys: top surface ztop[i, j], bottom zbot[i, j] (zbot < ztop everywhere),
    vertical side walls. Outward normals."""
    nx, ny = len(xs), len(ys)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    top = np.column_stack([X.ravel(), Y.ravel(), np.asarray(ztop).ravel()])
    bot = np.column_stack([X.ravel(), Y.ravel(), np.asarray(zbot).ravel()])
    V = np.vstack([top, bot])
    idx = np.arange(nx * ny).reshape(nx, ny)
    a, b, c, d = idx[:-1, :-1].ravel(), idx[1:, :-1].ravel(), idx[1:, 1:].ravel(), idx[:-1, 1:].ravel()
    off = nx * ny
    F = [np.column_stack([a, b, c, d]), np.column_stack([a + off, d + off, c + off, b + off])]
    ring = np.concatenate([idx[:, 0], idx[-1, 1:], idx[-2::-1, -1], idx[0, -2:0:-1]])   # boundary, counter-clockwise
    r2 = np.roll(ring, -1)
    F.append(np.column_stack([ring, ring + off, r2 + off, r2]))
    o = mesh_obj(name, V, np.vstack(F), coll)
    bm = bmesh.new(); bm.from_mesh(o.data); bmesh.ops.recalc_face_normals(bm, faces=bm.faces); bm.to_mesh(o.data); bm.free()
    return o


def prism_solid(name, poly, z0, z1, roll=0.0, n_roll=10, coll="tools"):
    """Closed prism over a closed plan polygon (CCW, star-shaped from its centroid), z0..z1; with roll > 0 the top
    edge rolls inward on a quarter circle of that radius (straight up to z1, top cap at z1 + roll)."""
    poly = np.asarray(poly, float)
    c = poly.mean(0)
    # vertex normals of the polygon (outward, CCW)
    t = np.roll(poly, -1, 0) - np.roll(poly, 1, 0)
    nrm = np.column_stack([t[:, 1], -t[:, 0]])
    nrm /= np.linalg.norm(nrm, axis=1)[:, None]
    rings = [np.column_stack([poly, np.full(len(poly), z0)]), np.column_stack([poly, np.full(len(poly), z1)])]
    if roll > 0:
        for a in np.linspace(0, np.pi / 2, n_roll + 1)[1:]:
            inset = roll * (1 - np.cos(a))
            rings.append(np.column_stack([poly - inset * nrm, np.full(len(poly), z1 + roll * np.sin(a))]))
    n = len(poly)
    V = np.vstack(rings + [[*c, z0], [*c, rings[-1][0, 2]]])
    F = []
    for k in range(len(rings) - 1):
        i = np.arange(n)
        F += [[k * n + j, k * n + (j + 1) % n, (k + 1) * n + (j + 1) % n, (k + 1) * n + j] for j in i]
    cb, ct = len(V) - 2, len(V) - 1
    last = (len(rings) - 1) * n
    F += [[cb, (j + 1) % n, j] for j in range(n)]
    F += [[ct, last + j, last + (j + 1) % n] for j in range(n)]
    o = mesh_poly(name, V, F, coll)
    bm = bmesh.new(); bm.from_mesh(o.data); bmesh.ops.recalc_face_normals(bm, faces=bm.faces); bm.to_mesh(o.data); bm.free()
    return o


def placement():
    with open(os.path.join(HERE, "placement.json")) as f:
        return json.load(f)


def ref_to_shell(P2, pl=None):
    pl = pl or placement()
    S, CX, CY = pl["scale"], pl["CX"], pl["CY"]
    P2 = np.atleast_2d(P2)
    return np.column_stack([-(P2[:, 1] - CY) * S, (P2[:, 0] - CX) * S])


def topmap(o, xs, ys, z0=120.0):
    """Highest surface z of object o over the grid (BVH ray cast down); NaN where the ray misses."""
    from mathutils.bvhtree import BVHTree
    bvh = BVHTree.FromObject(o, bpy.context.evaluated_depsgraph_get())
    Z = np.full((len(xs), len(ys)), np.nan)
    dn = Vector((0, 0, -1))
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            h = bvh.ray_cast(Vector((float(x), float(y), z0)), dn)
            if h[0] is not None:
                Z[i, j] = h[0].z
    return Z


def biharmonic_fill(Z, mask, iters=4000, tol=1e-7):
    """Fill Z inside mask with the biharmonic interpolant of the values outside (matrix-free conjugate gradient
    on the restricted bilaplacian; needs a ring of >= 2 known cells around the mask)."""
    def lap(U):
        P = np.pad(U, 1, mode="edge")
        return P[:-2, 1:-1] + P[2:, 1:-1] + P[1:-1, :-2] + P[1:-1, 2:] - 4 * U

    def B(u):
        U = np.zeros_like(Z)
        U[mask] = u
        return lap(lap(U))[mask]

    Z0 = np.where(mask, 0.0, Z)
    b = -lap(lap(Z0))[mask]
    # initial guess: harmonic-ish (mean of known)
    x = np.full(mask.sum(), np.nanmean(Z[~mask]))
    r = b - B(x)
    p = r.copy()
    rs = r @ r
    for k in range(iters):
        Ap = B(p)
        a = rs / (p @ Ap)
        x += a * p
        r -= a * Ap
        rn = r @ r
        if np.sqrt(rn) < tol * np.sqrt(b @ b + 1e-30):
            break
        p = r + (rn / rs) * p
        rs = rn
    out = Z.copy()
    out[mask] = x
    return out, k
