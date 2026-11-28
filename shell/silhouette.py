"""Silhouette check: the shell (top.stl + base.stl) against the reference STL, scaled and placed exactly as the shell
model uses it, projected from the top, the side and the front. Overlap = |A and B| / |A or B| (intersection over
union) on a 0.1 mm raster; overlays in renders/silhouette_*.png, numbers in silhouette.json."""
import json
import os
import sys

import numpy as np
import trimesh
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
PX = 0.1


def ref_mesh(fit):
    """Reference STL in shell coordinates: reference +x (nose) -> shell +y, reference +y -> shell -x, z up, scaled."""
    import ref_skin
    m = trimesh.load(ref_skin.REF_STL)
    m.apply_translation(-m.bounds[0])
    v = m.vertices
    s, cx, cy = fit["s"], fit["cx"], fit["cy"]
    w = np.column_stack([(cy - v[:, 1]) * s, (v[:, 0] - cx) * s, v[:, 2] * s])
    return trimesh.Trimesh(w, m.faces, process=False)


def raster(mesh, axes, lo, shape):
    img = Image.new("1", shape, 0)
    d = ImageDraw.Draw(img)
    P = (mesh.vertices[:, axes] - lo) / PX
    for t in P[mesh.faces]:
        d.polygon([tuple(q) for q in t], fill=1)
    return np.array(img, bool)


def main():
    import shell_model as m
    shell = trimesh.util.concatenate([trimesh.load(os.path.join(HERE, f)) for f in ("top.stl", "base.stl")])
    ref = ref_mesh(m.REF["fit"])
    views = {"top": (0, 1), "side": (1, 2), "front": (0, 2)}
    out = {}
    for name, ax in views.items():
        both = np.vstack([shell.vertices[:, ax], ref.vertices[:, ax]])
        lo = both.min(0) - 2
        hi = both.max(0) + 2
        shape = tuple(int(np.ceil(c)) for c in (hi - lo) / PX)
        A = raster(shell, list(ax), lo, shape)
        B = raster(ref, list(ax), lo, shape)
        inter, union = (A & B).sum(), (A | B).sum()
        out[name] = dict(overlap_iou_pct=round(100 * inter / union, 2), shell_mm2=round(A.sum() * PX * PX, 1),
                         reference_mm2=round(B.sum() * PX * PX, 1),
                         shell_only_mm2=round((A & ~B).sum() * PX * PX, 1), reference_only_mm2=round((B & ~A).sum() * PX * PX, 1))
        rgb = np.full(A.shape + (3,), 255, np.uint8)
        rgb[A & B] = (150, 150, 150)
        rgb[A & ~B] = (30, 110, 220)          # shell only: blue
        rgb[B & ~A] = (220, 60, 40)           # reference only: red
        im = Image.fromarray(np.flipud(rgb.transpose(1, 0, 2)))
        ImageDraw.Draw(im).text((10, 10), f"{name}: overlap {out[name]['overlap_iou_pct']} %  (grey both, blue shell only, "
                                          f"red reference only)", fill=(0, 0, 0))
        im.save(os.path.join(HERE, "renders", f"silhouette_{name}.png"), optimize=True)
    with open(os.path.join(HERE, "silhouette.json"), "w") as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
