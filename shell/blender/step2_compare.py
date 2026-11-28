"""Step 2 check: silhouettes (top / side / front) of the cleaned surface vs the reference (full-resolution, placed),
and the surface deviation outside the regions changed on purpose (well fill box, bowl, skirt below z 9)."""
import sys
sys.path.insert(0, "/home/etanpong/mouse-shell/shell/blender")
import importlib
import json
import numpy as np
import lib
importlib.reload(lib)
o, r = lib.obj("outer_clean"), lib.obj("ref_overlay")
rep = json.load(open(lib.OUT + "/step2_report.json"))
sil = [lib.silhouette_compare(o, r, v, f"{lib.OUT}/step2_sil_{v}.png") for v in ("top", "side", "front")]
bx, by = rep["well_fill"]["box_x"], rep["well_fill"]["box_y"]
def changed(V):
    well = (V[:, 0] > bx[0] - 4) & (V[:, 0] < bx[1] + 4) & (V[:, 1] > by[0] - 4) & (V[:, 1] < by[1] + 4) & (V[:, 2] > 20)
    seam = (np.abs(V[:, 0] - rep["well_fill"]["seam_x"]) < 6) & (V[:, 2] > 20)
    bowl = (np.abs(V[:, 0]) < 22) & (np.abs(V[:, 1] - 20.25) < 30) & (V[:, 2] > 30)
    nose = (np.abs(V[:, 0] - rep["well_fill"]["seam_x"]) < 6) & (V[:, 1] > 60)
    return well | seam | bowl | nose | (V[:, 2] < 9.5)
dev_all = lib.surface_deviation(o, r)
dev_kept = lib.surface_deviation(o, r, exclude=changed)
out = dict(silhouettes=sil, deviation_all=dev_all, deviation_outside_changed_regions=dev_kept)
rep["compare"] = out
txt = json.dumps(rep, indent=1, default=float)
open(lib.OUT + "/step2_report.json", "w").write(txt)
print(json.dumps(out, indent=1))
