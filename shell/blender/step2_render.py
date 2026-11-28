"""Step 2 renders: the cleaned outer surface alone, and with the reference (x-ray) for comparison."""
import sys
sys.path.insert(0, "/home/etanpong/mouse-shell/shell/blender")
import importlib
import lib
importlib.reload(lib)
o = lib.obj("outer_clean"); o.color = (0.72, 0.72, 0.76, 1)
ref = lib.obj("ref_overlay"); ref.color = (0.85, 0.3, 0.25, 1)
w = lib.obj("int_wheel"); w.color = (0.1, 0.1, 0.1, 1)
for v in ("top", "side", "side_r", "front", "rear", "iso", "iso_r", "bottom"):
    lib.render(f"{lib.OUT}/step2_{v}.png", v, [o, w], size=1000)
for v in ("top", "side", "front"):
    lib.render(f"{lib.OUT}/step2_{v}_vs_ref.png", v, [o, ref], size=1000, xray=True)
print("ok")
