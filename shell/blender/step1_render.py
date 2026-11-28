"""Step 1c: internals as boxes, renders of the placed reference with the internals (x-ray)."""
import sys
sys.path.insert(0, "/home/etanpong/mouse-shell/shell/blender")
import importlib
import lib
importlib.reload(lib)
I = lib.internals()
objs = []
for nm, b, col in (("int_battery", I["battery"]["box"], (0.2, 0.5, 0.9)), ("int_board", I["board"]["box"], (0.1, 0.6, 0.2)),
                   ("int_J3", I["tallest"]["box"], (0.9, 0.2, 0.2))):
    o = lib.box_obj(nm, b, "internals"); o.color = (*col, 1); objs.append(o)
for c in I["components"]:
    if c["ref"] != I["tallest"]["ref"] and c["box"][5] - c["box"][4] > 1.5:
        o = lib.box_obj("int_" + c["ref"], c["box"], "internals"); o.color = (0.9, 0.6, 0.1, 1); objs.append(o)
for k, sw in I["switches"].items():
    x, y = sw["xy"]
    o = lib.box_obj("int_" + k, [x - 3, x + 3, y - 3, y + 3, I["board"]["box"][5], sw["top_z"]], "internals"); o.color = (0.8, 0.1, 0.8, 1); objs.append(o)
w = I["wheel"]
o = lib.cyl_obj("int_wheel", w["centre"][1], w["centre"][2], -w["w"] / 2, w["w"] / 2, w["d"], 64, "internals", axis="X")
o.color = (0.1, 0.1, 0.1, 1); objs.append(o)
ref = lib.obj("ref_placed"); ref.color = (0.75, 0.75, 0.8, 1)
for v in ("top", "side", "front"):
    lib.render(f"{lib.OUT}/step1_{v}.png", v, [ref] + objs, size=1000, xray=True)
print("ok")
