#!/usr/bin/env python3
"""Fixed internals for the Blender shell build, written to internals.json.

Positions come from shell_model.py (option-B layout) and the KiCad board file. The PCB parser functions
(_tokenize, _parse, _find, _read_board) and the HEIGHTS table are taken from shell_model.py's source with `ast`,
so CadQuery is never imported. Run with any Python 3 (no third-party modules):

    python3 internals.py            -> internals.json next to this file
"""
import ast
import json
import math
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "shell_model.py")
PCB = os.path.join(HERE, "..", "..", "ESP32-S31-Vision-Pro-RevC.kicad_pcb")

tree = ast.parse(open(SRC).read())
ns = {"math": math, "re": re, "os": os}
want_fn = {"_tokenize", "_parse", "_find", "_read_board"}
for node in tree.body:
    if isinstance(node, ast.FunctionDef) and node.name in want_fn:
        exec(compile(ast.Module([node], []), SRC, "exec"), ns)
    if isinstance(node, ast.Assign) and any(getattr(t, "id", "") == "HEIGHTS" for t in node.targets):
        exec(compile(ast.Module([node], []), SRC, "exec"), ns)
    if isinstance(node, ast.ClassDef) and node.name == "P":          # parameter defaults (plain literals)
        P = {}
        for st in node.body:
            if isinstance(st, ast.AnnAssign) and st.value is not None:
                try:
                    P[st.target.id] = ast.literal_eval(st.value)
                except ValueError:
                    pass

outline, fps = ns["_read_board"](open(PCB).read())
BX0, BY0 = outline[0], outline[1]
board_z_top = P["BOARD_Z_BOT"] + P["BOARD_T"]


def b2s(bx, by):
    return (bx - P["BOARD_W"] / 2 + P["BOARD_SHIFT_X"], P["BOARD_FRONT_Y"] - by)


def page2s(px, py):
    return b2s(px - BX0, py - BY0)


def comp_height(fp):
    for rx, h, src in ns["HEIGHTS"]:
        if re.search(rx, fp):
            return h
    return 1.0


SW = {k: page2s(fps[k]["x"], fps[k]["y"]) for k in ("SW_L1", "SW_R1")}
HOLES = {k: page2s(fps[k]["x"], fps[k]["y"]) for k in ("H1", "H2", "H3", "H4")}
J5_MOUTH_X = page2s(fps["J5"]["fab"][0], 0)[0]
J5_Y = page2s(0, (fps["J5"]["fab"][1] + fps["J5"]["fab"][3]) / 2)[1]
J5_Z = board_z_top + 1.68

comps = []
for ref, q in fps.items():
    if ref.startswith(("H", "SW_L", "SW_R", "TP")) or q["crtyd"] is None:
        continue
    h = comp_height(q["fp"])
    cx0, cy0, cx1, cy1 = q["fab"] or q["crtyd"]
    a, b = page2s(cx0, cy0), page2s(cx1, cy1)
    comps.append(dict(ref=ref, fp=q["fp"], box=[min(a[0], b[0]), max(a[0], b[0]), min(a[1], b[1]), max(a[1], b[1]),
                                                 board_z_top, board_z_top + h]))
tallest = max(comps, key=lambda c: c["box"][5])

# board: the brief gives 60.05 x 58.05 (outline + Edge.Cuts line width); centred on the 60 x 58 outline
x0, y1 = b2s(0, 0)
bw, bh = 60.05, 58.05
cxb, cyb = x0 + P["BOARD_W"] / 2, y1 - P["BOARD_H"] / 2
bat_z0 = P["FLOOR"] + P["BAT_RIB_H"]
wheel_y = (SW["SW_L1"][1] + SW["SW_R1"][1]) / 2
SW_TOP_Z = board_z_top + P["SW_H"]
KEY_SOCKET_Z0 = {"SW_L1": 28.524580899159268, "SW_R1": 29.50243722582077}   # locked in shell_model.py (ref mode)

out = dict(
    frame="shell: X right, Y forward (+Y = nose), Z up, z = 0 shell bottom; mm",
    board=dict(box=[cxb - bw / 2, cxb + bw / 2, cyb - bh / 2, cyb + bh / 2, P["BOARD_Z_BOT"], board_z_top],
               outline_pcb=[round(outline[2] - outline[0], 3), round(outline[3] - outline[1], 3)], corner_r=P["BOARD_R"]),
    battery=dict(box=[P["BAT_X"] - P["BAT_W"] / 2, P["BAT_X"] + P["BAT_W"] / 2, P["BAT_Y"] - P["BAT_L"] / 2,
                      P["BAT_Y"] + P["BAT_L"] / 2, bat_z0, bat_z0 + P["BAT_T"]], corner_r=1.0),
    components=comps,
    tallest=dict(ref=tallest["ref"], fp=tallest["fp"], box=tallest["box"]),
    switches={k: dict(xy=v, body=6.0, body_h=P["SW_BODY_H"], top_z=SW_TOP_Z, act_d=P["SW_ACT_D"]) for k, v in SW.items()},
    holes=HOLES,
    rear_cover_screws=dict(L=(-P["REAR_SCREW_X"], P["REAR_SCREW_Y"]), R=(P["REAR_SCREW_X"], P["REAR_SCREW_Y"])),
    rear_iface_z=P["REAR_IFACE_Z"],
    screws=dict(
        front=dict(holes=["H1", "H2"], spec="M2.5 x 20 flat head, from below", length=20.0, head_d=4.7,
                   tip_z=20.0, thread_from_z=board_z_top, pilot_top_z=board_z_top + P["PILOT_DEPTH_FRONT"]),
        rear_cover=dict(spec="M2.5 x 20 flat head, from below", length=20.0, head_d=4.7, tip_z=20.0,
                        thread_from_z=P["REAR_IFACE_Z"], pilot_top_z=P["REAR_IFACE_Z"] + P["PILOT_DEPTH_REAR"]),
        board=dict(holes=["H3", "H4"], spec="M2.5 x 6 pan head, from below the board", length=6.0,
                   head_d=P["SCREW_HEAD_D"], head_k=P["SCREW_HEAD_K"], tip_z=P["BOARD_Z_BOT"] + 6.0,
                   thread_from_z=board_z_top, pilot_top_z=board_z_top + P["PILOT_DEPTH"]),
        clear_d=P["SCREW_CLEAR_D"], pilot_d=P["SCREW_PILOT_D"], csk_d=P["CSK_D"],
        boss_d=P["BOSS_D"], boss_d_rear=P["BOSS_D_REAR"], standoff_d=P["STANDOFF_D"]),
    wheel=dict(centre=[0.0, wheel_y, P["WHEEL_Z"]], d=P["WHEEL_D"], w=P["WHEEL_W"], axle_d=P["AXLE_D"],
               stub=P["AXLE_STUB"], open_clr=P["WHEEL_OPEN_CLR"], open_end=P["WHEEL_OPEN_END"],
               bracket_t=P["BRACKET_T"], bracket_len=P["BRACKET_LEN"], seat_d=P["AXLE_SEAT_D"]),
    usb=dict(mouth_x=J5_MOUTH_X, y=J5_Y, z=J5_Z, opening=[P["PLUG_W"] + 2 * P["PLUG_OPEN_CLR"], P["PLUG_H"] + 2 * P["PLUG_OPEN_CLR"]],
             wall="left (-X)"),
    keys=dict(hinge_y=P["HINGE_Y"], hinge_t=P["HINGE_T"], hinge_len=P["HINGE_LEN"], btn_x=P["BTN_X"],
              spine_hw=P["SPINE_HW"], slot=P["SLOT"], lip=P["LIP"], tongue_clr=P["TONGUE_CLR"],
              socket_d=P["KEY_SOCKET_D"], socket_h=P["KEY_SOCKET_H"], hole_d=P["KEY_HOLE_D"],
              hole_depth=P["KEY_HOLE_DEPTH"], socket_z0=KEY_SOCKET_Z0,
              plunger=dict(d=P["PLUNGER_D"], tip_d=P["PLUNGER_TIP_D"], peg_d=P["PLUNGER_PEG_D"],
                           peg_l=P["PLUNGER_PEG_L"], tip_z=SW_TOP_Z + P["KEY_GAP"])),
    wall=P["WALL"], floor=P["FLOOR"],
)
with open(os.path.join(HERE, "internals.json"), "w") as f:
    json.dump(out, f, indent=1)
print(json.dumps({k: out[k] for k in ("board", "battery", "tallest", "switches", "holes", "rear_cover_screws", "wheel", "usb")}, indent=1))
print(len(comps), "components")
