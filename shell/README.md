# Demo mouse shell — ESP32-S31 Vision Pro Rev C

A printable, non-functional demo mouse shell around the ESP32-S31 Rev C board. It has two symmetric front buttons on living hinges, and a scroll wheel with 24 detents on the centreline. Each button presses its real tact switch (SW_L1, SW_R1) through a plunger. Nothing connects to a computer.

The outer form follows the general shape of the reference photo `reference/mouse_reference.jpg`, with no logos or branding:
- a grey domed top over a black lower band;
- the wheel in a fixed strip between the buttons.

## Run

```
python3 shell_model.py     # build, all checks, STL/STEP export, checks.json
python3 render_shell.py    # 6 PNG views into renders/ (needs vtk with an off-screen backend; Linux: apt-get install libosmesa6)
```

**Needs:** `cadquery` (2.8 tested), `trimesh`, `numpy`, `manifold3d`, `embreex` (fast ray casting for the wall check).

**Input:** the board is read from `../ESP32-S31-Vision-Pro-RevC.kicad_pcb`, read only. It is byte-identical to the copy inside `../mouse-revc-src.zip`. No KiCad file is changed.

### Build speed (caches in `.cache/`, git-ignored)

- **Top part:** built in 5 stages, each cached on disk: `top_shell` (skin, slots, hinges, wheel opening) → `top_posts` → `top_wheel_mounts` → `top_holes` → `top_part` (outer-edge rounding). A stage's cache key holds its own source, the source of every earlier stage, every parameter and every upper-case module constant. A fix therefore reruns only the stage it touches and the ones after it.
- **Setup steps:** the lofted skins, cavity, hollow body, keep-out band, base and parsed board file are cached too.
- **Check results:** every exact distance, overlap, wall and web result is cached. Each is keyed on the exact B-rep of the shapes it reads and on the source of that kind of check. A rebuild that changes one part re-measures only the pairs involving it.
- **Skin sections are not simplified:** 72 points per half-section and 20 dome sections.
- **Features are fused one at a time**, each with its own rounding.

| Run | Time (4-core container) |
|---|---|
| Cold build of all parts | ~8 min (top stages: shell 2 min, posts 3.5 min, wheel mounts 2 min, holes 1 min, rounding < 1 min) |
| Cold checks (692 exact queries) | ~60 min |
| Everything cached | build 3–5 s; one full run with STL/STEP export ~13 min, most of it the uncached front-post check and the export |

`SHELL_NO_CACHE=1` disables all caches.

## Bottom line

| Check | Result |
|---|---|
| No overlaps: board, battery (6 × 60 × 90), switches, wheel, keys/plungers, base, top, screws | **Pass.** 0.000 mm³ for all 36 pairs. No unexpected contacts. |
| Plunger centred on its switch (≤ 0.5 mm), under its own button | **Pass.** Offset 0.00 mm for both. Each socket lies inside its own button. |
| Highest point ≤ 38 mm, wheel included | **Pass.** Wheel top 37.867 mm; skin 34.7 mm. |
| Wall ≥ 1.5 mm except at the hinges | **Pass.** Exact skin-to-skin minimum **1.500 mm**. No kept sample is below 1.48 mm (smallest 1.487). Hinges 1.002 mm. See [Wall check](#wall-check). |
| Webs between openings, and from each opening to the outer edge, ≥ JLC3DP minimum (1.2 mm) | **Pass.** Minimum **1.500 mm**, wheel opening to the slot in front of it. See [Web check](#web-check). |
| USB-C opening lined up with J5, left wall | **Pass.** Centred on J5 (y 30.00, z 16.78) in the left (−X) wall, 13.15 × 7.3 mm. |
| Front screw posts H1/H2 clear of the buttons | **Pass.** 2.000 / 2.147 mm vertical gap under the button (≥ 2.0). Each post and its rib are one piece, tied to the fixed side wall. |
| Edge rule: outer surface and USB-C opening edges ≥ 0.5 mm round | **Pass.** 0 required edges left sharp. 56 sharp edges inside slots, hinges and under the buttons are accepted ([list](#accepted-sharp-edges)). |
| STL files | All watertight. `base`, `top`, `wheel`, `key_L` and `key_R` are single bodies; `keys` holds 2 plungers. |

## Files

| File | Contents |
|---|---|
| `shell_model.py` | Parametric source. Dimensions are in the `P` dataclass at the top. |
| `render_shell.py` | Renderer for the 6 views |
| `base.stl` / `.step` | Bottom cover: floor, vertical band z 0–14, battery ribs / stops / clips, cover-screw columns |
| `top.stl` / `.step` | Domed top z 14–34.7: both buttons and their hinges, wheel strip and opening, wheel brackets, detent arm, board posts, plunger sockets |
| `keys.stl` / `.step` | Both plungers (2 bodies) |
| `key_L.stl` / `.step`, `key_R.stl` / `.step` | One plunger per file, for services that reject multi-body files |
| `wheel.stl` / `.step` | Wheel in one piece: tyre, 24-detent wavy ring, 2 mm axle stubs |
| `assembly.step` | All parts in place, plus reference models of the board, switches, battery and screw heads |
| `checks.json` | Every check result |
| `renders/*.png` | `iso_front_left.png`, `iso_rear_right.png`, `top.png`, `side_left.png`, `side_right.png`, `section_keys.png`. Grey top, black base and wheel, white background. |
| `wip/` | The skin prototype and the checks from before this reshape (`checks_before.json`) |

## Orientation

- **Board:** unrotated, shifted +2.5 mm in X.
  - Board X runs across the shell. The board's top edge (board Y = 0) faces the front.
  - The board sits 1.03 mm from the right inner wall and 6.03 mm from the left one.
- **Shell frame:** X right, Y forward (+Y = nose), Z up, z = 0 at the shell bottom.
  - `shell_x = board_x − 30 + 2.5`
  - `shell_y = 51.0 − board_y`
  - Board bottom at z 13.5, top at z 15.1.
- **Switches:**
  - SW_L1 at board (6.30, 31.50) → shell (−21.20, 19.50)
  - SW_R1 at board (42.15, 30.00) → shell (14.65, 21.00)
- **Wheel:** on the centreline at (0, 20.25, 27.867), axle along X.
- **USB-C:** opening in the left (−X) wall, centred on J5.
- **Battery:** 6 (Z) × 60 (X) × 90 (Y) under the board. Box x −30…30, y −46.7…43.3, z 3.5…9.5.

**Verified from the PCB file** (the script stops if any value differs):
- outline 60.0 × 58.0 mm;
- SW_L1 / SW_R1 positions and 0° rotation;
- holes H1–H4 at ±26.6 / ±25.6 from the board centre;
- J5 origin (3.67, 21.00) at −90°, its opening facing board −X.

## Dimensions

| Item | Value |
|---|---|
| Plan | 130 mm long (y −60…70) and 76 mm at the widest (y −25). 70.06 mm wide over the board / USB zone (y > 22), widening behind it with a cosine ramp. Nose: ellipse 35.03 × 20. Rear: superellipse, exponent 3. |
| Height | Vertical band z 0–14 (split line between `base` and `top`). Above it, a superellipse dome (exponent 3) up to **34.7 mm** at y −8, 60 % back from the nose. |
| Wall / floor | 1.5 / 1.5 mm |
| Buttons | Two, mirror-symmetric about x = 0. Each spans \|x\| 0.3…27.0, from the y = 10 hinge to 4.0 mm inside the outline at the front. 0.6 mm slots all round and down the centreline. Free corners R2. The side slots run past the hinge to y 7.65 and end in Ø1.6 relief holes. |
| Fixed strip around the wheel | x ±6.5, from the rear up to y 31.26, between the buttons |
| Wheel opening | x ±4.6, y 11.12…29.76, plan corners R1. Leaves **1.9 mm** of strip at each side and **1.5 mm** in front. |
| Living hinges | At y = 10 under each button: a Ø2.5 groove swept along the curved skin. Its depth follows the dome, so **1.0 mm** of skin is left across the whole button width. |
| Plungers | Ø4.5 body, Ø3.0 tip (45° taper), Ø2.4 peg into a Ø2.6 × 2.5 socket in a Ø6 boss under each button. Tip **0.2 mm** above the switch actuator at rest. |
| Front posts H1/H2 | Ø5 boss on the board top, cut **2.0 mm** (vertical) below the button underside. A 2 mm rib ties each one to the fixed side wall. The posts still clamp the board top. |
| Wheel | Ø20 × 8 mm: tyre, 1.2 mm groove and 1.5 mm detent ring. Ø2.0 × 2.5 axle stubs. Top 3.5 mm above the skin. |
| Detents | 24 waves, peak R 9.6, valley R 9.1. The arm tip rests 0.1 mm clear of a valley and is pushed 0.39 mm down on each peak. |
| USB-C opening | 13.15 × 7.3 mm: the plug overmold (12.35 × 6.5 max, USB-IF) plus 0.4 mm all round. The outer skin is 7.525 mm from the J5 face, and the plug stops 0.5 mm short of it. |

## Clearances

Minimum surface-to-surface distance in the assembly, in mm. "Before" is `wip/checks_before.json`, from the previous box shell.

| Pair | Now | Before | Note |
|---|---:|---:|---|
| base – top | 0.000 | 0.000 | base and top meet at the z = 14 split line |
| base – wheel | **14.993** | 14.426 |  |
| base – key_L | **12.563** | 12.520 |  |
| base – key_R | 16.859 | 16.859 |  |
| base – board | 0.000 | 0.000 | 2 front columns support the board bottom at H1/H2 |
| base – switches | **9.456** | 9.430 |  |
| base – battery | 0.000 | 0.000 | cell rests on 4 rib standoffs |
| base – screw_heads | 0.106 | 0.106 | cover screw heads seat in the counterbores |
| top – wheel | 0.075 | 0.075 | Ø2.15 axle seat on the Ø2.0 axle |
| top – key_L | 0.001 | 0.001 | plunger seated in the tongue socket |
| top – key_R | 0.001 | 0.001 | plunger seated in the tongue socket |
| top – board | 0.000 | 0.000 | 4 bosses clamp the board top at the M2.5 holes |
| top – switches | **8.120** | 9.300 |  |
| top – battery | 4.814 | 4.814 |  |
| top – screw_heads | 1.600 | 1.600 |  |
| wheel – key_L | **12.464** | 9.178 |  |
| wheel – key_R | **5.904** | 9.178 | expected drop: wheel moved to the centreline |
| wheel – board | **2.156** | 1.883 |  |
| wheel – switches | **7.656** | 10.925 |  |
| wheel – battery | **8.367** | 8.000 |  |
| wheel – screw_heads | **25.056** | 22.612 |  |
| key_L – key_R | 31.381 | 31.381 |  |
| key_L – board | 2.401 | 2.401 |  |
| key_L – switches | 0.200 | 0.200 | plunger tip to switch actuator, at rest |
| key_L – battery | 10.100 | 10.100 |  |
| key_L – screw_heads | 20.089 | 20.089 |  |
| key_R – board | 4.500 | 4.500 |  |
| key_R – switches | 0.200 | 0.200 | plunger tip to switch actuator, at rest |
| key_R – battery | 10.100 | 10.100 |  |
| key_R – screw_heads | 25.020 | 25.020 |  |
| board – switches | 0.000 | 0.000 | switches are soldered to the board |
| board – battery | 4.000 | 4.000 |  |
| board – screw_heads | 0.000 | 0.000 | rear board screw heads seat on the board bottom |
| switches – battery | 5.600 | 5.600 |  |
| switches – screw_heads | 17.673 | 17.673 |  |
| battery – screw_heads | 2.250 | 2.250 |  |

Values in **bold** changed. Every overlap volume is 0. Pairs that touch on purpose:
- top bosses on the board top;
- base columns under H1/H2;
- the cell on its ribs;
- plungers seated in their sockets;
- screw heads in their seats;
- base and top at the split line.

More clearances:
- **Battery to the base walls, clips, stops and columns** (ribs excluded): 1.000 mm.
- **USB-C plug overmold:** 0.384 to base, 0.383 to top, 0.500 to the J5 face.
- **Detent tip to the wheel ring:** 0.1 mm in a valley.
- **Nearest board components:**
  - top – J_RADAR1: 1.35
  - base – J6: 2.428
  - wheel – U8: 2.156
  - key_L – J1: 2.401
  - key_R – U8: 5.361

### Changes from the earlier checks

- **Wheel – key_R: 9.178 → 5.904 mm.** The wheel moved to the centreline (it was at x −3.275). This is the expected and accepted drop.
- **Other changes from the same move:**
  - wheel – key_L 9.178 → 12.464
  - wheel – switches 10.925 → 7.656
  - wheel – board (U8) 1.883 → 2.156
  - wheel top 37.5 → 37.867 mm
- **Top – switches: 9.30 → 8.12 mm.** The lower dome now comes closer to the switch bodies.
- **Wall check:** the previous report gave 1.477 mm (mesh sag on the old shell). The exact skin minimum is now 1.500. Readings that measure to a cut face are excluded (see below).
- **New checks:** web widths and the front-post clearance.
- **Edge rule changed** (see [Accepted sharp edges](#accepted-sharp-edges)). Only the outer surface and the USB-C opening must be rounded. The 24 slot exceptions listed for the old shell no longer apply.

## Wall check

Method:
1. 12,000 points are sampled on the closed shell (base + top fused). The 5,411 that lie on the outer skin are kept.
2. Each is measured along the inward normal through a mesh with 0.01 mm absolute deflection.
3. Every reading below 1.48 mm is re-measured on the exact B-rep.

| | Count |
|---|---:|
| Readings on the outer skin | 5,411 |
| Used | 5,252 (none below 1.48 mm; smallest 1.487 mm at (28.40, 46.14, 29.13)) |
| Excluded: on a living hinge (meant to be 1.0 mm; measured min 1.002) | 18 |
| Excluded: within 0.75 mm of a cut rim, or on the split-line V-groove | 132 |
| Excluded: the nearest inside surface is a slot wall | 9 (0.883–1.162 mm) |

The 9 slot-wall readings sit just outside the side slots (x ±28.4–28.5) and behind the front slot on the steep nose. Each one's nearest inside point lies on the slot's vertical cut face, so it measures the distance to a cut face, not the skin. Every reading that reaches the inside of the dome, or a rim round, is kept. The exact minimum between the outer and inner skin is **1.500 mm**.

## Web check

The material width between every pair of openings, and from each opening to the part's outer edge. The outer edge is the outline at the split line; for base openings it is also the outline at the bottom.
- **Openings:** the slot set with its relief holes, the wheel opening, the USB-C opening and the 4 cover-screw holes.
- **Method:** each opening is the exact volume its cutter removes from the shell, and widths are exact B-rep distances between these volumes.
- **Limit:** 1.2 mm, JLC3DP's quote-page rule (see [Printing](#printing)).

| Web | Width (mm) |
|---|---:|
| slots – wheel_opening | 1.500 |
| screw_H2 – outline at bottom (z = 0) | 3.456 |
| screw_rear_L – outline at bottom (z = 0) | 5.238 |
| screw_rear_R – outline at bottom (z = 0) | 5.238 |
| screw_H1 – outline at bottom (z = 0) | 8.455 |
| slots – outline at split line (z = 14) | 9.683 |
| slots – usb_c | 10.034 |
| usb_c – outline at bottom (z = 0) | 13.130 |

The remaining 24 webs are wider, and all are listed in `checks.json` → `webs`. The USB-C opening crosses the split line by design, as a notch in both parts, so it has no web there.

**Not in the pairwise list:** the strips beside the wheel opening are 1.9 mm, by design (opening edge x ±4.6, slot edge x ±6.5). The pairwise check reports only the smallest gap between the slot set and the wheel opening, which is the 1.5 mm strip in front.

## Accepted sharp edges

The rule (set 2026-11-25): only the outer surface and the USB-C opening edges need at least a 0.5 mm round. The build rounds every such edge; none is left sharp or under-sized.

Sharp edges **inside the slots, at the hinge grooves, under the buttons and on the hanging features** (posts, ribs, sockets, brackets, detent arm) are accepted without rounding. The top part has **56**; the base, wheel and plungers have none. Every one is listed in `checks.json` → `sharp_edges_left`, each with `required: false`.

## Deviations and accepted items

1. **USB-C is in the left wall, not the front.** J5 faces board −X. With the board unrotated, as specified, that is the left wall.
2. **Webs and skin are below JLC3DP's size table** (2.0 mm for 100–200 mm parts). They meet the 1.2 mm quote-page rule and the 1.0 mm MJF recommendation. This is kept as specified: 1.5 mm skin, 1.5 mm strip in front of the wheel, 1.9 mm beside it. Raise `WHEEL_SURROUND` / `SPINE_HW` or `WALL` and rebuild if you want 2.0.
3. **Intentional thin sections:** the 1.0 mm living hinges, and the 0.5 mm rounds on both parts at the split line, which form a small V-groove.
4. **Front-post gap:** 2.0 mm measured vertically, the direction the button tip travels. Because the skin slopes, the shortest straight-line gap is 1.701 mm.
5. **`keys.stl` holds the plungers, not the key surfaces.** The buttons are part of the top so the living hinges work. The plungers are separate so the 0.2 mm gap can be tuned without reprinting the shell.

## Assumptions (not verified)

| Item | Value used | Why |
|---|---|---|
| Battery | 6 × 60 × 90 mm pouch | Given in the brief |
| PTS645 travel | not in the C&K datasheet read | 0.2 mm gap + ~0.25 mm travel needs ~0.045 rad of button rotation about the y = 10 hinge (plungers 9.5 / 11 mm in front of it). Estimate only. |
| JST PH S2B-PH-SM4-TB (J1) height | 5.5 mm | Distributor data only (Digi-Key 5.51 mm, JLCPCB 5.5 mm). No JST or KiCad height. |
| Small QFN/DFN/SON parts | 1.0 mm | Envelope only |

Sourced heights: U8 3.2 mm ([Espressif](https://documentation.espressif.com/esp32-s31-wroom-1_wroom-1u_datasheet_en.html)), SW_RST1 / SW_BOOT1 3.3 mm ([Alps Alpine](https://tech.alpsalpine.com/e/products/detail/SKRTLAE010/)), J5 3.26 mm ([GCT](https://gct.co/files/drawings/usb4110.pdf)). The rest come from the KiCad library 3D models. Each component is modelled as its F.Fab outline × height.

## Assembly

1. Press the plungers (`key_L`, `key_R`) into the sockets under the buttons.
2. Snap the wheel axle stubs up into the bracket throats, then check the wheel clicks.
3. Put the board on the 4 top bosses, components facing up into the top. The front two (H1/H2) are the shortened posts held by a rib from the side wall. Fit 2 × **M2.5 × 6 pan-head thread-forming screws** (for plastic) through H3/H4 from below.
4. Place the cell in the base: slide its front edge under the front lip, then press the rear down past the 4 clips. Route the leads around the board's left edge to J1.
5. Close the base. Fit 4 × **M2.5 countersunk thread-forming screws** from below: **× 20 mm** at the front (H1/H2, through the board) and **× 18 mm** at the rear.

The battery cannot be swapped without opening the base, and there is no battery door. No screw touches the cell.

## Printing

- **Process:** MJF or SLS nylon. Neither needs supports, so the 0.6 mm slots and the hinges come out clean.
- **JLC3DP rules (read 2026-11-26):**
  - **Clearance between moving parts:** **0.6 mm** for Nylon (MJF) and Nylon (SLS) ([design guideline](https://jlc3dp.com/help/article/3D-Printing-Design-Guideline), "5. Model Clearance", B). The slots are 0.6 mm (`SLOT`).
  - **Wall:** JLC3DP states no single absolute nylon minimum.
    - The [instant-quote page](https://jlc3dp.com/3d-printing-quote) ("Upload 3D models") says "Wall thickness > 1.2mm, Thinnest part ≥ 0.8mm" for every upload.
    - The [MJF page](https://jlc3dp.com/3d-printing/multi-jet-fusion) ("Design Guidelines for MJF") lists nylon "Recommended wall thickness 1.0mm".
    - The [design guideline](https://jlc3dp.com/help/article/3D-Printing-Design-Guideline) ("2. Wall Thickness") has an unlabelled size table. Nylon (MJF/SLS) is 1.0 / 1.2 / 1.5 / 2.0 / 2.0 mm for parts of 5×5 / 10×10 / 50×50 / 100×100 / 200×200 mm. The page frames it as guidance: "we recommend that you increase your wall thickness whenever you scale up your design".
    - **What this shell does:** the 1.5 mm skin and the 1.5 mm webs around the wheel opening meet the 1.2 mm quote-page rule, which the web check enforces. They are below the 2.0 mm size-table value for a 130 mm part. That is accepted (see [Deviations](#deviations-and-accepted-items)). The 1.0 mm hinges meet "Thinnest part ≥ 0.8mm".
  - **Multi-body files:** `keys.stl` holds 2 bodies, which JLC3DP flags as an assembly. Upload `key_L.stl` and `key_R.stl` instead.
- **Slots at other services:** check the 0.6 mm slots against your service's minimum gap.
  - [Materialise](https://www.materialise.com/en/academy/industrial/design-am/pa12-mjf) lists 0.4 mm between assembled parts.
  - [Stratasys Direct](https://www.stratasys.com/siteassets/sdm/content---website-storage/design-guides/dg_mjf_stratasys_direct_03062023.pdf) lists 0.5 mm.
  - [Protolabs](https://www.protolabs.com/resources/design-tips/how-to-design-for-nylon-3d-printing/) recommends 0.76–1 mm.

  If your service needs more, change `SLOT` and rebuild.
- **Hinges:** published MJF PA12 living-hinge minimums are 0.5 mm ([Stratasys Direct](https://www.stratasys.com/siteassets/sdm/content---website-storage/design-guides/dg_mjf_stratasys_direct_03062023.pdf), [Materialise](https://reseller.materialise.com/en/manufacturing/materials/pa-12-mjf/design-guidelines)). The 1.0 mm hinge here is stiffer, which is fine for a demo.
- **Orientation:** print `top` with its top surface facing up, so the hinge grooves are not built parallel to Z ([guidance](https://rapidmade.com/designing-snap-fits-and-living-hinges-for-mjf-3d-printing-a-complete-guide-for-pa11-pa12-and-pp/)).

## Sources

- C&K PTS645 datasheet: Ø3.5 round plunger, 4.3 ± 0.1 mm height, 3.45 mm body ([Farnell copy](https://www.farnell.com/datasheets/3762905.pdf))
- GCT USB4110 drawing: 8.94 × 7.35 × 3.26 mm, opening centre 1.68 mm above the PCB, plug overmold 6.5 mm high max ([gct.co](https://gct.co/files/drawings/usb4110.pdf))
- KiCad 3D models, from [kicad-packages3D](https://gitlab.com/kicad/libraries/kicad-packages3D), for the component heights marked "KiCad 3D model" in `shell_model.py`
- JST PH datasheet ([SparkFun copy](https://cdn.sparkfun.com/assets/0/e/f/7/2/ePH.pdf)) gives no height for the SM4-TB; the 5.5 mm figure is from the [JLCPCB listing](https://jlcpcb.com/partdetail/C295747)
