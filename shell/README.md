# Demo mouse shell — ESP32-S31 Vision Pro Rev C

A printable demo mouse shell. It has two moving front keys and a scroll wheel with 24 detents. The keys press the real tact switches SW_L1 and SW_R1. There are no electronics in the shell and nothing connects to a computer.

`python3 shell_model.py` rebuilds every file in this folder in about 4 minutes. It reads the board directly from `../mouse-revc-src.zip` and does not unpack or change it. It runs all the checks and writes `checks.json`. It needs `cadquery`, `trimesh`, `numpy` and `manifold3d`.

## Bottom line

| Check | Result |
|---|---|
| 1. No overlaps (board, battery, switches, wheel, keys, base, top, screws) | **Pass.** Overlap volume is 0.000 mm³ for all 36 pairs. |
| 2. Key tongue centred on its switch (≤ 1 mm) | **Pass.** Offset is 0.00 mm for both keys. |
| 3. Total height ≤ 38 mm | **Pass.** Keys top out at 34.0 mm and the wheel at 37.5 mm. |
| 4. USB-C opening lines up with J5 | **Pass, with a deviation.** The opening is centred on J5 (Y 30.00, Z 16.78), but in the **left** side wall, as your revised orientation requires. It is not at the front: see Conflicts. |
| 5a. Walls ≥ 1.5 mm | **Pass.** The exact surface-to-surface minimum is 1.500 mm. The 1.0 mm living hinges are intentionally thinner. |
| 5b. No edge sharper than 0.5 mm radius | **Not met on 18 edges, all on the top part's key slots.** 16 edges are unrounded and 2 have only 0.3–0.4 mm. See below. |
| 6. Assembly exported, clearances reported | **Done.** See `assembly.step` and the tables below. |

## Files

| File | Contents |
|---|---|
| `shell_model.py` | Parametric source. All dimensions are in the `P` dataclass at the top. |
| `base.stl` / `.step` | Removable bottom cover: floor, lower walls (z 0–14), battery pocket and clips, cover-screw columns |
| `top.stl` / `.step` | Hump (z 14–34) with the two key tongues and living hinges, wheel opening and brackets, detent arm, board bosses |
| `keys.stl` / `.step` | The 2 key plungers (2 bodies in one file; print 1 of each). Each presses into the socket under its tongue. |
| `wheel.stl` / `.step` | Scroll wheel in one piece: tyre, 24-detent wavy ring and 2 mm axle stubs |
| `assembly.step` | All parts in place, plus reference models of the board, switches, battery and screw heads |
| `checks.json` | Every check result: pairwise clearances, walls, edges, STL watertightness |

All four STLs are watertight. `base`, `top` and `wheel` are single bodies; `keys` holds the two plungers. Units are mm.

## Orientation chosen (as specified)

- **Board rotation:** 0°. Board X runs across the shell, and the board's top edge (board Y = 0) faces the shell front.
- **Board shift:** +2.5 mm in X. That leaves the board 1.0 mm from the right inner wall and 6.0 mm from the left one.
- **Shell frame:** X is right, Y is forward (+Y = front, the key end), Z is up, and z = 0 is the shell bottom.
  - shell_x = board_x − 30 + 2.5
  - shell_y = 51.0 − board_y
- **Board height:** the bottom is at z 13.5 and the top at z 15.1.
- **Keys and wheel:** one row across the width.
  - SW_L at (−21.20, 19.50)
  - SW_R at (14.65, 21.00)
  - Wheel at x −3.275, between the two tongues, with its axle along X
- **USB-C:** the opening is in the left (−X) wall, centred on J5.
- **Battery:** 6 (Z) × 60 (X) × 90 (Y), under the board.
  - Box: x −30…30, y −46.7…43.3, z 3.5…9.5
  - It extends 39.7 mm past the board's rear edge. Its front edge is 7.7 mm behind the board's front edge, clear of the H1/H2 columns. The 60 mm width is the same as the board's.

### Verified from the PCB file

The script parses `ESP32-S31-Vision-Pro-RevC.kicad_pcb` and stops with an error if any value differs:
- Outline 60.0 × 58.0 mm.
- SW_L1 at (6.30, 31.50) and SW_R1 at (42.15, 30.00), both at rotation 0.
- Holes at (3.4, 3.4), (56.6, 3.4), (3.4, 54.6) and (56.6, 54.6), which are ±26.6 / ±25.6 from the board centre.
- **J5:**
  - Footprint origin is (3.67, 21.00) at −90°.
  - Body outline (F.Fab) spans board X −0.005…7.345.
  - Its signal pads sit at X 7.35, the back of the connector. So the opening faces board −X and sits flush with the left board edge, centred at board Y 21.00.

## Dimensions

| Item | Value |
|---|---|
| Outer footprint | 70 × 114 mm. Plan corners R8 at the front, R18 at the rear. |
| Base (vertical walls) | z 0–14. The split line between `base` and `top` is at z 14. |
| Hump | z 14–34 with a 24° taper (from vertical) at the front and rear |
| Hump rounding | R10 where the vertical wall turns into the taper, R12 at the front top, R25 at the rear top (palm), R3 along the top edge |
| Sides | Vertical over the full length, so there is no taper at the keys |
| Wall / floor | 1.5 / 1.5 mm |
| Key tongues | 21 × 18 mm with 0.4 mm slots on three sides. The slots run 1.25 mm past the hinge line and end in Ø1.0 relief holes. |
| Living hinge | A round groove (R1.25) under each tongue's rear edge leaves **1.0 mm** of skin. |
| Key plungers | Ø4.5 body, Ø3.0 tip with a 45° taper, Ø2.4 peg into a Ø2.6 × 2.5 mm socket. Each tip rests **0.2 mm** above its switch plunger. |
| Wheel | Ø20 × 8 mm, made of a tyre (5.3 mm wide), a 1.2 mm groove and a 1.5 mm detent ring. Axle stubs are Ø2.0 × 2.5 mm. The top is 3.5 mm above the shell. |
| Detents | 24 waves (15° pitch), peak radius 9.6 mm, valley radius 9.1 mm |
| Detent arm | 1.3 × 1.3 mm cantilever on a bracket hanging from the inner ceiling. Its rounded tip (R0.7) sits 0.1 mm clear of a valley and is pushed 0.39 mm down on each peak. |
| Wheel mount | Two 1.6 mm brackets from the ceiling. The axle seat is Ø2.15 with a 1.7 mm snap throat. |
| Battery hold | 4 rib standoffs (2 mm), a rear stop, a front lip over the cell's front edge and 4 flexing clips (1.3 mm arms, 0.9 mm hook) |
| Board mount | 4 bosses from the top clamp the board top at H1–H4. 2 base columns support H1/H2 from below. |
| USB-C opening | 13.15 × 7.3 mm: the plug overmold plus 0.4 mm clearance all round. Height 6.5 mm max is from the GCT drawing. Width 12.35 mm is the USB Type-C overmold limit as I recall it, not checked against the spec. The outer skin is 7.5 mm from the J5 face, and the plug stops 0.5 mm short of it. |

## Clearances (from `assembly.step`, minimum between surfaces)

| Pair | Min clearance (mm) | Note |
|---|---:|---|
| key plunger – switch plunger | **0.200** | gap at rest, both keys |
| top (axle seats) – wheel | 0.075 | Ø2.15 seat on a Ø2.0 axle |
| detent tip – wheel ring (at rest, in a valley) | 0.100 | by design |
| wheel – board (U8 module, height assumed 3.5) | 1.653 | |
| key_L plunger – board | 2.312 | |
| top – nearest board component (J_RADAR1) | 1.350 | |
| base – nearest board component (J6) | 2.405 | |
| battery – base walls, clips, stops, columns | **1.000** | not counting the ribs it rests on |
| battery – board | 4.000 | |
| battery – screw heads (rear board screws) | 2.250 | |
| battery – top | 4.814 | |
| USB-C plug overmold – base / top | 0.384 / 0.383 | |
| USB-C plug overmold – J5 face | 0.500 | |
| top – rear board screw heads | 1.600 | |
| switch body – any shell part | ≥ 9.3 | no shelf or bracket touches a switch body |

Parts that touch on purpose (overlap is still 0):
- Top bosses on the board top.
- Base columns under H1/H2.
- Cell on its ribs.
- Plungers seated in their sockets.
- Screw heads in their seats.
- Base and top at the split line.

## Conflicts and deviations

1. **USB-C is not at the front (orientation conflict, reported in the first round).**
   - J5's opening faces board −X.
   - At 0° and 180° the keys form a row, but J5 faces the left or right side.
   - At 90° and 270° the keys sit one behind the other, 35.85 mm apart. Only at 270° does J5 face the front.
   - Your revised brief fixed 0° with the opening in the left wall, and that is what was built.
2. **18 edges on the top part are sharper than 0.5 mm.** Base, wheel and plungers have none.
   - **16 have no fillet**, all around the left key's left slot.
     - The slot's outer wall (x −32.10) lands 0.10 mm inside the start of the 3 mm side fillet (x −32.00). OCC cannot round edges that close to tangent.
     - They are the slot's top and bottom edges plus the corner edges at its ends, and the tongue's left underside edge. All are inside the 0.4 mm slot.
     - I tried a 2.5 mm side fillet instead: it left 33 sharp edges, so it does not help.
     - A larger side fillet is untested and would go against the brief.
   - **2 have reduced radii.** One edge at the right end of the left key's front slot has 0.4 mm. The left key's hinge-groove end has 0.3 mm. Locations are in `checks.json` → `reduced_radius_edges`.
3. **Thin sections that are intentional:**
   - The living hinges are 1.0 mm, as the brief asks.
   - The split line has 0.5 mm rounds on both parts, which form a small V-groove.
4. **The project files are not at the repo root.** The KiCad PCB and notes are only inside `mouse-revc-src.zip`. They were read from the zip and nothing in it was changed.
5. **`keys.stl` holds the plungers, not the key surfaces.** The tongues have to be part of the top so the living hinge can work. The plungers are separate so you can tune the 0.2 mm gap without reprinting the shell.

## Assumptions (not verified)

| Item | Value used | Why |
|---|---|---|
| Battery | 6 × 60 × 90 mm pouch | Given in the brief, not yet verified |
| USB-C plug overmold width | 12.35 mm | From memory of the USB Type-C spec, not checked here. A larger overmold will not fit the opening. |
| PTS645 travel | not in the C&K datasheet I read | The tongue can move more than 1 mm at the switch. The 0.2 mm gap plus about 0.25 mm of travel needs a hinge rotation of about 0.05 rad, which gives about 1.6 % strain in the 1.0 mm hinge (estimate). |
| ESP32-S31-WROOM-1U height | 3.5 mm | No datasheet height found. It sets the 1.65 mm wheel-to-board clearance. |
| JST PH S2B-PH-SM4-TB (J1) height | 6.0 mm | JLCPCB lists 5.5 mm, not confirmed. |
| Alps SKRTLAE010 (SW_RST1, SW_BOOT1) height | 4.0 mm | One distributor lists 3.3 mm, not confirmed. |
| Small QFN/DFN/SON parts | 1.0 mm | Envelope only |

Every other height comes from KiCad's 3D models or the GCT USB4110 drawing (3.26 mm). Each component is modelled as its F.Fab body outline × height.

## Assembly

1. Press the plungers (`keys`) into the sockets under the tongues.
2. Snap the wheel axle stubs up into the bracket throats, then check the wheel clicks.
3. Put the board on the 4 top bosses, components facing up into the top. Fit 2 × **M2.5 × 6 pan-head thread-forming screws** (for plastic) through H3/H4 from below.
4. Place the cell in the base: slide its front edge under the front lip, then press the rear down past the 4 clips. Route the leads around the board's left edge to J1.
5. Close the base. Fit 4 × **M2.5 countersunk thread-forming screws** from below: **× 20 mm** at the front (H1/H2, through the board) and **× 18 mm** at the rear.

The battery cannot be swapped without opening the base, and there is no battery door. No screw touches the cell.

## Printing

- **Process:** MJF or SLS PA12 (no supports, so the 0.4 mm slots and hinges come out clean).
- **0.4 mm slots:** check them against your service's minimum gap. [Materialise](https://www.materialise.com/en/academy/industrial/design-am/pa12-mjf) lists 0.4 mm clearance between assembled parts. [Stratasys Direct](https://www.stratasys.com/siteassets/sdm/content---website-storage/design-guides/dg_mjf_stratasys_direct_03062023.pdf) lists 0.5 mm. [Protolabs](https://www.protolabs.com/resources/design-tips/how-to-design-for-nylon-3d-printing/) recommends 0.76–1 mm. If your service needs more, change `SLOT` and rebuild. The tongue stays centred on the switch.
- **Hinges:** published MJF PA12 living-hinge minimums are 0.5 mm ([Stratasys Direct](https://www.stratasys.com/siteassets/sdm/content---website-storage/design-guides/dg_mjf_stratasys_direct_03062023.pdf), [Materialise](https://reseller.materialise.com/en/manufacturing/materials/pa-12-mjf/design-guidelines)). The 1.0 mm hinge here is stiffer and is fine for a demo.
- **Orientation:** print `top` with its top surface facing up, so the hinge grooves are not built parallel to Z ([guidance](https://rapidmade.com/designing-snap-fits-and-living-hinges-for-mjf-3d-printing-a-complete-guide-for-pa11-pa12-and-pp/)).

## Sources

- C&K PTS645 datasheet: Ø3.5 round plunger, 4.3 ± 0.1 mm height, 3.45 mm body ([Farnell copy](https://www.farnell.com/datasheets/3762905.pdf))
- GCT USB4110 drawing: 8.94 × 7.35 × 3.26 mm, opening centre 1.68 mm above the PCB, plug overmold 6.5 mm high max ([gct.co](https://gct.co/files/drawings/usb4110.pdf))
- KiCad 3D models, from [kicad-packages3D](https://gitlab.com/kicad/libraries/kicad-packages3D), for the component heights marked "KiCad 3D model" in `shell_model.py`
- JST PH datasheet ([SparkFun copy](https://cdn.sparkfun.com/assets/0/e/f/7/2/ePH.pdf)) gives no height for the SM4-TB; the 5.5 mm figure is from the [JLCPCB listing](https://jlcpcb.com/partdetail/C295747)
