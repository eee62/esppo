# ERC / DRC: Rev B vs Rev C

Tool: KiCad 10.0.8 `kicad-cli`. Rev B reports were copied from the Rev B project (2026-11-15). The Rev C reports were run on 2026-11-22 with the same settings: `--severity-all`, DRC with `--schematic-parity --refill-zones`.

| Check | Rev B | Rev C | Change |
|---|---:|---:|---|
| ERC errors | 0 | 0 | none |
| ERC warnings | 0 | 0 | none |
| ERC ignored checks | 3 (four-way junction, SPICE, footprint filters) | same 3 | none |
| DRC violations (errors + warnings) | 0 | 0 | none |
| DRC unconnected pads | 0 | 0 | none |
| DRC footprint errors | 0 | 0 | none |
| Schematic parity issues | (not listed) | 0 | — |
| DRC ignored checks | 5 | same 5 | none |

Files: `RevB-ERC.rpt`, `RevB-DRC.rpt`, `RevC-ERC.rpt`, `RevC-DRC.rpt`.

## New-net driver check (from the Rev C netlist, pin electrical types)

| Net | Pins | Outputs on net | Inputs driven? |
|---|---|---:|---|
| TMR_OUT | U3.3 (output), D_TMR1.A, R_TMR_TOP1 | 1 | — |
| EN_TPS | U2.1 EN (input), D_TMR1.K, D_WAKE1.K, D_HOLD1.K, R_EN_PD1 | 0 | yes: three diodes + 100k pull-down, never floating |
| WAKE | SW_R1.2, D_WAKE1.A, R_WAKE_PD1, R_SENSE_R_TOP1 | 0 | passive only; defined low by 1M |
| WAKE_MID | SW_L1.2, SW_R1.1, R_SENSE_L_TOP1 | 0 | passive only; defined low through 147k when open |
| HOLD | U8.46 (bidir), D_HOLD1.A | 0 | EN_TPS is pulled down while IO12 is high-Z |
| SENSE_L / SENSE_R / TMR_SENSE | U8 pin (bidir) + divider | 0 | divider sets the level |
| CHG_EN2 | U1.5 EN2 (input), R50 to VBUS | 0 | yes |

No net has two outputs, and no input is left unconnected. Both switches have both pins on nets: SW_L1 is VSYS/WAKE_MID and SW_R1 is WAKE_MID/WAKE.

## Tamper loop

- 148 TAMPER_LOOP copper items in Rev B and 148 in Rev C. Each item matches on type, layer, endpoints, width and lock flag (`pcb_revc_post.py`).
- DRC reports 0 unconnected pads, so the loop is still continuous from R43 to J_TAMPER_LID1 / R_TAMPER_BYP1.
- The Tamper net class still has 0.30 mm clearance and DRC passes. The nearest new copper is 0.71 mm away (HOLD).

## Protected geometry (`verify_revc.py`)

- Edge.Cuts identical to Rev B: 60.0 × 58.0 mm, R 2.0 mm.
- H1–H4 at the same positions: (103.4, 63.4), (156.6, 63.4), (103.4, 114.6), (156.6, 114.6).
- RF keep-out rule area identical.
- No footprint on the bottom side, and no Rev B footprint moved or rotated.
