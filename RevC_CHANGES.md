# ESP32-S31 Vision Pro — Rev C changes (from Rev B)

Date: 2026-11-22. Base: Rev B as of 2026-11-15 (`../ESP32-S31-Vision-Pro/`, not modified).
Netlist spec: `rev_c_chord_power_on.md` (the corrected version). Wherever the original brief and the corrected spec disagree, the corrected spec wins: diode-OR instead of an OR gate, series switch chain, and sense dividers instead of pull-ups.

Rev C adds three things: chord power-on, two 6 × 6 mm keys, and a charger sized for a cell of up to 3000 mAh. Everything else is Rev B as it was. The outline, holes, tamper loop and RF keep-out are byte-identical to Rev B, and no Rev B footprint moved.

## 1. Netlist changes

| # | Change | Rev B | Rev C |
|---|---|---|---|
| 1 | Net rename | `PWR_EN` (U3.3 → U2.1) | `TMR_OUT` (U3.3 TMR3102 output) |
| 2 | New net | — | `EN_TPS` = U2.1 (TPS63802 EN), cathodes of D_TMR1/D_WAKE1/D_HOLD1, R_EN_PD1 |
| 3 | Diode-OR | U2.1 driven directly by U3 | TMR_OUT → D_TMR1, WAKE → D_WAKE1, HOLD → D_HOLD1 (anode → cathode `EN_TPS`) |
| 4 | Key chain | — | VSYS → SW_L1 → `WAKE_MID` → SW_R1 → `WAKE` (series, SPST) |
| 5 | WAKE pull-down | — | R_WAKE_PD1 1 MΩ, WAKE → GND |
| 6 | Sense SW_L side | U8 pin 8 IO0 NC | `WAKE_MID` → R_SENSE_L_TOP1 47k → `SENSE_L` (IO0) → R_SENSE_L_BOT1 100k → GND |
| 7 | Sense chord | U8 pin 6 IO4 NC | `WAKE` → R_SENSE_R_TOP1 47k → `SENSE_R` (IO4) → R_SENSE_R_BOT1 100k → GND |
| 8 | Magnet sense | U8 pin 47 IO13 NC | `TMR_OUT` → R_TMR_TOP1 470k → `TMR_SENSE` (IO13) → R_TMR_BOT1 1M → GND |
| 9 | Self-latch | U8 pin 46 IO12 NC | `HOLD` (IO12) → D_HOLD1 → EN_TPS |
| 10 | EN pull-down | — | R_EN_PD1 100k, EN_TPS → GND |
| 11 | BQ24075 mode | EN1 (pin 6) = `CHG_EN1` = VBUS via R50; EN2 (pin 5) = GND → USB500 (475 mA) | EN2 (pin 5) = `CHG_EN2` = VBUS via R50; EN1 (pin 6) = GND → **ILIM resistor mode** |

Pin check against `GPIO_audit.md` (Rev B): IO0 (pin 8), IO4 (pin 6), IO12 (pin 46) and IO13 (pin 47) were all "not connected". None of them is a strap pin (IO36/37/60/61) or a pad-JTAG pin. IO0 and IO4 are LP pins. The regenerated audit is in `fab/GPIO_audit.md`.

Item 11 is not in the spec. The brief requires it ("ILIM resistor method, EN1 low, EN2 high"), because a Rev B board in USB500 mode would ignore R2 entirely.

## 2. Components added / changed

Reference designators carry a `1` suffix (SW_L1 …) like the Rev B special refs (SW_RST1, R_TAMPER_BYP1). KiCad treats a reference that ends in a letter as unannotated.

| Ref | Value | Footprint | MPN | LCSC | Status |
|---|---|---|---|---|---|
| SW_L1, SW_R1 | 6 × 6 tact, SPST, H 4.3 mm, 2.6 N | Button_Switch_SMD:SW_SPST_PTS645Sx43SMTR92 | C&K PTS645SK43SMTR92LFS | C221871 | new |
| D_TMR1, D_WAKE1, D_HOLD1 | BAT54WS Schottky | Diode_SMD:D_SOD-323 | BAT54WS | C7502694 | new |
| R_EN_PD1 | 100k | 0402 | 0402WGF1003TCE | C25741 | new (existing line) |
| R_WAKE_PD1 | 1M | 0402 | 0402WGF1004TCE | C26083 | new (existing line) |
| R_SENSE_L_TOP1, R_SENSE_R_TOP1 | 47k | 0402 | 0402WGF4702TCE | C25792 | new (existing line) |
| R_SENSE_L_BOT1, R_SENSE_R_BOT1 | 100k | 0402 | 0402WGF1003TCE | C25741 | new (existing line) |
| R_TMR_TOP1 | 470k | 0402 | 0402WGF4703TCE | C25790 | new line |
| R_TMR_BOT1 | 1M | 0402 | 0402WGF1004TCE | C26083 | new (existing line) |
| R2 (ILIM) | 2.2k → **1.13k 1 %** | 0402 | 0402WGF1131TCE | C67014 | changed, new line |
| R3 (ISET) | 2.2k → **887R 0.1 %** | 0402 | RT0402BRD07887RL | C852938 | changed, new line (the 1 % thick-film 887R C474137 has 9 in stock) |
| R49 (TMR) | 47k → **68k 1 %** | 0402 | 0402WGF6802TCE | C36871 | changed, new line |
| J1 | value text "LiPo 603450" → "LiPo 1S <=3000mAh" | unchanged | unchanged | unchanged | text only |

The JLC BOM grows from 45 to 51 lines. Every PTS645, BAT54WS, 887R and 1.13k line is new, plus 470k and 68k.

## 3. Deviations from the spec text, with reasons

| Spec says | Rev C uses | Why |
|---|---|---|
| Key dividers "~43k top / 100k bottom" (open item) | 47k / 100k (ratio 0.680) | The pin stays ≤ 3.6 V (ESP32 VIH max = VDD + 0.3) up to VSYS 5.29 V, which covers USB VBUS max 5.25 V. 43k/100k reaches 3.6 V at VSYS 5.15 V. 47k is already on the BOM. |
| TMR divider "~43k / 100k" (open item) | 470k / 1M (same 0.680 ratio) | This divider is biased whenever the magnet holds the board on, including deep sleep. 43k/100k costs 25 µA there; 470k/1M costs 2.5 µA. IIH of 50 nA × 318 kΩ gives a 16 mV error, which is negligible. |
| R49 47k → 56k | **68k** | 56k gives only 5.6 h at KTMR min (36 s/kΩ), which is 1.28× a 4.38 h worst-tolerance charge, under the brief's 1.5× rule. 68k gives 1.55× in that case. It is ≤ 72k (DS max). |
| ILIM "about 1.1 kΩ" | 1.13k 1 % | DS RILIM range is 1.1–8 kΩ. A 1.1k 1 % part can be 1.089 kΩ. |
| D_x "BAT54-type, SOT-23" | BAT54WS, SOD-323 | Same die family. Preferred part with 411k in the DB, and a smaller pad set in the crowded area. |
| Brief: 3-input OR gate, R_IO0_PU/R_IO4_PU, R_HOLD_PD | not fitted | Replaced by the corrected spec (diode-OR, dividers). R_EN_PD1 already defines EN_TPS when HOLD floats at reset. |

## 4. Charge values and calculations

Datasheet: TI BQ2407x, SLUS810N (Oct 2021), Electrical Characteristics and §9.3.5.6 / §10.2.2.2.
Design ceiling (documented assumption): **1S LiPo ≤ 3000 mAh**, a placeholder until the shell job picks the cell.

**Charge current, R3 (ISET).** ICHG = KISET / RISET with KISET = 797 / 890 / 975 AΩ (min/typ/max). RISET must be 590–8900 Ω and 1 % or better.
R3 = 890 / 1.0 A = 890 Ω → 887 Ω (E96).
ICHG = 890 / 887 = **1.003 A typ** (0.899–1.099 A) = **0.33 C** at 3000 mAh, 0.37 C at the KISET maximum. The ceiling is 0.5 C = 1.5 A, so 1.0 A is fast but leaves margin for tolerance and a hot cell.
Pre-charge = KPRECHG / R3 = 88 / 887 = 99 mA (79–120). Termination = 10 % of ICHG = 100 mA.

**Input current limit, R2 (ILIM), EN2 = HI and EN1 = LO.** IIN-MAX = KILIM / RILIM, with KILIM = 1500 / 1610 / 1720 AΩ in the 0.5–1.5 A band. The range is 200–1500 mA, RILIM 1.1–8 kΩ.
Needed: charge 1.003 A + clip load (360 mA on 3.3 V → 360 × 3.3 / (5.0 × 0.93) = 255 mA at VSYS) = 1.26 A.
R2 = 1.13 kΩ → **1.425 A typ** (1.314 A min, 1.537 A max). The minimum still covers charge + clip load. The worst-case burst (407 mA at VSYS) briefly exceeds it; DPPM then cuts charge current, not system current. The Rev B limit was 475 mA (USB500).

**Safety timer, R49 (TMR).** tMAXCHG = 10 × KTMR × RTMR, with KTMR = 36 / 48 / 60 s/kΩ and RTMR 18–72 kΩ. tPRECHG = KTMR × RTMR.
Charge model: CC to 80 % of capacity, then exponential CV taper to ITERM. For 3000 mAh at 1.003 A: CC 2.39 h + CV 1.53 h = **3.92 h**.
For 1.5× margin: tMAXCHG ≥ 5.9 h typical, or ≥ 6.6 h with min ICHG (4.38 h). At KTMR min that needs RTMR ≥ 6.6 h × 3600 / (10 × 36) = 65.7 kΩ → **68k**.

| Case | Charge time | Timer | Margin |
|---|---:|---:|---:|
| Typical parts | 3.92 h | 9.07 h | **2.31×** |
| Min ICHG + min KTMR | 4.38 h | 6.80 h | **1.55×** |
| Aged cell (CV at 70 %), typical parts | 4.39 h | 9.07 h | 2.07× |
| Aged cell + worst tolerance | 4.90 h | 6.80 h | 1.39× (fault, re-plug restarts) |

Pre-charge timer: 54 min typ, 41 min min.

**Unchanged safety limits.** VBAT(REG) 4.2 V (BQ24075 fixed). BQ29700 over-charge 4.275 V, over-discharge 2.8 V and over-current all stay. TH1 10k NTC on TS stays. The timer is still enabled (TMR is not tied to VSS). Thermal regulation at TJ 125 °C stays.

## 5. Layout changes

- **Added:** SW_L1 at (106.30, 91.50) and SW_R1 at (142.15, 90.00), both top side, rotation 0. Positions relative to the board are in README.md.
- **Diode-OR cluster** (D_TMR1, D_WAKE1, D_HOLD1, R_EN_PD1, R_WAKE_PD1, R_TMR_TOP1/BOT1) sits at x 112–119, y 87–94, beside SW_L1. **Sense dividers** sit at x 121–125, y 72–76, next to U8 pins 6 and 8.
- **Routing:** only the new connections were routed, with all Rev B copper locked (`prouter.py`, LOCK=1, F.Cu and B.Cu only, In1/In2 planes untouched). New copper per net: HOLD 70 mm, WAKE_MID 76 mm, WAKE 62 mm, TMR_SENSE 33 mm, TMR_OUT 28 mm, EN_TPS 18 mm, GND 14 mm, SENSE_L 7 mm, SENSE_R 6 mm, VSYS 6 mm.
- **Removed from Rev B copper:** 46 items, all on PWR_EN, CHG_EN1 or GND. No other net changed.
  - The 13-segment PWR_EN stub between U3.3 and its via.
  - The dead B.Cu EN_TPS run below the new T-junction at (104.3, 94.55).
  - The U1 pin 5 GND link and the pin 6 EN1 jog.
  - Two GND stitching vias under the SW_L1 pads, at (103.0, 90.0) and (103.0, 93.0).
- **U1:** pin 5 (EN2) now runs to the existing R50 via. Pins 4 and 6 tie into the exposed pad with 0.2 mm stubs.
- **FAN_PWM** keeps its Rev B route. SW_R1 is centred on that diagonal track, which passes between its pads at about 0.15 mm clearance (rule 0.10 mm).
- **Silkscreen:**
  - "Rev B" → "Rev C".
  - "ESP32-S31 VISION PRO" moved to (125.0, 90.75) and reduced to 0.8 mm.
  - "I_3V3" moved to (118.3, 85.9).
  - The battery "+" was removed (SW_L1 now covers its spot). The J1 pin-1 label "VBAT" became "+VBAT" at 0.6 mm.
  - New legends "SW_L" and "SW_R".
  - The fab-notes title now says Rev C.

## 6. How it was built (re-runnable)

`pcb_build/revc/scripts/`:
1. `design.py` → `gen_sch.py` writes the 4 sheets. `check_netlist.py` confirms 137 components with 0 mismatches.
2. `run_revc.sh` runs in order:
   - `pcb_revc.py`: the ECO on the Rev B board.
   - `prouter.py`: routes only the new nets, with Rev B copper locked.
   - `pcb_revc_post.py`: prunes dead copper, restores the Rev B lock flags, and checks that nothing else changed net and that the tamper loop is identical.
   - `pcb_revc_silk.py`: the silkscreen changes.
   - DRC.
3. `verify_revc.py` compares the outline, holes, keep-out, side and moved parts against Rev B.
4. `export_revc.sh` writes the fab package.
5. `../scripts/battery_model.py` writes the power model.
