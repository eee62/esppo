# Rev C change: chord power-on (hold both keys 2 s) — CORRECTED

Scope: ESP32-S31 Vision Pro board, Rev C. Rev C has not been ordered. Board outline, mounting holes and tamper trace stay as they are.
This version replaces the earlier spec. The switch wiring and level fixes were wrong and are corrected here.

## Why
- TMR_OUT (U3 TMR3102, push-pull, powered from VSYS) drives the TPS63802 EN node. It swings up to VSYS (about 4.2 V on battery, 4.9 V on USB).
- The MCU is off until EN is high, so the power-on path has to be hardware-only. The MCU then latches itself on.

## Behaviour
1. Both keys pressed together close a series chain from VSYS through both switches. This turns the board on at once.
2. The MCU boots, checks both keys are still held for 2 s, and asserts HOLD.
3. If either key is released before 2 s, the MCU drops HOLD and the board turns off.
4. HOLD stays high until the magnet turns the board off (TMR_OUT goes low). The MCU then drops HOLD.
5. A single key press does nothing.

## Switches
- 4-pin 6 x 6 mm tact switches are SPST: one contact per switch, pins 1-2 joined and pins 3-4 joined, pressing connects both pairs. Use the series chain only. Do NOT wire one pair to VSYS and the other to GND. That shorts the battery.

## Netlist
Rename the net PWR_EN to TMR_OUT.

| Ref | Part / change | Nets |
|---|---|---|
| SW_L | 6x6 4-pin tact (SPST) | VSYS -> WAKE_MID |
| SW_R | 6x6 4-pin tact (SPST) | WAKE_MID -> WAKE |
| R_WAKE_PD | 1 MOhm | WAKE to GND |
| R_SENSE_L_TOP, R_SENSE_L_BOT | ~43 kOhm top, 100 kOhm bottom | WAKE_MID -> node SENSE_L -> GND; SENSE_L -> IO0 |
| R_SENSE_R_TOP, R_SENSE_R_BOT | ~43 kOhm top, 100 kOhm bottom | WAKE -> node SENSE_R -> GND; SENSE_R -> IO4 |
| R_TMR_TOP, R_TMR_BOT | ~43 kOhm top, 100 kOhm bottom | TMR_OUT -> node TMR_SENSE -> GND; TMR_SENSE -> IO13 |
| D_TMR, D_WAKE, D_HOLD | Schottky, low Vf (e.g. BAT54-type, SOT-23) | anode TMR_OUT / WAKE / HOLD -> cathode EN_TPS |
| R_EN_PD | 100 kOhm | EN_TPS to GND |
| U2 TPS63802 | EN pin | EN_TPS |

Divider check: at 4.9 V VSYS, SENSE pins read about 3.4 V (under the ESP32 3.6 V limit). At 4.2 V they read about 2.9 V. The agent must confirm the ESP32 high threshold at the lowest battery voltage and the TPS63802 EN absolute maximum rating.

## MCU pins (from GPIO_audit.md)
- IO0 (LP_GPIO0, pin 8) = SENSE_L (SW_L side of chain)
- IO4 (LP_GPIO4, pin 6) = SENSE_R (SW_R side of chain)
- IO12 (pin 46) = HOLD output, through D_HOLD
- IO13 (pin 47) = TMR_SENSE, divided from TMR_OUT

None of these are strap pins. All four were unconnected in Rev B.

## Charge settings (Rev C)
- Input current limit: ILIM resistor, about 1.1 kOhm, limit about 1.46 A. Verify against the datasheet ILIM formula and range (200-1500 mA).
- Charge current: ISET R3 = 887 Ohm, about 1.0 A (0.33C for 3000 mAh). Verify against the datasheet ISET formula.
- Timer: R49 47k -> 56k, about 7.5 h, which covers a full charge at 1.0 A with margin. Confirm the maximum allowed TMR resistor in the datasheet.
- Keep the voltage cap, BQ29700 protection and the timer. No safety limit is removed.

## Checks
- ERC: no nets with two outputs; no unconnected inputs; no new warnings.
- DRC: no new errors. Tamper trace still continuous with its clearance.
- Power: rerun battery_model.py with the new divider and diode leakage and report the standby change.
- Bench: both keys pressed -> board on; one key released before 2 s -> board off; single key -> nothing.

## Open items
- Schottky part and its JLCPCB stock.
- Exact divider values after the level check.
- Real switch datasheet confirmation of SPST contact layout.
- Charge time measurement on a real cell.
