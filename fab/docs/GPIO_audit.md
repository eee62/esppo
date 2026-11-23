# ESP32-S31-WROOM-1U GPIO assignment and strapping / wake audit (Rev C, generated from design.py)

Sources: ESP32-S31-WROOM-1 & -1U datasheet v0.5 (pin table, Chapter 4 Boot Configurations). Strapping pins are IO36, IO37, IO60 and IO61 only;
LP (RTC-domain, deep-sleep capable) pins are IO0-IO7. On this chip the boot strap is **IO61**; GPIO0 is LP_GPIO0 / XTAL_32K_N and is a normal LP pin.

| Pin | Module pin name | Net | Domain | Function | Boot / wake audit |
|---|---|---|---|---|---|
| 1 | GND | GND |  | power / ground |  |
| 2 | 3V3 | +3V3_SYS |  | power / ground |  |
| 3 | EN | ESP_EN |  | CHIP_PU: R12 10k pull-up, C20 1 uF, SW_RST1 (no external watchdog) |  |
| 4 | IO2/LP_GPIO2 | RADAR_OUT | LP  | HLK-LD2410S presence OUT, R61 100k pull-down -> EXT1 wake (high) | OK: LP pin, EXT1-capable wake source; not a strap |
| 5 | IO3/LP_GPIO3 | TAMPER_SENSE | LP  | tamper loop return, R44 1M pull-up, C44 10n -> EXT1 wake (high = loop cut / lid open) | OK: LP pin, EXT1-capable wake source; not a strap |
| 6 | IO4/LP_GPIO4 | SENSE_R | LP  | Rev C: chord closed (WAKE) via R_SENSE_R 47k/100k divider (input, internal pulls off) | OK: LP pin, not a strap; divider keeps it <= 3.6 V up to VSYS 5.29 V; reads high only from VSYS >= 3.64 V |
| 7 | IO5/LP_GPIO5 | SENS_INT | LP  | MAX17048 /ALRT (open drain, active low), R39 100k pull-up -> wake (low) | OK: LP pin, EXT1-capable wake source; not a strap |
| 8 | IO0/LP_GPIO0 | SENSE_L | LP  | Rev C: SW_L side of the key chain (WAKE_MID) via R_SENSE_L 47k/100k divider (input, internal pulls off) | OK: LP pin, not a strap; divider keeps it <= 3.6 V up to VSYS 5.29 V; reads high only from VSYS >= 3.64 V |
| 9 | IO1/LP_GPIO1 | TAMPER_DRV | LP  | drives the tamper loop through R43 10k (hold low in deep sleep) | OK: LP pin, state held in deep sleep; not a strap (XTAL_32K unused) |
| 10 | IO6/LP_I2C_SCL | I2C_SCL | LP  | LP I2C SCL: MAX17048, SHT40, J3 (R20 4.7k) | OK |
| 11 | IO7/LP_I2C_SDA | I2C_SDA | LP  | LP I2C SDA (R19 4.7k) | OK |
| 12 | IO35 | CAM_PWDN |  | OV5640 PWDN (R35 10k pull-down) | OK |
| 13 | IO33/USB_D- | USB_DN |  | USB D- (via USBLC6) | OK |
| 14 | IO34/USB_D+ | USB_DP |  | USB D+ (via USBLC6) | OK |
| 15 | IO20/SDIO_D0 | SD_D0 |  | SDIO D0 (R24 47k) | OK |
| 16 | IO21/SDIO_D1 | SD_D1 |  | SDIO D1 (R25 47k) | OK |
| 17 | IO22/SDIO_D2 | SD_D2 |  | SDIO D2 (R26 47k) | OK |
| 18 | IO23/SDIO_D3 | SD_D3 |  | SDIO D3 (R27 47k) | OK |
| 19 | IO24/SDIO_CLK | SD_CLK_R |  | SDIO CLK -> R56 33R -> SD_CLK | OK |
| 20 | IO25/SDIO_CMD | SD_CMD |  | SDIO CMD (R23 47k) | OK |
| 21 | IO36/STRAP | (NC) | STRAP  | not connected | strap OK: left open (module pull-up) |
| 22 | IO37/STRAP | JTAG_SEL | STRAP  | R14 10k to GND | strap OK: driven low by R14 (never floats) |
| 23 | IO38 | CAM_SIOD |  | SCCB SDA (R32 4.7k to DOVDD) | OK |
| 24 | IO39 | CAM_SIOC |  | SCCB SCL (R33 4.7k) | OK |
| 25 | IO40 | CAM_RESET |  | OV5640 RESETB (R34 10k pull-up) | OK |
| 26 | IO60/STRAP | ESP_IO60 | STRAP  | TP2 only (weak internal pull-up keeps it 1) | strap OK: only a test pad; internal pull-up = 1 |
| 27 | IO61/BOOT | ESP_BOOT | STRAP  | R13 10k pull-up, SW_BOOT1 to GND | strap OK: R13 pull-up = SPI boot; SW_BOOT1 = download |
| 28 | IO42 | CAM_VSYNC |  | DVP VSYNC | OK |
| 29 | IO43 | CAM_HREF |  | DVP HREF | OK |
| 30 | IO44 | CAM_D7 |  | DVP data D7 | OK |
| 31 | IO45 | CAM_D6 |  | DVP data D6 | OK |
| 32 | IO46 | CAM_PCLK |  | DVP PCLK (R57 33R at the FPC) | OK |
| 33 | IO47 | CAM_D0 |  | DVP data D0 | OK |
| 34 | IO54/MTDO | (NC) |  | not connected | OK: pad-JTAG pins free (JTAG via USB, IO37 low, eFuse default) |
| 35 | IO55/MTCK | (NC) |  | not connected | OK: pad-JTAG pins free (JTAG via USB, IO37 low, eFuse default) |
| 36 | RXD0/IO59 | (NC) |  | not connected | OK |
| 37 | TXD0/IO58 | (NC) |  | not connected | OK |
| 38 | IO56/MTDI | (NC) |  | not connected | OK: pad-JTAG pins free (JTAG via USB, IO37 low, eFuse default) |
| 39 | IO57/MTMS | (NC) |  | not connected | OK: pad-JTAG pins free (JTAG via USB, IO37 low, eFuse default) |
| 40 | GND | GND |  | power / ground |  |
| 41 | GND_EPAD | GND |  | power / ground |  |
| 42 | IO8 | SD_CD |  | microSD card detect (internal pull-up, low = card) | OK |
| 43 | IO9 | EN_FAN |  | TPS22919 U10 ON (FAN_VCC), R52 100k pull-down | OK: 100k pull-down holds the rail OFF from reset through boot and in deep sleep |
| 44 | IO10 | UART1_TX |  | radar RX via R60 1k | OK |
| 45 | IO11 | EN_CAM |  | TPS22919 U5 ON (+3V3_CAM), R11 100k pull-down | OK: 100k pull-down holds the rail OFF from reset through boot and in deep sleep |
| 46 | IO12 | HOLD |  | Rev C: self-latch output -> D_HOLD1 -> EN_TPS (TPS63802 EN); high = stay on | OK: not a strap, no pull at reset (EN_TPS has R_EN_PD1 100k); must be pad-held high in deep sleep - not an LP pin, verify digital-pad hold |
| 47 | IO13 | TMR_SENSE |  | Rev C: TMR3102 output (TMR_OUT) via R_TMR 470k/1M divider (input, internal pulls off) | OK: not a strap; not an LP pin (cannot wake on TMR_OUT falling, poll on wake) |
| 48 | IO14 | (NC) |  | not connected | OK |
| 49 | IO15 | FAN_PWM |  | LEDC PWM -> R45 100R -> AO3400A gate (R46 100k) | OK |
| 50 | IO16 | UART1_RX |  | radar TX via R59 1k | OK |
| 51 | IO17 | EN_SD |  | TPS22919 U4 ON (+3V3_SD), R10 100k pull-down | OK: 100k pull-down holds the rail OFF from reset through boot and in deep sleep |
| 52 | IO18 | CHG_STAT |  | BQ24075 /CHG (open drain, R4 100k) | OK |
| 53 | IO19 | CHG_PGOOD |  | BQ24075 /PGOOD (open drain, R5 100k) | OK |
| 54 | USB_DM | (NC) |  | not connected |  |
| 55 | USB_DP | (NC) |  | not connected |  |
| 56 | IO48 | CAM_XCLK_R |  | DVP XCLK out -> R22 33R | OK |
| 57 | IO49 | CAM_D5 |  | DVP data D5 | OK |
| 58 | IO50 | CAM_D4 |  | DVP data D4 | OK |
| 59 | IO51 | CAM_D3 |  | DVP data D3 | OK |
| 60 | IO52 | CAM_D1 |  | DVP data D1 | OK |
| 61 | IO53 | CAM_D2 |  | DVP data D2 | OK |
| 62 | GND | GND |  | power / ground |  |
| 63 | ANT2 | (NC) |  | not connected |  |
| 64 | GND | GND |  | power / ground |  |

GPIOs used: 46. Unconnected: IO36/STRAP (pin 21), IO54/MTDO (pin 34), IO55/MTCK (pin 35), RXD0/IO59 (pin 36), TXD0/IO58 (pin 37), IO56/MTDI (pin 38), IO57/MTMS (pin 39), IO14 (pin 48), USB_DM (pin 54), USB_DP (pin 55), ANT2 (pin 63).

## Conflict checks

* **Straps:** IO61 = 1 at reset (R13), IO60 = 1 (internal pull-up, only TP2 attached), IO37 = 0 (R14), IO36 open. No load-switch enable, wake source or
  high-speed bus sits on a strap pin, so no peripheral can force download mode or change VDD_SPI / JTAG source.
* **Wake sources (all on LP pins, EXT1):** RADAR_OUT IO2 (high), TAMPER_SENSE IO3 (high), SENS_INT IO5 (low). Mixed polarities: wake on
  RADAR_OUT/TAMPER_SENSE with EXT1 ANY_HIGH and use the LP core (or a second EXT1 group if supported) to poll SENS_INT; or invert in the LP core. Timed wake: internal LP timer.
* **Rail enables:** EN_SD IO17, EN_CAM IO11, EN_FAN IO9 - each has a 100k pull-down, TPS22919 VIH = 1 V, smart 530k internal pull-down:
  every switched rail is OFF from power-on through ROM boot and in deep sleep (pins go high-Z).
* **Charger status:** CHG_STAT IO18 and CHG_PGOOD IO19 are plain inputs with 100k pull-ups to +3V3_SYS (open-drain sources), no strap or JTAG function.
* **High-speed buses:** SDIO on the dedicated IO20-IO25 pins (no strap), DVP via the GPIO matrix on IO35/IO38-IO53 (no strap, no LP), USB on IO33/IO34.
* **Back-powering:** radar UART through 1k (R59/R60); keep SDIO and DVP low or high-Z while their rail is off.
* **Rev C chord power-on:** IO0 SENSE_L, IO4 SENSE_R (LP), IO12 HOLD, IO13 TMR_SENSE - all four were unconnected in Rev B; none is a strap or JTAG pin.
  VSYS-level signals reach the pins only through 0.68 dividers (3.57 V at VSYS 5.25 V). HOLD drives EN_TPS through a Schottky, so a 3.3 V high is ample (TPS63802 VIH 1.2 V).
* **Cost cut (RTC / watchdog / IR-cut):** no PCF85063A, TPL5010 or DRV8837. Freed pins IO55, IO56 are left unconnected (IO0 and IO4 are used again in Rev C). ESP_EN = R12 10k pull-up + C20 + SW_RST1 only.
  Use the internal RTC/LP timer (NTP re-sync) and the chip watchdogs (task WDT, RTC WDT) instead. No discrete ESD diodes on SD / radar lines (USBLC6 stays on USB).

