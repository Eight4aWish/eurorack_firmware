# Eurorack Firmware Monorepo

## User Guide

See `docs/PICO2W.md` for Pico 2 W UI navigation and per-patch details. See `docs/TEENSY_MOVE.md` for Teensy-specific wiring and behavior.

## Build

PlatformIO is used for building across targets. Pico 2 W is the default.

```sh
# Build default env
pio run

# Build & upload Pico 2 W
pio run -e pico2w -t upload

# Monitor serial
pio device monitor -b 115200
```

## Teensy 4.1 — `teensy-move`

This target implements a modular synth controller on Teensy 4.1 with two on-board channels and two expander channels driven via a 74HCT595. USB runs in composite mode (Audio + MIDI + Serial).

- Features:
	- USB Audio passthrough: I2S input routed to both I2S out and USB out.
	- MIDI: Channels 1–4 produce gates + Mod/Pitch CV; Channel 10 notes 36–39 produce drum triggers.
	- MIDI clock: 24 PPQN divided to quarter-note pulses; Start emits a short Reset pulse.
	- OLED: Status lines for CLK/G1/G2/R, P1/P2 and M1/M2; last MIDI event overlay or drums row.

- On-board channels (1–2):
	- Gates: `PIN_GATE1=40`, `PIN_GATE2=38` (through 74HCT14; firmware writes inverted via `GATE_WRITE`).
	- DACs: Two MCP4822 chips, chip-selects on `PIN_CS_DAC1=33`, `PIN_CS_DAC2=34`.
	- Calibration: `kPitchSlope/kPitchOffset`, `kModSlope/kModOffset` map target volts to DAC codes (2× gain enabled).

- Expander channels (3–4) via 74HCT595:
	- Latch: `PIN_595_LATCH=32`. Shared SPI bus `MOSI=11`, `SCK=13`.
	- Bit map: Q1→Gate1 (ch3), Q0→Gate2 (ch4), Q2–Q5→Drum1–4, Q6→Mod DAC CS, Q7→Pitch DAC CS.
	- Gates: LOW at the 595 asserts gates at the jacks (through HCT14 inverters).
	- DAC mapping: Q6 (active-low CS) selects Mod DAC (A=Mod3, B=Mod4); Q7 selects Pitch DAC (A=Pitch3, B=Pitch4).

- Drum triggers:
	- MIDI Channel 10, notes 36..39 (C1..D#1) → Q2..Q5; ~15 ms pulses.

- Build & Upload:

```sh
# Teensy 4.1 (teensy-move)
pio run -e teensy41
pio run -e teensy41 -t upload
```

See `docs/TEENSY_MOVE.md` for wiring notes, expander sequencing, and behavior details.

## Pico 2 W — `pico2w`

This target implements a menu-driven multi-patch Eurorack utility on Raspberry Pi Pico 2 W with an SSD1306 OLED, ADS1115 ADC inputs, and an MCP4728 quad DAC for CV outputs.

- Features:
	- OLED UI with short/long press navigation (menu and in-patch controls).
	- Patches: Clock, Quant, Euclid, Env (dual envelopes), QuadLFO, Scope, Calib, Diag.
	- Inputs: Two analog inputs via ADS1115 plus an external clock input (`AD_EXT_CLOCK_CH`).
	- Outputs: Four CVs via MCP4728 (calibrated mapping for bipolar/unipolar where applicable). Timing patches use fixed gate codes for crisp edges.
	- Consistent grid-based UI layout for readability on 128x64 OLED.

- Hardware mapping:
	- See `include/pico2w/pins.h` for physical macros: `CV0_DA_CH..CV3_DA_CH`, `AD0_CH`, `AD1_CH`, `AD_EXT_CLOCK_CH`.
	- External clock is detected on rising edges on `AD_EXT_CLOCK_CH` in Clock/Env patches.

- Build & Upload:

```sh
# Pico 2 W
pio run -e pico2w
pio run -e pico2w -t upload
pio device monitor -b 115200
```

See `docs/PICO2W.md` for full UI behavior and patch-specific controls.
