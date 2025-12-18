# Eurorack Firmware Monorepo

## User Guide

See `docs/USER_GUIDE.md` for UI navigation and per-patch operation details (home menu, edit mode, Euclid modes, and controls).

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
