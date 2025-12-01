# Eurorack Firmware — User Guide

This guide covers navigation, controls, and behavior for the current functional patches on Pico 2 W. The OLED UI uses a compact fixed grid optimized for split (yellow/blue) SSD1306 displays.

## UI Basics

- Home Menu: Short press cycles selection; long press enters the highlighted patch.
- In-Patch: Short press performs the patch’s action (e.g., run/stop, cycle parameter, toggle edit target). Long press returns to the menu.
- Pots: Pot1, Pot2, Pot3 are read inverted so clockwise increases value. Some patches smooth pots to reduce jitter.
- Layout Grid: Title at `y=0`. Content rows at `y=16, 26, 36, 46, 56`.
- Title Right: Some patches show mode/status on the right side of the title line.

## Navigation Summary

- Menu: short = next, long = enter.
- Patch: short = patch-specific action, long = back to menu.

## Patches

### Diag
- Purpose: Hardware diagnostics for pots, ADS1115 inputs, and MCP4728 outputs.
- Display:
  - Button state and raw reads for Pot1/2/3.
  - ADS raw codes for ADC0/ADC1.
  - MCP codes for physical CV0..CV3 (mapped via `include/pico2w/pins.h`).
- Controls:
  - Short press: Cycle selected physical CV output (CV0→CV1→CV2→CV3).
  - Pot1: Sets the DAC code (0..4095) for the selected CV only; other CVs are set to 0.
  - Pot2/Pot3: No effect.

### Clock
- Purpose: Lightweight 4-channel clock with divisions and optional external clocking.
- Outputs: 10 ms gates on CV0..CV3 using fixed “gate codes” (about 0 V at code ~2047, +5 V at code 0), intentionally ignoring calibration.
- Display: `INT/EXT`, BPM, and `RUN/STOP`; per-channel division labels for CH0..CH3.
- Controls:
  - Short press: Toggle RUN/STOP.
  - Pot1: BPM (INT mode: 30–300 BPM). If external clock edges are detected on `AD_EXT_CLOCK_CH`, the clock follows the external period.
  - Pot2: Division for CH0; CH1 is derived at half speed of CH0.
  - Pot3: Division for CH2; CH3 is derived at half speed of CH2.

### Euclid
- Purpose: Euclidean drum triggers on up to 4 outputs.
- Outputs: 30 ms gates on CV0..CV3 (fixed gate codes as above).
- Modes (shown on title right):
  - Simple: Shared Steps/Pulses/Rotation; 4 channels are rotated variants.
  - Complex: Per-channel Steps/Pulses/Rotation.
- Controls:
  - Pot1: BPM (30–300).
  - Pot2: Mode select — <50%: Simple, ≥50%: Complex.
  - Pot3: Adjusts the currently selected parameter (Steps/Pulses/Rotation).
  - Short press: Cycles the selected parameter. In Complex mode, when the cycle wraps, the selected channel advances (CH0→CH1→CH2→CH3).

### Env (Dual Envelopes)
- Purpose: Two independent macro ADSR-style envelopes.
- Outputs: Env1→CV0, Env2→CV1. 0 V baseline at code ~2047, up to ~+5 V at code 0. Each envelope has its own velocity scaling.
- Triggers: Env1 from `AD_EXT_CLOCK_CH`; Env2 from `AD1_CH` (rising-edge detection).
- Controls (smoothed):
  - Pot1: Velocity (amplitude) for the selected envelope.
  - Pot2: AD macro — Attack and Decay are linked; turning clockwise lengthens both. Short settings are punchy.
  - Pot3: S/R macro — Sustain level and Release time together; higher values raise sustain and lengthen release.
  - Short press: Toggle which envelope is edited (E1/E2). Header shows the active target; first row displays `Vel <n>%` for the selected envelope.
- Envelope Model:
  - Attack(ms) = 1 + (AD²) × 2000.
  - Decay(ms) = Attack × (0.15 + 0.85 × Sustain) for percussive response.
  - Release(ms) = 1 + (SR²) × 2000.
  - Output = Level × Velocity, mapped to DAC codes for ~0..+5 V.

### Quant
- Purpose: 2-channel semitone quantizer (1 V/oct).
- Inputs: ADS `AD0_CH`, `AD1_CH` (mapped to volts via calibration if available).
- Outputs: CV0, CV1 (mapped back to DAC codes via calibration).
- Display: In0/Out0 and In1/Out1 values plus current DAC codes.
- Controls: None (always-on behavior).

### Scope
- Purpose: Simple oscilloscope for ADS channel 0.
- Display: Waveform trace under the title; title-right shows `Vx<gain> H<samples> M<mid>`.
- Controls:
  - Pot1: Vertical gain (~0.25x to 4x).
  - Pot2: Horizontal window (32..128 samples).
  - Pot3: Midpoint offset.

### Calib
- Purpose: Guided calibration for ADS bias/gain and DAC offset/scale. Values persist in EEPROM.
- Steps (short press to advance):
  1) ADS ch0: set −4 V, capture.
  2) ADS ch0: set +4 V, capture (gain computed from ±4 V span).
  3) DAC: set −4 V, capture codes for all channels.
  4) DAC: set +4 V, capture; compute codes-per-volt for each channel.
  5) Save & summary. Long press to return to menu.

### LFO (Placeholder)
- Purpose: Reserved; UI placeholder only.
- Controls/Output: None yet.

## Tips

- Physical Mapping: DAC channels use physical macros `CV0_DA_CH..CV3_DA_CH`; ADS channels use `AD0_CH`, `AD1_CH`, and `AD_EXT_CLOCK_CH` in `include/pico2w/pins.h`.
- External Clocking: Provide clean rising edges into `AD_EXT_CLOCK_CH` for reliable detection.
- OLED Grid: Keep titles at `y=0`; use rows `16/26/36/46/56` for content.

## PlatformIO Quick Commands

```sh
# Build default environment
pio run

# Build Pico 2 W explicitly
pio run -e pico2w

# Upload to Pico 2 W
pio run -e pico2w -t upload

# Monitor serial at 115200
pio device monitor -b 115200
```

## Contributing

- Keep per-patch UIs compact and consistent with the grid.
- Use physical channel macros for all I/O; avoid legacy aliases.
