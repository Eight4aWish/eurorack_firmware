# Eurorack Firmware — User Guide

This guide explains the UI navigation conventions and how to operate each patch on supported targets (Pico 2 W, Teensy 4.1, Daisy, etc.). The OLED UI uses a compact fixed grid optimized for yellow/blue split displays.

## UI Basics

- Home Menu: A grid of patch names. Short press the button to move the selection; long press to enter the selected patch.
- Return to Menu: From any patch, long press the button.
- Edit Mode: Many patches support parameter editing via short presses. A small underline indicates the currently selected parameter; turning Pot1 adjusts it.
- Pots: Unless stated otherwise, Pot1 adjusts the currently selected parameter; Pot2 may select modes; Pot3 commonly controls tempo/BPM.
- Cursor Grid: Text rows appear at Y positions `0, 14, 24, 34, 44, 54`.
- Title Bar: The top line shows the patch name. Some patches use the right side to display mode or status, e.g. Euclid shows `Simple/Complex`, Scope shows zoom as `Vx<H> H<samples>`.

## Navigation Summary

- Short Press (Menu): Move to next item.
- Long Press (Menu): Enter selected patch.
- Short Press (Patch): Cycle the selected parameter (and sometimes the selected channel).
- Long Press (Patch): Exit patch and return to the home menu.

## Patches

### Diag (Util → Diag)
- Purpose: Quick hardware diagnostics for pots, ADS inputs, and MCP outputs.
- Display:
  - Button state and raw ADC values for Pot1/2/3.
  - ADS readings (ADS0/ADS1).
  - MCP output codes D0–D3.
- Controls: Turn pots and observe readings; if DAC sweep is enabled in code, outputs will ramp.

### Clock
- Purpose: Lightweight clock with divisions.
- Display:
  - Mode: `INT` (internal) or `EXT` (external) depending on ADS threshold detection.
  - BPM or period derived from internal tempo or external clock.
  - Channel divisions mapped from Pot1/2/3.
- Controls:
  - Short press: Start/Stop.
  - Pot1: Tempo (INT mode) and division for CH0.
  - Pot2: Division for CH1.
  - Pot3: Division for CH2. CH3 mirrors CH0.

### Euclid
- Purpose: Euclidean drum triggers on up to 4 outputs.
- Modes (shown in title’s right side):
  - Simple: Shared `Steps`, `Pulses`, `Rotation`, `BPM` for all outputs.
  - Complex: Per-channel `Steps`, `Pulses`, `Rotation`, with global `BPM`.
- Selection:
  - Pot2 selects the mode: <50% → Simple, ≥50% → Complex.
- Edit Mode (both variants):
  - Short press: Cycles the selected parameter. In Complex mode, when the cycle wraps to `Steps`, the channel selection advances (CH0→CH1→CH2→CH3).
  - Underline: Indicates the currently selected parameter (and channel in Complex).
  - Pot1: Adjusts the selected parameter.
  - Pot3: BPM (30–300). If BPM is selected, Pot1 adjusts it too.
- Ranges:
  - Steps: 1–16
  - Pulses: 0–Steps
  - Rotation: 0–Steps-1
  - BPM: 30–300

### Scope
- Purpose: Simple oscilloscope view for ADS input 0.
- Display: A scrolling waveform with a midline, spanning from y=14 to the bottom (full area beneath the title). The title’s right side shows zoom status, e.g. `Vx1.2 H96`.
- Controls:
  - Pot1: Vertical zoom (gain) from ~0.25x to 4x.
  - Pot2: Horizontal window (visible samples) from 32 to 128.
  - Connect a signal to ADS channel 0 and observe.

### Placeholders (Quant, Mod, Osc, Calib)
- Purpose: Reserved for future patches.
- Display: Single title line below the top margin.
- Controls: None yet.

## Tips

- External Clock: Ensure ADS channel 0 wiring and thresholds are suitable for your clock source.
- OLED Split: The Y grid avoids the color split band; if you change fonts or sizes, keep titles at `y=0` and main content starting at `y=24`.
- Uploading: Use PlatformIO commands (see project README) to build and upload for your target.

## PlatformIO Quick Commands

```sh
# Build default environment
pio run

# Build Pico 2 W explicitly
pio run -e pico2w

# Upload to Pico 2 W (picotool)
pio run -e pico2w -t upload

# Monitor serial at 115200
pio device monitor -b 115200
```

## Contributing

- When adding new patches, follow the grid and edit-mode conventions.
- Keep per-patch UI concise; reserve the title’s right side for brief status or mode.
