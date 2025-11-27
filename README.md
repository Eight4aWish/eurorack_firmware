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
