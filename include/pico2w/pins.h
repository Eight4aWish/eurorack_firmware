#ifndef PICO2W_PINS_H
#define PICO2W_PINS_H

// Pins / Addresses (extracted from src/pico2w/main.cpp)
#define PIN_BTN 6
#define PIN_POT1 28   // ADC0
#define PIN_POT2 27   // ADC1
#define PIN_POT3 26   // ADC2

#define I2C_SDA 18
#define I2C_SCL 19

#define I2C_ADDR_SSD1306 0x3C
#define I2C_ADDR_ADS     0x48   // change if you wired A0 differently
#define I2C_ADDR_MCP     0x60

// Display dimensions
#define OLED_W 128
#define OLED_H 64

// UI layout
// Height in pixels reserved for the top color/title band. Adjust this to move
// patch content below the yellow zone.
// Increase this value if patch text still overlaps the coloured top band.
#define UI_TOP_MARGIN 18

#endif // PICO2W_PINS_H
