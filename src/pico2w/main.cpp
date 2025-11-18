#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP4728.h>
#include <Bounce2.h>

#include "eurorack_ui/OledHelpers.hpp"
#include "eurorack_ui/OledHomeMenu.hpp"

// -------------------- Pins / Addresses (moved to header) --------------------
#include "pico2w/pins.h"

// -------------------- OLED --------------------
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire1, -1);

// -------------------- Devices --------------------
Adafruit_ADS1115  ads;    // pass Wire1 in begin()
Adafruit_MCP4728  mcp;    // pass Wire1 in begin()

// -------------------- Button --------------------
Bounce btn;

// -------------------- Timing --------------------
static uint32_t lastUiMs = 0;
static uint32_t lastTickMs = 0;
#define UI_FRAME_MS_ACTIVE  50
#define CTRL_TICK_MS         5

// -------------------- Calibration-ish --------------------
// ADS1115 scale via library's computeVolts()
static float cvBias = 1.65f;
static float cvGain = (10.0f / 3.3f);  // maps 0..3.3V around 1.65V to ±5V

// MCP4728 full-scale volts assumption for display only.
// With default Vref=VDD (likely 3.3V) and Gain=1x, FS ≈ 3.3V.
// If you later switch MCP4728 to internal 2.048V + 2x gain, set to 4.096f.
static float mcpVrefFull = 3.3f;
static float mcpVoltsPerCode = (mcpVrefFull / 4095.0f);
// Eurorack output stage scaling (your analog stage 0..Vref -> ±5V)
static float dacToEurorackGain = (10.0f / mcpVrefFull);

// -------------------- State --------------------
struct Patch {
  const char* name;
  void (*enter)();
  void (*tick)();
  void (*render)();
};
struct Bank {
  const char* name;
  Patch* patches[8];
  uint8_t patchCount;
};

static bool haveSSD = false, haveADS = false, haveMCP = false;
static bool dacSweep = false;
static uint16_t dacCode = 0;
static int dacDir = +1;
static float pot1 = 0, pot2 = 0, pot3 = 0;
static float adc0V = 0, adc1V = 0;
static float cv0V  = 0, cv1V  = 0;

// -------------------- Helpers --------------------
float readPotNorm(int pin) {
  int v = analogRead(pin);          // 12-bit after setResolution(12)
  return constrain(v / 4095.0f, 0.0f, 1.0f);
}

void i2cScan(bool &ssd, bool &adsOK, bool &mcpOK) {
  ssd = adsOK = mcpOK = false;
  for (uint8_t addr : {I2C_ADDR_SSD1306, I2C_ADDR_ADS, I2C_ADDR_MCP}) {
    Wire1.beginTransmission(addr);
    uint8_t err = Wire1.endTransmission();
    if (addr == I2C_ADDR_SSD1306 && err == 0) ssd = true;
    if (addr == I2C_ADDR_ADS     && err == 0) adsOK = true;
    if (addr == I2C_ADDR_MCP     && err == 0) mcpOK = true;
  }
}

void drawBar(int x, int y, int w, int h, float norm) {
  eurorack_ui::drawBar(oled, x, y, w, h, norm, false);
}

// Small local UI forwarding helpers to keep call sites concise
namespace ui {
static void printClipped(int x, int y, int w, const char* s) { eurorack_ui::printClipped(oled, x, y, w, s); }
static void printClippedBold(int x, int y, int w, const char* s, bool bold) { eurorack_ui::printClippedBold(oled, x, y, w, s, bold); }
static void printLabelOnly(int x, int y, int w, const char* label) { eurorack_ui::printLabelOnly(oled, x, y, w, label); }
static void drawBarF(int x, int y, int w, int h, float norm) { eurorack_ui::drawBar(oled, x, y, w, h, norm, false); }
}

// -------------------- Patch: Util/Diag --------------------
void diag_enter() {}

void diag_tick() {
  // Pots
  pot1 = readPotNorm(PIN_POT1);
  pot2 = readPotNorm(PIN_POT2);
  pot3 = readPotNorm(PIN_POT3);

  // ADS1115: read A0 and A1 single-ended
  if (haveADS) {
    int16_t a0 = ads.readADC_SingleEnded(0);
    int16_t a1 = ads.readADC_SingleEnded(1);
    adc0V = ads.computeVolts(a0);
    adc1V = ads.computeVolts(a1);
    cv0V  = (adc0V - cvBias) * cvGain;
    cv1V  = (adc1V - cvBias) * cvGain;
  } else {
    adc0V = adc1V = cv0V = cv1V = NAN;
  }

  // DAC: either sweep or set via pot1
  if (haveMCP) {
    if (dacSweep) {
      if (dacDir > 0) { if (dacCode < 4095) dacCode += 8; else dacDir = -1; }
      else            { if (dacCode >    0) dacCode -= 8; else dacDir = +1; }
    } else {
      dacCode = (uint16_t)(pot1 * 4095.0f);
    }
    // Write same code to all 4 channels in one go
    mcp.fastWrite(dacCode, dacCode, dacCode, dacCode);
  }
}

void diag_render() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  ui::printClipped(0, 0, 64, "Util:Diag");
  oled.setCursor(66, 0);
  oled.print(haveSSD ? "OLED " : "noOLED ");
  oled.print(haveADS ? "ADS "  : "noADS ");
  oled.print(haveMCP ? "MCP"   : "noMCP");

  // Pots
  drawBar(0, 12, 40, 8, pot1);
  drawBar(44,12, 40, 8, pot2);
  drawBar(88,12, 40, 8, pot3);
  ui::printLabelOnly(0, 22, 20, "P1"); oled.print((int)(pot1*100)); oled.print("%  ");
  ui::printLabelOnly(40, 22, 20, "P2"); oled.print((int)(pot2*100)); oled.print("%  ");
  ui::printLabelOnly(80, 22, 20, "P3"); oled.print((int)(pot3*100)); oled.print("%");

  // CV inputs raw
  ui::printLabelOnly(0, 34, 36, "ADC0");
  if (isnan(adc0V)) oled.print("--"); else { oled.print(adc0V, 3); oled.print("V"); }
  ui::printLabelOnly(66, 34, 36, "ADC1");
  if (isnan(adc1V)) oled.print("--"); else { oled.print(adc1V, 3); oled.print("V"); }

  // Reconstructed ±5V
  ui::printLabelOnly(0, 44, 36, "CV0");
  if (isnan(cv0V)) oled.print("--"); else { oled.print(cv0V, 2); oled.print("V"); }
  ui::printLabelOnly(66, 44, 36, "CV1");
  if (isnan(cv1V)) oled.print("--"); else { oled.print(cv1V, 2); oled.print("V"); }

  // DAC status
  float dacV = dacCode * mcpVoltsPerCode;
  float eurV = dacV * dacToEurorackGain;
  ui::printClipped(0, 54, 64, "DAC A..D code"); oled.print(dacCode);
  oled.setCursor(96, 54); oled.print("~"); oled.print(eurV, 2); oled.print("V");

  oled.display();
}

// -------------------- Registry --------------------
Patch patch_diag = { "Diag", diag_enter, diag_tick, diag_render };
Bank bank_util = { "Util", { &patch_diag, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr }, 1 };
Bank* banks[] = { &bank_util };
static uint8_t bankIdx  = 0;
static uint8_t patchIdx = 0;

// ---- Home menu + input state ----
static const char* kHomeItems[] = { "Diag", "Quant", "Clock", "Scope" };
static eurorack_ui::OledHomeMenu homeMenu;
static bool homeMenuActive = true;
static volatile bool patchShortPressed = false;
static int activePlaceholder = -1; // -1 = none; 0 reserved for Diag
static uint32_t menuIgnoreUntil = 0;

// -------------------- Input --------------------
void handleButtons() {
  btn.update();
  static uint32_t downAt = 0;
  if (btn.fell())  downAt = millis();
  if (btn.rose()) {
    uint32_t held = millis() - downAt;
    // If home menu active: short -> next, long -> select
    if (homeMenuActive) {
      // ignore spurious releases immediately after entering menu
      if (millis() < menuIgnoreUntil) return;
      if (held <= 600) {
        // short press -> next
        homeMenu.next();
      } else {
        // long press -> select current
        uint8_t sel = homeMenu.commit();
        if (sel == 0) {
          // Diag: activate patch 0 in bank 0
          homeMenuActive = false;
          activePlaceholder = -1;
          bankIdx = 0; patchIdx = 0;
          Patch* p = banks[bankIdx]->patches[patchIdx];
          if (p && p->enter) p->enter();
        } else {
          // Placeholder screens for other items: remember which placeholder is active
          activePlaceholder = sel; // 1..N
          homeMenuActive = false;
          // draw immediately (single-word placeholder at top)
          oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
          ui::printClipped(0, 0, OLED_W, kHomeItems[sel]);
          oled.display();
        }
      }
    } else {
      // not in menu: short press -> patch-specific action flag; long press -> return to menu
      if (held <= 600) {
        patchShortPressed = true;
      } else {
        // enter menu from patch: reset placeholder, clear patch short flag, force redraw and ignore spurious inputs
        homeMenuActive = true;
        activePlaceholder = -1;
        patchShortPressed = false;
        menuIgnoreUntil = millis() + 400;
        lastUiMs = 0; // force next UI tick to redraw immediately
        homeMenu.invalidate();
        homeMenu.draw();
        Serial.print("[UI] Returned to menu from patch\n");
      }
    }
  }
}

// -------------------- Setup / Loop --------------------
void setup() {
  // Serial optional
  Serial.begin(115200);
  delay(50);

  pinMode(PIN_BTN, INPUT_PULLUP);
  btn.attach(PIN_BTN);
  btn.interval(5);

  analogReadResolution(12); // 0..4095

  Wire1.setSDA(I2C_SDA);
  Wire1.setSCL(I2C_SCL);
  Wire1.begin();
  Wire1.setClock(400000);

  // Quick I2C presence check
  i2cScan(haveSSD, haveADS, haveMCP);

  // OLED
  if (haveSSD) {
    haveSSD = oled.begin(SSD1306_SWITCHCAPVCC, I2C_ADDR_SSD1306);
    if (haveSSD) {
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setTextColor(SSD1306_WHITE);
      ui::printClipped(0, 0, OLED_W, "Pico2W Util/Diag");
      oled.display();
      delay(200);
    }
  }

  // Home menu init (use shared menu)
  if (haveSSD) {
    homeMenu.begin(&oled, 0);
    homeMenu.setItems(kHomeItems, (uint8_t)(sizeof(kHomeItems)/sizeof(kHomeItems[0])));
    homeMenuActive = true;
    homeMenu.draw();
    menuIgnoreUntil = millis() + 400;
  }

  // ADS1115
  if (haveADS) {
    // begin(address, wirePort)
    haveADS = ads.begin(I2C_ADDR_ADS, &Wire1);
    if (haveADS) {
      ads.setGain(GAIN_ONE);                 // ±4.096V FSR
      ads.setDataRate(RATE_ADS1115_860SPS);  // fastest
    }
  }

  // MCP4728
  if (haveMCP) {
    haveMCP = mcp.begin(I2C_ADDR_MCP, &Wire1); // uses default addr 0x60 by itself
    // No per-channel config calls here—defaults are fine for bring-up
  }
}

void loop() {
  handleButtons();

  uint32_t now = millis();

  if (now - lastTickMs >= CTRL_TICK_MS) {
    lastTickMs = now;
    Patch* p = banks[bankIdx]->patches[patchIdx];
    if (p && p->tick) p->tick();
  }

  if (haveSSD && (now - lastUiMs >= UI_FRAME_MS_ACTIVE)) {
    lastUiMs = now;
    if (homeMenuActive) {
      homeMenu.draw();
    } else if (activePlaceholder >= 1) {
      // show placeholder single-word screen (already drawn on select, but keep it)
      // draw minimal heartbeat so screen doesn't get overwritten
      ;
    } else {
      Patch* p = banks[bankIdx]->patches[patchIdx];
      if (p && p->render) p->render();
    }
  }
}
