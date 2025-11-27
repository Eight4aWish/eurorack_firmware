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
static int16_t ads_raw0 = 0, ads_raw1 = 0;
static uint16_t mcp_values[4] = {0, 0, 0, 0};
static volatile bool patchShortPressed = false;

// Normalized pot read (inverted so clockwise increases value)
float readPotNorm(int pin) {
  int v = analogRead(pin);
  return constrain(1.0f - (v / 4095.0f), 0.0f, 1.0f);
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
    ads_raw0 = a0;
    ads_raw1 = a1;
    adc0V = ads.computeVolts(a0);
    adc1V = ads.computeVolts(a1);
    cv0V  = (adc0V - cvBias) * cvGain;
    cv1V  = (adc1V - cvBias) * cvGain;
  } else {
    adc0V = adc1V = cv0V = cv1V = NAN;
    ads_raw0 = ads_raw1 = 0;
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
    // Remember what we wrote for the diag display
    mcp_values[0] = dacCode; mcp_values[1] = dacCode;
    mcp_values[2] = dacCode; mcp_values[3] = dacCode;
  }
}

void diag_render() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  // Disable wrap so numeric prints don't bleed into next lines
  oled.setTextWrap(false);
  ui::printClipped(0, 0, 64, "Diag");
  // Keep O/A/M status visible on Diag screen
  oled.setCursor(66, 0);
  oled.print(haveSSD ? "O" : "-"); oled.print(' ');
  oled.print(haveADS ? "A" : "-"); oled.print(' ');
  oled.print(haveMCP ? "M" : "-");

  // Button + Pots (show raw ADC values)
  oled.setCursor(0, 14);
  oled.print("BTN "); oled.print(btn.read() == LOW ? "DOWN" : "UP  ");

  int raw1 = analogRead(PIN_POT1);
  int raw2 = analogRead(PIN_POT2);
  int raw3 = analogRead(PIN_POT3);
  // Line: Btn <status>    P1 <raw>
  oled.setCursor(64, 14); oled.print("P1 "); oled.print(raw1);
  // Next line: P2 <raw>    P3 <raw>
  oled.setCursor(0, 24); oled.print("P2 "); oled.print(raw2);
  oled.setCursor(64, 24); oled.print("P3 "); oled.print(raw3);

  // CV inputs raw
  // ADS1115 raw codes (16-bit signed; positive max ~= 32767)
  // Show the ADC raw readings captured in diag_tick()
  oled.setCursor(0, 34); oled.print("A0 "); oled.print(ads_raw0);
  oled.setCursor(64, 34); oled.print("A1 "); oled.print(ads_raw1);

  // Show MCP/DAC outputs (codes) for channels 0..3
  oled.setCursor(0, 44); oled.print("D0 "); oled.print(mcp_values[0]);
  oled.setCursor(64, 44); oled.print("D1 "); oled.print(mcp_values[1]);
  oled.setCursor(0, 54); oled.print("D2 "); oled.print(mcp_values[2]);
  oled.setCursor(64, 54); oled.print("D3 "); oled.print(mcp_values[3]);

  oled.display();
}

// -------------------- Registry --------------------
Patch patch_diag = { "Diag", diag_enter, diag_tick, diag_render };
// -------------------- Patch: Clock (lightweight) --------------------
static bool clock_running = false;
static uint32_t clock_last_internal_ms = 0;
static uint32_t clock_last_external_edge_ms = 0;
static uint32_t clock_base_interval_ms = 500; // default 120 BPM -> 500ms per beat
static uint32_t clock_ext_interval_ms = 0;
static int ads_prev0 = 0;
// division / multiplication options
static const char* div_labels[] = { "1/4", "1/3", "1/2", "1", "x2", "x3", "x4" };
static const float div_factors[] = { 0.25f, 0.3333333f, 0.5f, 1.0f, 2.0f, 3.0f, 4.0f };
static const int kDivCount = sizeof(div_factors) / sizeof(div_factors[0]);
// per-channel scheduling
static uint32_t ch_next_fire_ms[4] = {0,0,0,0};
static uint32_t ch_pulse_end_ms[4] = {0,0,0,0};
static bool ch_state[4] = {false,false,false,false};

void clock_enter() {
  clock_running = false;
  clock_last_internal_ms = millis();
  clock_last_external_edge_ms = 0;
  clock_ext_interval_ms = 0;
  ads_prev0 = 0;
  for (int i=0;i<4;i++) { ch_next_fire_ms[i]=0; ch_pulse_end_ms[i]=0; ch_state[i]=false; }
}

void clock_tick() {
  // handle start/stop short-press from main handler
  if (patchShortPressed) {
    clock_running = !clock_running;
    patchShortPressed = false;
  }

  // read pots (12-bit)
  int pot_raw[3];
  pot_raw[0] = 4095 - analogRead(PIN_POT1);
  pot_raw[1] = 4095 - analogRead(PIN_POT2);
  pot_raw[2] = 4095 - analogRead(PIN_POT3);

  // map pots to division indices
  int div_idx[3];
  for (int i=0;i<3;i++) div_idx[i] = (int)((uint32_t)pot_raw[i] * kDivCount / 4096);
  for (int i=0;i<3;i++) if (div_idx[i] < 0) div_idx[i]=0; else if (div_idx[i]>=kDivCount) div_idx[i]=kDivCount-1;

  uint32_t now = millis();

  // detect external clock on ADS channel 0 (if present)
  bool have_ext = false;
  if (haveADS) {
    int a0 = ads.readADC_SingleEnded(0);
    // simple threshold edge detector: rising when value increases by > delta
    const int delta = 4000; // heuristic for 16-bit readings
    if (ads_prev0 && a0 - ads_prev0 > delta) {
      // rising edge
      if (clock_last_external_edge_ms) {
        clock_ext_interval_ms = now - clock_last_external_edge_ms;
      }
      clock_last_external_edge_ms = now;
      have_ext = true;
    }
    ads_prev0 = a0;
  }

  // base interval: external if present, else internal from pot1 mapped to BPM
  if (have_ext && clock_ext_interval_ms > 0) {
    clock_base_interval_ms = clock_ext_interval_ms;
  } else {
    // use pot1 to set tempo 30..300 BPM
    int p = pot_raw[0];
    int bpm = 30 + (int)(((long)p * (300-30)) / 4095);
    if (bpm <= 0) bpm = 120;
    clock_base_interval_ms = 60000 / bpm;
  }

  // schedule per-channel events based on division factors
  for (int ch=0; ch<3; ch++) {
    float factor = div_factors[div_idx[ch]];
    // compute channel interval (ms)
    uint32_t ch_interval_ms = (uint32_t)(clock_base_interval_ms / factor);
    if (ch_interval_ms == 0) ch_interval_ms = 1;

    if (!clock_running) continue;

    if (now >= ch_next_fire_ms[ch]) {
      // start a short pulse
      ch_state[ch] = true;
      ch_pulse_end_ms[ch] = now + 10; // 10 ms pulse
      ch_next_fire_ms[ch] = now + ch_interval_ms;
    }
    if (ch_state[ch] && now >= ch_pulse_end_ms[ch]) {
      ch_state[ch] = false;
    }
  }

  // channel 3 mirror channel 0 (optional)
  ch_state[3] = ch_state[0];

  // write MCP outputs if available
  if (haveMCP) {
    uint16_t out0 = ch_state[0] ? 4095 : 0;
    uint16_t out1 = ch_state[1] ? 4095 : 0;
    uint16_t out2 = ch_state[2] ? 4095 : 0;
    uint16_t out3 = ch_state[3] ? 4095 : 0;
    // remember values for display
    mcp_values[0] = out0; mcp_values[1] = out1; mcp_values[2] = out2; mcp_values[3] = out3;
    // write all channels at once
    mcp.fastWrite(out0, out1, out2, out3);
  }
}

void clock_render() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextWrap(false);
  ui::printClipped(0, 0, 64, "Clock");
  // status area unused on patches per request

  // Mode and running state
  oled.setCursor(0, 14);
  oled.print("Mode: ");
  if (clock_ext_interval_ms > 0) oled.print("EXT"); else oled.print("INT");
  oled.setCursor(64,14); oled.print(clock_running ? "RUN" : "STOP");

  // Tempo / period
  oled.setCursor(0, 24);
  if (clock_ext_interval_ms > 0) {
    // show BPM computed from ext interval
    float bpm = 60000.0f / (float)clock_ext_interval_ms;
    oled.print("BPM "); oled.print((int)(bpm+0.5f));
  } else {
    float bpm = 60000.0f / (float)clock_base_interval_ms;
    oled.print("BPM "); oled.print((int)(bpm+0.5f));
  }

  // Divisions for pots
  int raw1 = 4095 - analogRead(PIN_POT1);
  int raw2 = 4095 - analogRead(PIN_POT2);
  int raw3 = 4095 - analogRead(PIN_POT3);
  int idx1 = (int)((uint32_t)raw1 * kDivCount / 4096); if (idx1<0) idx1=0; else if (idx1>=kDivCount) idx1=kDivCount-1;
  int idx2 = (int)((uint32_t)raw2 * kDivCount / 4096); if (idx2<0) idx2=0; else if (idx2>=kDivCount) idx2=kDivCount-1;
  int idx3 = (int)((uint32_t)raw3 * kDivCount / 4096); if (idx3<0) idx3=0; else if (idx3>=kDivCount) idx3=kDivCount-1;

  oled.setCursor(0, 34); oled.print("CH0 "); oled.print(div_labels[idx1]); oled.setCursor(64,34); oled.print("CH1 "); oled.print(div_labels[idx2]);
  oled.setCursor(0, 44); oled.print("CH2 "); oled.print(div_labels[idx3]); oled.setCursor(64,44); oled.print("CH3 "); oled.print(div_labels[idx1]);

  // Show current DAC codes for debugging
  oled.setCursor(0, 54); oled.print("D0 "); oled.print(mcp_values[0]); oled.setCursor(64,54); oled.print("D1 "); oled.print(mcp_values[1]);

  oled.display();
}

Patch patch_clock = { "Clock", clock_enter, clock_tick, clock_render };
// -- Placeholder patch stubs for menu entries (lightweight)
void quant_enter() {}
void quant_tick() {}
void quant_render() {
  oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
  ui::printClipped(0, UI_TOP_MARGIN, OLED_W, "Quant");
  oled.display();
}
Patch patch_quant = { "Quant", quant_enter, quant_tick, quant_render };

// ---- Euclid patch: Euclidean drum triggers on up to 4 MCP outputs ----
static int euclid_steps = 8;
static int euclid_pulses = 3;
static int euclid_rotation = 0;
static int euclid_step_idx = 0;
static uint32_t euclid_next_ms = 0;
static uint32_t euclid_pulse_end_ms[4] = {0,0,0,0};
static bool euclid_state[4] = {false,false,false,false};
static bool euclid_patterns[4][16]; // max 16 steps
// Euclid edit mode: cycle parameters with short press; adjust selected via pot1
static bool euclid_edit_mode = true;
static int euclid_selected_param = 0; // 0=Steps,1=Pulses,2=Rotation,3=BPM
// Euclid modes: 0=simple (shared params), 1=complex (per-channel params)
static int euclid_mode = 0;
static int euclid_ch_steps[4] = {8,8,8,8};
static int euclid_ch_pulses[4] = {3,3,3,3};
static int euclid_ch_rotation[4] = {0,1,2,3};
// Selection in complex mode: (channel, param)
static int euclid_sel_channel = 0; // 0..3

void build_euclid_pattern(bool *out, int steps, int pulses, int rotate) {
  // simple even-distribution algorithm
  int acc = 0;
  for (int i = 0; i < steps; i++) {
    acc += pulses;
    if (acc >= steps) {
      out[i] = true;
      acc -= steps;
    } else out[i] = false;
  }
  // rotate
  if (rotate != 0) {
    bool tmp[16];
    for (int i=0;i<steps;i++) tmp[(i+rotate)%steps] = out[i];
    for (int i=0;i<steps;i++) out[i] = tmp[i];
  }
}

void euclid_enter() {
  euclid_steps = 8; euclid_pulses = 3; euclid_rotation = 0; euclid_step_idx = 0; euclid_next_ms = millis();
  for (int c=0;c<4;c++) { euclid_pulse_end_ms[c]=0; euclid_state[c]=false; }
}

void euclid_tick() {
  // read pots
  int raw1 = analogRead(PIN_POT1);
  int raw2 = analogRead(PIN_POT2);
  int raw3 = analogRead(PIN_POT3);
  // Mode selection via Pot2 (coarse split): <50% simple, >=50% complex
  euclid_mode = (raw2 >= 2048) ? 1 : 0;

  int steps = euclid_steps;
  int pulses = euclid_pulses;
  int bpm = 30 + (int)(((long)raw3 * (300-30)) / 4095);

  if (euclid_mode == 0) {
    // Simple mode: shared params
    if (euclid_edit_mode) {
      switch (euclid_selected_param) {
        case 0: steps  = 1 + (raw1 * 15) / 4095; break; // 1..16
        case 1: pulses = (raw1 * euclid_steps) / 4095; break; // 0..steps
        case 2: euclid_rotation = (raw1 * euclid_steps) / 4095; break; // 0..steps-1 approx
        case 3: bpm    = 30 + (int)(((long)raw1 * (300-30)) / 4095); break; // 30..300
      }
    } else {
      steps  = 1 + (raw1 * 15) / 4095;
      pulses = (raw2 * steps) / 4095;
      // bpm from raw3 already set
    }
  } else {
    // Complex mode: per-channel params; Pot1 edits selected (channel,param)
    if (euclid_edit_mode) {
      int ch = euclid_sel_channel;
      switch (euclid_selected_param) {
        case 0: euclid_ch_steps[ch]   = 1 + (raw1 * 15) / 4095; break;
        case 1: euclid_ch_pulses[ch]  = (raw1 * euclid_ch_steps[ch]) / 4095; break;
        case 2: euclid_ch_rotation[ch]= (raw1 * euclid_ch_steps[ch]) / 4095; break;
        case 3: bpm = 30 + (int)(((long)raw1 * (300-30)) / 4095); break;
      }
    }
  }
  if (bpm <= 0) bpm = 120;

  // short-press cycles selected parameter (and channel in complex mode)
  if (patchShortPressed) {
    if (euclid_mode == 0) {
      euclid_selected_param = (euclid_selected_param + 1) % 4;
    } else {
      // cycle param first, then channel
      euclid_selected_param = (euclid_selected_param + 1) % 4;
      if (euclid_selected_param == 0) {
        euclid_sel_channel = (euclid_sel_channel + 1) % 4;
      }
    }
    patchShortPressed = false;
  }

  // rebuild patterns if needed
  if (euclid_mode == 0 && (steps != euclid_steps || pulses != euclid_pulses)) {
    euclid_steps = steps;
    euclid_pulses = pulses;
    // build a base pattern and make 4 rotated variants
    bool base[16] = {0};
    build_euclid_pattern(base, euclid_steps, euclid_pulses, 0);
    for (int ch=0; ch<4; ch++) {
      int ro = (euclid_rotation + ch) % euclid_steps;
      // copy & rotate into pattern
      for (int i=0;i<euclid_steps;i++) euclid_patterns[ch][i] = base[(i - ro + euclid_steps) % euclid_steps];
    }
  }
  if (euclid_mode == 1) {
    // per-channel patterns
    for (int ch=0; ch<4; ch++) {
      int steps_c = euclid_ch_steps[ch]; if (steps_c < 1) steps_c = 1; if (steps_c > 16) steps_c = 16;
      int pulses_c = euclid_ch_pulses[ch]; if (pulses_c < 0) pulses_c = 0; if (pulses_c > steps_c) pulses_c = steps_c;
      int rot_c = euclid_ch_rotation[ch]; if (rot_c < 0) rot_c = 0; if (rot_c >= steps_c) rot_c = steps_c-1;
      bool base[16] = {0};
      build_euclid_pattern(base, steps_c, pulses_c, 0);
      for (int i=0;i<steps_c;i++) euclid_patterns[ch][i] = base[(i - rot_c + steps_c) % steps_c];
      // zero out remainder to avoid stray triggers
      for (int i=steps_c;i<16;i++) euclid_patterns[ch][i] = false;
    }
  }

  uint32_t now = millis();
  uint32_t interval_ms = 60000 / bpm;
  if (now >= euclid_next_ms) {
    euclid_next_ms = now + interval_ms;
    // advance step index
    euclid_step_idx = (euclid_step_idx + 1) % (euclid_mode==0 ? euclid_steps : 16);
    // trigger channels according to their patterns
    for (int ch=0; ch<4; ch++) {
      bool on = false;
      if ((euclid_mode==0 && euclid_steps > 0) || euclid_mode==1) on = euclid_patterns[ch][euclid_step_idx];
      if (on) {
        euclid_state[ch] = true;
        euclid_pulse_end_ms[ch] = now + 30; // 30 ms gate
      }
    }
  }

  // expire pulses
  for (int ch=0; ch<4; ch++) {
    if (euclid_state[ch] && now >= euclid_pulse_end_ms[ch]) euclid_state[ch] = false;
  }

  // write MCP outputs
  if (haveMCP) {
    uint16_t out0 = euclid_state[0] ? 4095 : 0;
    uint16_t out1 = euclid_state[1] ? 4095 : 0;
    uint16_t out2 = euclid_state[2] ? 4095 : 0;
    uint16_t out3 = euclid_state[3] ? 4095 : 0;
    mcp_values[0]=out0; mcp_values[1]=out1; mcp_values[2]=out2; mcp_values[3]=out3;
    mcp.fastWrite(out0,out1,out2,out3);
  }
}

void euclid_render() {
  oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
  ui::printClipped(0, 0, 64, "Euclid");
  // Show mode in status area: Simple/Complex
  oled.setCursor(66,0);
  oled.print(euclid_mode == 0 ? "Simple" : "Complex");

  if (euclid_mode == 0) {
    // Simple mode UI
    oled.setCursor(0,14); oled.print("Steps "); oled.print(euclid_steps);
    if (euclid_selected_param == 0) oled.drawFastHLine(0, 23, 40, SSD1306_WHITE);
    oled.setCursor(64,14); oled.print("Pulses "); oled.print(euclid_pulses);
    if (euclid_selected_param == 1) oled.drawFastHLine(64, 23, 50, SSD1306_WHITE);
    oled.setCursor(0,24); oled.print("Rot "); oled.print(euclid_rotation);
    if (euclid_selected_param == 2) oled.drawFastHLine(0, 33, 30, SSD1306_WHITE);
    int bpm_disp = 30 + (int)(((long)(4095 - analogRead(PIN_POT3)) * (300-30)) / 4095);
    oled.setCursor(64,24); oled.print("BPM "); oled.print(bpm_disp);
    if (euclid_selected_param == 3) oled.drawFastHLine(64, 33, 40, SSD1306_WHITE);
  } else {
    // Complex mode UI: show per-channel params compactly
    for (int ch=0; ch<4; ch++) {
      int y = (ch < 2) ? 14 : 24;
      int x = (ch % 2 == 0) ? 0 : 64;
      oled.setCursor(x, y);
      oled.print("CH"); oled.print(ch); oled.print(" ");
      oled.print(euclid_ch_steps[ch]); oled.print('/');
      oled.print(euclid_ch_pulses[ch]); oled.print(" r");
      oled.print(euclid_ch_rotation[ch]);
      // underline current selection (channel + param)
      if (ch == euclid_sel_channel) {
        int ux = x;
        int uw = (euclid_selected_param==0)?20:(euclid_selected_param==1)?28:10;
        int uy = y + 9;
        oled.drawFastHLine(ux, uy, uw, SSD1306_WHITE);
      }
    }
    int bpm_disp = 30 + (int)(((long)(4095 - analogRead(PIN_POT3)) * (300-30)) / 4095);
    oled.setCursor(0,34); oled.print("BPM "); oled.print(bpm_disp);
  }

  // channel states
  oled.setCursor(0,34); oled.print("D0 "); oled.print(mcp_values[0]); oled.setCursor(64,34); oled.print("D1 "); oled.print(mcp_values[1]);
  oled.setCursor(0,44); oled.print("D2 "); oled.print(mcp_values[2]); oled.setCursor(64,44); oled.print("D3 "); oled.print(mcp_values[3]);

  oled.display();
}
Patch patch_euclid = { "Euclid", euclid_enter, euclid_tick, euclid_render };

void mod_enter() {}
void mod_tick() {}
void mod_render() {
  oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
  ui::printClipped(0, UI_TOP_MARGIN, OLED_W, "Mod");
  oled.display();
}
Patch patch_mod = { "Mod", mod_enter, mod_tick, mod_render };

void osc_enter() {}
void osc_tick() {}
void osc_render() {
  oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
  ui::printClipped(0, UI_TOP_MARGIN, OLED_W, "Osc");
  oled.display();
}
Patch patch_osc = { "Osc", osc_enter, osc_tick, osc_render };

void calib_enter() {}
void calib_tick() {}
void calib_render() {
  oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
  ui::printClipped(0, UI_TOP_MARGIN, OLED_W, "Calib");
  oled.display();
}
Patch patch_calib = { "Calib", calib_enter, calib_tick, calib_render };

// ---- Scope patch: basic ADC oscilloscope for ADS inputs ----
static const int SCOPE_SAMPLES = 128;
static int16_t scope_buf[SCOPE_SAMPLES];
static int scope_idx = 0;

void scope_enter() {
  scope_idx = 0;
  for (int i=0;i<SCOPE_SAMPLES;i++) scope_buf[i]=0;
}

void scope_tick() {
  if (!haveADS) return;
  int16_t a0 = ads.readADC_SingleEnded(0);
  scope_buf[scope_idx++] = a0;
  if (scope_idx >= SCOPE_SAMPLES) scope_idx = 0;
}

void scope_render() {
  oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
  ui::printClipped(0, 0, 64, "Scope");

  // Zoom controls: Pot1 = vertical gain, Pot2 = horizontal window (visible samples)
  int rawV = 4095 - analogRead(PIN_POT1);
  int rawH = 4095 - analogRead(PIN_POT2);
  float vgain = 0.25f + (3.75f * (float)rawV / 4095.0f);      // ~0.25x .. 4x
  int visible = 32 + (rawH * (128 - 32)) / 4095; if (visible < 2) visible = 2; // 32..128

  // Show zoom status in title bar (right side)
  oled.setCursor(66,0);
  oled.print("Vx"); oled.print(vgain, 1); oled.print(" H"); oled.print(visible);

  // plotting area: from y = 14 .. OLED_H-1
  const int y0 = 14;
  const int h = OLED_H - y0 - 1;
  if (h <= 4) { oled.display(); return; }

  // draw center line
  const int cy = y0 + (h/2);
  oled.drawFastHLine(0, cy, OLED_W, SSD1306_WHITE);

  // Baseline (DC) as average of buffer to center waveform
  long sum = 0; for (int i=0;i<SCOPE_SAMPLES;i++) sum += (int)scope_buf[i];
  int base = (int)(sum / SCOPE_SAMPLES);

  // Determine start index for last 'visible' samples
  int start = scope_idx - visible; while (start < 0) start += SCOPE_SAMPLES;
  int prevx = 0; int prevy = cy;
  for (int i=0;i<visible;i++) {
    int s = scope_buf[(start + i) % SCOPE_SAMPLES];
    int centered = s - base; // ~-32767..+32767
    int y = cy - (int)((centered * vgain * (h-1)) / 32767.0f);
    if (y < y0) y = y0; else if (y > y0 + h - 1) y = y0 + h - 1;
    int x = (i * (OLED_W - 1)) / (visible - 1);
    if (i>0) oled.drawLine(prevx, prevy, x, y, SSD1306_WHITE);
    prevx = x; prevy = y;
  }

  oled.display();
}
Patch patch_scope = { "Scope", scope_enter, scope_tick, scope_render };
// Arrange the bank so indexes match the desired home-menu ordering below.
// We'll place `patch_clock` at index 0 and `patch_diag` at index 7 so selecting
// those menu entries activates the registered patches. Other slots are placeholders.
Bank bank_util = { "Util", { &patch_clock, &patch_quant, &patch_euclid, &patch_mod, &patch_osc, &patch_calib, &patch_scope, &patch_diag }, 8 };
Bank* banks[] = { &bank_util };
static uint8_t bankIdx  = 0;
static uint8_t patchIdx = 0;

// ---- Home menu + input state ----
// Home menu items (4x2 grid viewport). Order updated to the requested first-8 patches.
static const char* kHomeItems[] = { "Clock", "Quant", "Euclid", "Mod", "Osc", "Calib", "Scope", "Diag" };
static eurorack_ui::OledHomeMenu homeMenu;
static bool homeMenuActive = true;
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
        // If the selected index corresponds to a registered patch in the current bank,
        // activate that patch. Otherwise treat it as a placeholder screen.
        if (sel < banks[bankIdx]->patchCount && banks[bankIdx]->patches[sel] != nullptr) {
          homeMenuActive = false;
          activePlaceholder = -1;
          bankIdx = 0; patchIdx = sel;
          Patch* p = banks[bankIdx]->patches[patchIdx];
          if (p && p->enter) p->enter();
        } else {
          // Placeholder screens for other items: remember which placeholder is active
          activePlaceholder = sel; // 1..N
          homeMenuActive = false;
          // draw immediately (single-word placeholder below the top band)
          oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
          ui::printClipped(0, UI_TOP_MARGIN, OLED_W, kHomeItems[sel]);
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
    // Reserve a top band for the menu/colour zone so patch info prints below it
    homeMenu.begin(&oled, UI_TOP_MARGIN);
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

