#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP4728.h>
#include <math.h>
#include <Bounce2.h>
#include <EEPROM.h>

#include "eurorack_ui/OledHelpers.hpp"
#include "eurorack_ui/OledHomeMenu.hpp"

// -------------------- Pins / Addresses (moved to header) --------------------
#include "pico2w_oc/pins.h"

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

// ---- Forward declarations for calibration helpers (implemented after Calib) ----
static bool calibAvailable();
static float mapAdsToCv(float adsV);
static uint16_t voltsToDac(int ch, float v);
// Eurorack output stage scaling (your analog stage 0..Vref -> ±5V)
static float dacToEurorackGain = (10.0f / mcpVrefFull);
// Gate output codes (ignore calibration for clock-like gates)
static const uint16_t kGateLowCode = 2047; // ~0V baseline
static const uint16_t kGateHighCode = 0;   // ~+5V on your hardware

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
// Diag patch DAC control state
static int diag_sel_dac = 0; // which DAC channel is under manual control
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

// Smoothed pot read (inverted), simple exponential moving average
static float potSmooth[3] = {0.0f, 0.0f, 0.0f};
float readPotNormSmooth(int pin, int idx) {
  int v = analogRead(pin);
  float norm = constrain(1.0f - (v / 4095.0f), 0.0f, 1.0f);
  const float alpha = 0.2f; // smoothing factor
  potSmooth[idx] = (1.0f - alpha) * potSmooth[idx] + alpha * norm;
  return potSmooth[idx];
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
    int16_t a0 = ads.readADC_SingleEnded(AD0_CH);
    int16_t a1 = ads.readADC_SingleEnded(AD1_CH);
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

  // DAC manual test: short press cycles logical DAC index (0..3). Pot1 sets ONLY that physical channel using mapping macros.
  if (haveMCP) {
    if (patchShortPressed) {
      diag_sel_dac = (diag_sel_dac + 1) & 0x3; // cycle CV1->CV4
      patchShortPressed = false;
    }
    // Clear all first (physical indices)
        mcp_values[0] = 0; mcp_values[1] = 0; mcp_values[2] = 0; mcp_values[3] = 0;
    // Map selected physical CV (CV0..CV3) to underlying DA channel using pins.h
    const uint8_t cv_phys[4] = { CV0_DA_CH, CV1_DA_CH, CV2_DA_CH, CV3_DA_CH };
    uint8_t phys = cv_phys[diag_sel_dac];
    mcp_values[phys] = (uint16_t)(pot1 * 4095.0f);
    // Write in physical channel order
        mcp.fastWrite(
          mcp_values[CV3_DA_CH],
          mcp_values[CV0_DA_CH],
          mcp_values[CV2_DA_CH],
          mcp_values[CV1_DA_CH]
        );
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
  oled.setCursor(0, 16);
  oled.print("BTN "); oled.print(btn.read() == LOW ? "DOWN" : "UP  ");

  int raw1 = analogRead(PIN_POT1);
  int raw2 = analogRead(PIN_POT2);
  int raw3 = analogRead(PIN_POT3);
  // Line: Btn <status>    P1 <raw>
  oled.setCursor(64, 16); oled.print("P1 "); oled.print(raw1);
  // Next line: P2 <raw>    P3 <raw>
  oled.setCursor(0, 26); oled.print("P2 "); oled.print(raw2);
  oled.setCursor(64, 26); oled.print("P3 "); oled.print(raw3);

  // CV inputs raw
  // ADS1115 raw codes (16-bit signed). Show as ADC0/ADC1.
  oled.setCursor(0, 36); oled.print("ADC0 "); oled.print(ads_raw0);
  oled.setCursor(64, 36); oled.print("ADC1 "); oled.print(ads_raw1);

  // Show physical CV outputs (CV0..CV3) mapped to their DA channels
  oled.setCursor(0, 46);  oled.print(diag_sel_dac==0?">":" "); oled.print("CV0 "); oled.print(mcp_values[CV0_DA_CH]);
  oled.setCursor(64, 46); oled.print(diag_sel_dac==1?">":" "); oled.print("CV1 "); oled.print(mcp_values[CV1_DA_CH]);
  oled.setCursor(0, 56);  oled.print(diag_sel_dac==2?">":" "); oled.print("CV2 "); oled.print(mcp_values[CV2_DA_CH]);
  oled.setCursor(64, 56); oled.print(diag_sel_dac==3?">":" "); oled.print("CV3 "); oled.print(mcp_values[CV3_DA_CH]);

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
// Option B state
static int clock_div_idx[4] = {kDivCount/2, kDivCount/2, kDivCount/2, kDivCount/2};

// Helper: find nearest division index to a target factor
static int clock_find_nearest_div(float target) {
  int best = 0; float bestDiff = fabsf(div_factors[0] - target);
  for (int i=1;i<kDivCount;i++) {
    float d = fabsf(div_factors[i] - target);
    if (d < bestDiff) { bestDiff = d; best = i; }
  }
  return best;
}

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

  // read pots (smoothed, inverted) POT1=BPM, POT2=CH0 base, POT3=CH2 base
  float p_bpm = readPotNormSmooth(PIN_POT1, 0);
  float p_ch0 = readPotNormSmooth(PIN_POT2, 1);
  float p_ch2 = readPotNormSmooth(PIN_POT3, 2);
  int pot_raw_bpm = (int)(p_bpm * 4095.0f);
  int pot_raw_ch0 = (int)(p_ch0 * 4095.0f);
  int pot_raw_ch2 = (int)(p_ch2 * 4095.0f);

  uint32_t now = millis();

  // detect external clock on ADS channel 0 (if present)
  bool have_ext = false;
  if (haveADS) {
    int a0 = ads.readADC_SingleEnded(AD_EXT_CLOCK_CH);
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

  // base interval: external if present, else internal from POT1 mapped to BPM
  if (have_ext && clock_ext_interval_ms > 0) {
    clock_base_interval_ms = clock_ext_interval_ms;
  } else {
    // use POT1 to set tempo 30..300 BPM
    int p = pot_raw_bpm;
    int bpm = 30 + (int)(((long)p * (300-30)) / 4095);
    if (bpm <= 0) bpm = 120;
    clock_base_interval_ms = 60000 / bpm;
  }
  // Map POT2 -> CH0 division index, POT3 -> CH2 division index
  int idx_ch0 = (int)((uint32_t)pot_raw_ch0 * kDivCount / 4096);
  if (idx_ch0 < 0) idx_ch0 = 0; else if (idx_ch0 >= kDivCount) idx_ch0 = kDivCount - 1;
  int idx_ch2 = (int)((uint32_t)pot_raw_ch2 * kDivCount / 4096);
  if (idx_ch2 < 0) idx_ch2 = 0; else if (idx_ch2 >= kDivCount) idx_ch2 = kDivCount - 1;
  clock_div_idx[0] = idx_ch0;
  clock_div_idx[2] = idx_ch2;

  // Derived channels: CH1 = half speed of CH0, CH3 = half speed of CH2
  // "Half" interpreted as pulses at half the rate (interval doubled -> factor halved)
  float f0 = div_factors[clock_div_idx[0]] * 0.5f;
  float f2 = div_factors[clock_div_idx[2]] * 0.5f;
  if (f0 < div_factors[0]) f0 = div_factors[0];
  if (f2 < div_factors[0]) f2 = div_factors[0];
  clock_div_idx[1] = clock_find_nearest_div(f0);
  clock_div_idx[3] = clock_find_nearest_div(f2);

  for (int ch=0; ch<4; ch++) {
    float factor = div_factors[clock_div_idx[ch]];
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

  // CH1 & CH3 derived above; no independent editing.

  // write MCP outputs if available (direct codes: low~2047, high~0)
  if (haveMCP) {
    uint16_t out0 = ch_state[0] ? kGateHighCode : kGateLowCode;
    uint16_t out1 = ch_state[1] ? kGateHighCode : kGateLowCode;
    uint16_t out2 = ch_state[2] ? kGateHighCode : kGateLowCode;
    uint16_t out3 = ch_state[3] ? kGateHighCode : kGateLowCode;
    // remember values for display
    mcp_values[CV0_DA_CH] = out0; mcp_values[CV1_DA_CH] = out1; mcp_values[CV2_DA_CH] = out2; mcp_values[CV3_DA_CH] = out3;
    // write physical channels A..D
    mcp.fastWrite(
      mcp_values[CV3_DA_CH],
      mcp_values[CV0_DA_CH],
      mcp_values[CV2_DA_CH],
      mcp_values[CV1_DA_CH]
    );
  }
}

void clock_render() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextWrap(false);
  ui::printClipped(0, 0, 64, "Clock");
  // status area unused on patches per request

  // Mode / BPM / Run state on single line (y=16)
  oled.setCursor(0, 16);
  bool ext_mode = (clock_ext_interval_ms > 0);
  int bpm_disp = ext_mode ? (int)(60000.0f / (float)clock_ext_interval_ms + 0.5f)
                          : (int)(60000.0f / (float)clock_base_interval_ms + 0.5f);
  oled.print(ext_mode ? "EXT " : "INT ");
  oled.print(bpm_disp); oled.print(' '); oled.print(clock_running ? "RUN" : "STOP");

  // Channel divisions rows (fixed, no selection underline)
  oled.setCursor(0, 26); oled.print("CH0 "); oled.print(div_labels[clock_div_idx[0]]);
  oled.setCursor(64,26); oled.print("CH1 "); oled.print(div_labels[clock_div_idx[1]]);
  oled.setCursor(0, 36); oled.print("CH2 "); oled.print(div_labels[clock_div_idx[2]]);
  oled.setCursor(64,36); oled.print("CH3 "); oled.print(div_labels[clock_div_idx[3]]);

  oled.display();
}

Patch patch_clock = { "Clock", clock_enter, clock_tick, clock_render };
// -- Placeholder patch stubs for menu entries (lightweight)

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
  // read pots inverted so clockwise increases (12-bit)
  // Pot1=BPM, Pot2=mode, Pot3=param
  int raw1 = 4095 - analogRead(PIN_POT1);
  int raw2 = 4095 - analogRead(PIN_POT2);
  int raw3 = 4095 - analogRead(PIN_POT3);
  // Mode selection via Pot2 (coarse split): <50% simple, >=50% complex
  euclid_mode = (raw2 >= 2048) ? 1 : 0;

  int steps = euclid_steps;
  int pulses = euclid_pulses;
  int bpm = 30 + (int)(((long)raw1 * (300-30)) / 4095);

  if (euclid_mode == 0) {
    // Simple mode: shared params
    if (euclid_edit_mode) {
      switch (euclid_selected_param) {
        case 0: steps  = 1 + (raw3 * 15) / 4095; break; // 1..16
        case 1: pulses = (raw3 * euclid_steps) / 4095; break; // 0..steps
        case 2: euclid_rotation = (raw3 * euclid_steps) / 4095; break; // 0..steps-1 approx
        case 3: /* BPM via Pot1 */ break;
      }
    } else {
      steps  = 1 + (raw3 * 15) / 4095;
      pulses = (raw3 * steps) / 4095;
      // bpm from raw1 already set
    }
  } else {
    // Complex mode: per-channel params; Pot1 edits selected (channel,param)
    if (euclid_edit_mode) {
      int ch = euclid_sel_channel;
      switch (euclid_selected_param) {
        case 0: euclid_ch_steps[ch]   = 1 + (raw3 * 15) / 4095; break;
        case 1: euclid_ch_pulses[ch]  = (raw3 * euclid_ch_steps[ch]) / 4095; break;
        case 2: euclid_ch_rotation[ch]= (raw3 * euclid_ch_steps[ch]) / 4095; break;
        case 3: /* BPM via Pot1 */ break;
      }
    }
  }
  if (bpm <= 0) bpm = 120;

  // short-press cycles selected parameter (and channel in complex mode)
  if (patchShortPressed) {
    if (euclid_mode == 0) {
      euclid_selected_param = (euclid_selected_param + 1) % 4;
    } else {
      // cycle Steps/Pulses/Rotation; advance channel when wrapping
      euclid_selected_param = (euclid_selected_param + 1) % 3;
      if (euclid_selected_param == 0) euclid_sel_channel = (euclid_sel_channel + 1) % 4;
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

  // write MCP outputs (direct codes: low~2047, high~0)
  if (haveMCP) {
    uint16_t out0 = euclid_state[0] ? kGateHighCode : kGateLowCode;
    uint16_t out1 = euclid_state[1] ? kGateHighCode : kGateLowCode;
    uint16_t out2 = euclid_state[2] ? kGateHighCode : kGateLowCode;
    uint16_t out3 = euclid_state[3] ? kGateHighCode : kGateLowCode;
    mcp_values[CV0_DA_CH]=out0; mcp_values[CV1_DA_CH]=out1; mcp_values[CV2_DA_CH]=out2; mcp_values[CV3_DA_CH]=out3;
    mcp.fastWrite(
      mcp_values[CV3_DA_CH],
      mcp_values[CV0_DA_CH],
      mcp_values[CV2_DA_CH],
      mcp_values[CV1_DA_CH]
    );
  }
}

void euclid_render() {
  oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
  ui::printClipped(0, 0, 64, "Euclid");
  // Mode in header right
  oled.setCursor(66,0);
  oled.print(euclid_mode == 0 ? "Simple" : "Complex");
  // Row 1: BPM only
  int bpm_disp = 30 + (int)(((long)(4095 - analogRead(PIN_POT1)) * (300-30)) / 4095);
  oled.setCursor(0,16); oled.print("BPM "); oled.print(bpm_disp);

  if (euclid_mode == 0) {
    // Simple mode UI
    // Row 2: Steps / Pulses
    oled.setCursor(0,26); oled.print("Steps "); oled.print(euclid_steps);
    if (euclid_selected_param == 0) oled.drawFastHLine(0, 35, 40, SSD1306_WHITE);
    oled.setCursor(64,26); oled.print("Pulses "); oled.print(euclid_pulses);
    if (euclid_selected_param == 1) oled.drawFastHLine(64, 35, 50, SSD1306_WHITE);
    // Row 3: Rotation (right side unused)
    oled.setCursor(0,36); oled.print("Rot "); oled.print(euclid_rotation);
    if (euclid_selected_param == 2) oled.drawFastHLine(0, 45, 30, SSD1306_WHITE);
  } else {
    // Complex mode UI: show per-channel params compactly
    for (int ch=0; ch<4; ch++) {
      int y = (ch < 2) ? 26 : 36;
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
    // BPM shown above; underline selection as before
  }

  // Row 4 & 5: CV outputs (codes) for visibility
  oled.setCursor(0,46); oled.print("CV0 "); oled.print(mcp_values[CV0_DA_CH]);
  oled.setCursor(64,46); oled.print("CV1 "); oled.print(mcp_values[CV1_DA_CH]);
  oled.setCursor(0,56); oled.print("CV2 "); oled.print(mcp_values[CV2_DA_CH]);
  oled.setCursor(64,56); oled.print("CV3 "); oled.print(mcp_values[CV3_DA_CH]);

  oled.display();
}
Patch patch_euclid = { "Euclid", euclid_enter, euclid_tick, euclid_render };

// ---- QuadLFO patch: 4 independent LFOs (Amp / Rate / Shape) ----
// Pots (smoothed, inverted):
//   Pot1: Amplitude (0..1 -> 0..5V peak, bipolar)
//   Pot2: Rate (0.05Hz .. 20Hz, squared mapping for fine low-end control)
//   Pot3: Shape selection (Sine/Tri/Square/Up/Dn)
// Short press: cycle edited LFO (0..3)
// Long press: return to menu (handled globally)

enum LfoShape { LFO_SINE, LFO_TRI, LFO_SQUARE, LFO_RAMP_UP, LFO_RAMP_DOWN, LFO_COUNT };
static const char* kLfoShapeNames[LFO_COUNT] = { "Sin", "Tri", "Sq", "Up", "Dn" };
static float lfo_phase[4]     = {0,0,0,0};   // 0..1 wrapping
static float lfo_rate_hz[4]   = {1,1,1,1};   // Hz
static float lfo_amp[4]       = {1,1,1,1};   // 0..5V peak (amplitude knob scales)
static uint8_t lfo_shape[4]   = {LFO_SINE,LFO_TRI,LFO_SQUARE,LFO_RAMP_UP};
static int lfo_edit_idx       = 0;           // which LFO pots are editing
static uint32_t lfo_last_ms   = 0;           // last tick time

static float quadlfo_shape_eval(uint8_t shape, float ph) {
  // ph in [0,1)
  switch(shape) {
    case LFO_SINE: return sinf(ph * 2.0f * PI); // -1..1
    case LFO_TRI: {
      float t = ph;
      float tri = (t < 0.25f) ? (t * 4.0f) : (t < 0.75f ? 2.0f - t*4.0f : (t*4.0f - 4.0f));
      return tri; // -1..1
    }
    case LFO_SQUARE: return (ph < 0.5f) ? 1.0f : -1.0f;
    case LFO_RAMP_UP: return (ph * 2.0f) - 1.0f;          // -1..1 rising
    case LFO_RAMP_DOWN: return 1.0f - (ph * 2.0f);        // -1..1 falling
    default: return 0.0f;
  }
}

void quadlfo_enter() {
  lfo_edit_idx = 0;
  for (int i=0;i<4;i++) { lfo_phase[i]=0.0f; lfo_rate_hz[i]=1.0f; lfo_amp[i]=2.5f; }
  lfo_shape[0]=LFO_SINE; lfo_shape[1]=LFO_TRI; lfo_shape[2]=LFO_SQUARE; lfo_shape[3]=LFO_RAMP_UP;
  lfo_last_ms = millis();
}

void quadlfo_tick() {
  uint32_t now = millis();
  float dt_ms = (float)(now - lfo_last_ms); if (dt_ms < 0) dt_ms = 0; lfo_last_ms = now;
  float dt = dt_ms * 0.001f; // seconds

  // Pots (smoothed, inverted)
  float p_amp  = readPotNormSmooth(PIN_POT1, 0); // amplitude 0..1
  float p_rate = readPotNormSmooth(PIN_POT2, 1); // rate mapping
  float p_shape= readPotNormSmooth(PIN_POT3, 2); // shape select

  // Short press cycles edited LFO index
  if (patchShortPressed) { lfo_edit_idx = (lfo_edit_idx + 1) & 0x3; patchShortPressed = false; }

  // Update selected LFO params
  // Amplitude: 0..5V peak (bipolar), use direct scaling
  lfo_amp[lfo_edit_idx] = p_amp * 5.0f;
  // Rate: square for resolution at low end -> 0.05 .. 20 Hz
  float rate = 0.05f + (p_rate * p_rate) * (20.0f - 0.05f);
  lfo_rate_hz[lfo_edit_idx] = rate;
  // Shape index
  int sh = (int)(p_shape * (float)LFO_COUNT);
  if (sh < 0) sh = 0; else if (sh >= LFO_COUNT) sh = LFO_COUNT - 1;
  lfo_shape[lfo_edit_idx] = (uint8_t)sh;

  // Advance all LFO phases
  for (int i=0;i<4;i++) {
    float inc = lfo_rate_hz[i] * dt; // cycles per second * seconds = cycles
    lfo_phase[i] += inc;
    // wrap
    if (lfo_phase[i] >= 1.0f) lfo_phase[i] -= floorf(lfo_phase[i]);
    if (lfo_phase[i] < 0.0f) lfo_phase[i] = fmodf(lfo_phase[i], 1.0f);
  }

  // Generate outputs and write DACs (CV0..CV3)
  if (haveMCP) {
    for (int i=0;i<4;i++) {
      float val = quadlfo_shape_eval(lfo_shape[i], lfo_phase[i]); // -1..1
      float volts = val * lfo_amp[i]; // -amp .. +amp (bipolar)
      // Clamp to ±5V domain
      if (volts < -5.0f) volts = -5.0f; else if (volts > 5.0f) volts = 5.0f;
      uint16_t code = voltsToDac(i, volts); // i assumes CVx_DA_CH logical match order 0..3
      // Map logical i to physical macros for safety
      uint8_t physIndex = (i==0)?CV0_DA_CH:(i==1)?CV1_DA_CH:(i==2)?CV2_DA_CH:CV3_DA_CH;
      mcp_values[physIndex] = code;
    }
    mcp.fastWrite(
      mcp_values[CV3_DA_CH],
      mcp_values[CV0_DA_CH],
      mcp_values[CV2_DA_CH],
      mcp_values[CV1_DA_CH]
    );
  }
}

void quadlfo_render() {
  oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE); oled.setTextWrap(false);
  ui::printClipped(0, 0, 64, "QuadLFO");
  oled.setCursor(66,0); oled.print("L"); oled.print(lfo_edit_idx);

  // Row 1: Show only amplitude for the selected LFO
  float ampV = lfo_amp[lfo_edit_idx];
  oled.setCursor(0,16);
  oled.print(">L"); oled.print(lfo_edit_idx);
  oled.print(" Amp "); oled.print(ampV,1); oled.print("V");

  // Rows for each LFO summary
  for (int i=0;i<4;i++) {
    int y = 26 + i*10; if (y > 56) y = 56; // ensure fits grid
    oled.setCursor(0,y);
    if (i == lfo_edit_idx) oled.print("*"); else oled.print(" ");
    oled.print("L"); oled.print(i); oled.print(" ");
    oled.print(lfo_rate_hz[i],2); oled.print("Hz ");
    oled.print(kLfoShapeNames[lfo_shape[i]]); oled.print(" A"); oled.print(lfo_amp[i],1);
  }

  oled.display();
}
Patch patch_mod = { "LFO", quadlfo_enter, quadlfo_tick, quadlfo_render };

// ---- Env patch: Dual macro ADSR (per-env AD + SR + Velocity) ----
enum EnvStage { ENV_IDLE, ENV_ATTACK, ENV_DECAY, ENV_RELEASE };
static uint32_t env_last_ms = 0;   // last tick time for dt
// Editing selection: which envelope pots are writing to (0=E1,1=E2)
static int env_edit_idx = 0;
// Per-envelope params and runtime state
static float env_params_AD[2]  = {0.5f, 0.5f}; // 0..1 maps to attack/decay pair
static float env_params_SR[2]  = {0.5f, 0.5f}; // 0..1 maps to sustain/release
static float env_params_Vel[2] = {1.0f, 1.0f}; // 0..1 output scaling
static float env_levels[2]     = {0.0f, 0.0f}; // current envelope levels
static EnvStage env_stages[2]  = {ENV_IDLE, ENV_IDLE};

void env_enter() {
  env_edit_idx = 0;
  env_params_AD[0] = env_params_AD[1] = 0.5f;
  env_params_SR[0] = env_params_SR[1] = 0.5f;
  env_params_Vel[0] = env_params_Vel[1] = 1.0f;
  env_levels[0] = env_levels[1] = 0.0f;
  env_stages[0] = env_stages[1] = ENV_IDLE;
  env_last_ms = millis();
}

void env_tick() {
  uint32_t now = millis();
  float dt = (float)(now - env_last_ms);
  if (dt < 0) dt = 0; env_last_ms = now;

  // Pots inverted with smoothing so CW increases
  float potVel = readPotNormSmooth(PIN_POT1, 0); // 0..1 velocity (amplitude)
  float potAD  = readPotNormSmooth(PIN_POT2, 1); // 0..1
  float potSR  = readPotNormSmooth(PIN_POT3, 2); // 0..1

  // Short press: select which envelope to edit (Env1/Env2)
  if (patchShortPressed) { env_edit_idx ^= 1; patchShortPressed = false; }

  // Update selected env params from pots
  env_params_AD[env_edit_idx]  = potAD;
  env_params_SR[env_edit_idx]  = potSR;
  env_params_Vel[env_edit_idx] = potVel;

  // External triggers: independent rising edges
  // Env1: ADS channel `AD_EXT_CLOCK_CH` (external clock)
  // Env2: ADS channel `AD1_CH` (second input)
  if (haveADS) {
    const int delta = 4000; // heuristic for 16-bit readings
    static int prev_e0 = 0;
    static int prev_e1 = 0;
    int a0 = ads.readADC_SingleEnded(AD_EXT_CLOCK_CH);
    int a1 = ads.readADC_SingleEnded(AD1_CH);
    if (prev_e0 && a0 - prev_e0 > delta) env_stages[0] = ENV_ATTACK;
    if (prev_e1 && a1 - prev_e1 > delta) env_stages[1] = ENV_ATTACK;
    prev_e0 = a0; prev_e1 = a1;
  }

  // Advance both envelopes using their own params
  for (int i=0;i<2;i++) {
    float AD = env_params_AD[i];
    float SR = env_params_SR[i];
    float attack_ms = 1.0f + (AD * AD) * 2000.0f;
    float decay_base= attack_ms; // link A and D scaling together
    float sustain   = SR; if (sustain < 0.0f) sustain = 0.0f; if (sustain > 1.0f) sustain = 1.0f;
    // Make percussive settings truly punchy: scale decay by sustain
    // sustain=0 -> ~15% of base (fast), sustain=1 -> 100% of base
    float decay_ms  = decay_base * (0.15f + 0.85f * sustain);
    float release_ms= 1.0f + (SR * SR) * 2000.0f;

    switch (env_stages[i]) {
      case ENV_IDLE: env_levels[i] = 0.0f; break;
      case ENV_ATTACK: {
        float step = (attack_ms <= 0.0f) ? 1.0f : (dt / attack_ms);
        env_levels[i] += step; if (env_levels[i] >= 1.0f) { env_levels[i] = 1.0f; env_stages[i] = ENV_DECAY; }
      } break;
      case ENV_DECAY: {
        float span = (1.0f - sustain); if (span < 1e-6f) span = 1e-6f;
        float step = (decay_ms <= 0.0f) ? span : (dt * span / decay_ms);
        env_levels[i] -= step; if (env_levels[i] <= sustain) { env_levels[i] = sustain; env_stages[i] = ENV_RELEASE; }
      } break;
      case ENV_RELEASE: {
        float span = sustain; if (span < 1e-6f) span = 1e-6f;
        float step = (release_ms <= 0.0f) ? span : (dt * span / release_ms);
        env_levels[i] -= step; if (env_levels[i] <= 0.0f) { env_levels[i] = 0.0f; env_stages[i] = ENV_IDLE; }
      } break;
    }
  }

  // Output Env1->CV0, Env2->CV1 using their velocities
  if (haveMCP) {
    float v0 = env_levels[0] * env_params_Vel[0]; if (v0 < 0.0f) v0 = 0.0f; if (v0 > 1.0f) v0 = 1.0f;
    float v1 = env_levels[1] * env_params_Vel[1]; if (v1 < 0.0f) v1 = 0.0f; if (v1 > 1.0f) v1 = 1.0f;
    uint16_t c0 = (uint16_t)(2047.0f - (v0 * 2047.0f) + 0.5f);
    uint16_t c1 = (uint16_t)(2047.0f - (v1 * 2047.0f) + 0.5f);
    mcp_values[CV0_DA_CH] = c0;
    mcp_values[CV1_DA_CH] = c1;
    mcp.fastWrite(
      mcp_values[CV3_DA_CH],
      mcp_values[CV0_DA_CH],
      mcp_values[CV2_DA_CH],
      mcp_values[CV1_DA_CH]
    );
  }
}

void env_render() {
  oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE); oled.setTextWrap(false);
  ui::printClipped(0, 0, 64, "Env");
  // Editing indicator in header right
  oled.setCursor(66,0); oled.print(env_edit_idx==0?"E1":"E2");

  // Compute param displays from stored per-env values
  auto calc_params = [](float AD, float SR, float &Ams, float &Dms, int &Sperc, float &Rms){
    Ams = 1.0f + (AD * AD) * 2000.0f;
    float decay_base = Ams; // link A and D scaling together for display
    float S = SR; if (S < 0.0f) S = 0.0f; if (S > 1.0f) S = 1.0f; Sperc = (int)(S * 100.0f + 0.5f);
    Dms = decay_base * (0.15f + 0.85f * S);
    Rms = 1.0f + (SR * SR) * 2000.0f;
  };

  float Ams0, Dms0, Rms0; int S0;
  float Ams1, Dms1, Rms1; int S1;
  calc_params(env_params_AD[0], env_params_SR[0], Ams0, Dms0, S0, Rms0);
  calc_params(env_params_AD[1], env_params_SR[1], Ams1, Dms1, S1, Rms1);

  // Row 1 after title: velocity percentage of the selected envelope
  oled.setCursor(0,16);
  int vperc = (int)(env_params_Vel[env_edit_idx] * 100.0f + 0.5f);
  if (vperc < 0) vperc = 0; if (vperc > 100) vperc = 100;
  oled.print("Vel "); oled.print(vperc); oled.print("%");

  // E1 rows
  oled.setCursor(0,26);  oled.print(env_edit_idx==0?">E1 ":" E1 ");
  oled.print("A "); oled.print((int)Ams0); oled.print(" D "); oled.print((int)Dms0);
  oled.setCursor(0,36);  oled.print("    S "); oled.print(S0); oled.print("% R "); oled.print((int)Rms0);
  // E2 rows
  oled.setCursor(0,46);  oled.print(env_edit_idx==1?">E2 ":" E2 ");
  oled.print("A "); oled.print((int)Ams1); oled.print(" D "); oled.print((int)Dms1);
  oled.setCursor(0,56);  oled.print("    S "); oled.print(S1); oled.print("% R "); oled.print((int)Rms1);

  oled.display();
}
Patch patch_env = { "Env", env_enter, env_tick, env_render };

// ---- Calibration workflow (±4V references, averaging, EEPROM persistence) ----
struct CalibData {
  uint32_t magic;
  float ads_bias;   // volts bias at -4V
  float ads_gain;   // scale from ADS volts to ±4V (per-volt)
  float dac_offset[4]; // code at 0V per channel
  float dac_scale[4];  // codes per volt (±4V basis) per channel
};
static CalibData calib;
static int calib_step = 0; // 0: intro, 1: ADS -4V, 2: ADS +4V, 3: DAC -4V, 4: DAC +4V, 5: summary

static float getAdsVoltsAvg(int ch, int n) {
  if (!haveADS) return NAN;
  long acc = 0;
  for (int i=0;i<n;i++) acc += (int)ads.readADC_SingleEnded(ch);
  int16_t avg = (int16_t)(acc / n);
  return ads.computeVolts(avg);
}

void calib_enter() {
  // try load from EEPROM
  calib.magic = 0; EEPROM.get(0, calib.magic);
  if (calib.magic == 0xC01A1B00) {
    EEPROM.get(4, calib.ads_bias);
    EEPROM.get(8, calib.ads_gain);
    for (int i=0;i<4;i++) { EEPROM.get(12 + i*8, calib.dac_offset[i]); EEPROM.get(12 + i*8 + 4, calib.dac_scale[i]); }
  } else {
    calib.ads_bias = 0; calib.ads_gain = 1.0f; for (int i=0;i<4;i++){ calib.dac_offset[i]=2047.0f; calib.dac_scale[i]=4095.0f/10.0f; }
  }
  calib_step = 0;
}
void calib_tick() {
  if (patchShortPressed) { calib_step++; patchShortPressed=false; }
  if (calib_step == 1 && haveADS) {
    float v = getAdsVoltsAvg(0, 32);
    calib.ads_bias = v; // reading at -4V reference
  } else if (calib_step == 2 && haveADS) {
    float v = getAdsVoltsAvg(0, 32);
    // ads_gain maps (v - bias) to 4V; then *1.25 to reach ±5V domain later
    float span = v - calib.ads_bias; if (fabs(span) < 1e-6f) span = 1.0f;
    calib.ads_gain = 4.0f / span;
  } else if (calib_step == 3 && haveMCP) {
    // user sets DAC to -4V externally; capture codes (D0..D3)
    // assume current mcp_values hold last written codes
    // store as offsets for -4V baseline
    for (int ch=0; ch<4; ch++) calib.dac_offset[ch] = (float)mcp_values[ch];
  } else if (calib_step == 4 && haveMCP) {
    // user sets DAC to +4V; compute scale per volt (codes over 8V span)
    for (int ch=0; ch<4; ch++) {
      float codes_span = (float)mcp_values[ch] - calib.dac_offset[ch];
      calib.dac_scale[ch] = codes_span / 8.0f; // codes per volt for ±4V range
    }
  } else if (calib_step >= 5) {
    // persist and exit to menu (handled by long-press normally)
    calib.magic = 0xC01A1B00;
    EEPROM.put(0, calib.magic);
    EEPROM.put(4, calib.ads_bias);
    EEPROM.put(8, calib.ads_gain);
    for (int i=0;i<4;i++) { EEPROM.put(12 + i*8, calib.dac_offset[i]); EEPROM.put(12 + i*8 + 4, calib.dac_scale[i]); }
    // stay on summary until user long-presses to return
    calib_step = 5;
  }
}
void calib_render() {
  oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE); oled.setTextWrap(false);
  ui::printClipped(0, 0, 64, "Calib");
  if (calib_step == 0) {
    oled.setCursor(0,16); oled.print("ADS ch0: set -4V");
    oled.setCursor(0,26); oled.print("Short press to capture");
  } else if (calib_step == 1) {
    oled.setCursor(0,16); oled.print("ADS captured -4V");
    oled.setCursor(0,26); oled.print("Set +4V, short press");
  } else if (calib_step == 2) {
    oled.setCursor(0,16); oled.print("ADS captured +4V");
    oled.setCursor(0,26); oled.print("DAC: set -4V, press");
  } else if (calib_step == 3) {
    oled.setCursor(0,16); oled.print("DAC captured -4V");
    oled.setCursor(0,26); oled.print("DAC: set +4V, press");
  } else if (calib_step == 4) {
    oled.setCursor(0,16); oled.print("DAC captured +4V");
    oled.setCursor(0,26); oled.print("Press to save & exit");
  }
  // summary preview
  oled.setCursor(0,46); oled.print("bias "); oled.print(calib.ads_bias,3); oled.setCursor(64,46); oled.print("gain "); oled.print(calib.ads_gain,3);
  oled.setCursor(0,56); oled.print("D0off "); oled.print((int)calib.dac_offset[0]); oled.setCursor(64,56); oled.print("D0/v "); oled.print(calib.dac_scale[0],1);
  oled.display();
}
Patch patch_calib = { "Calib", calib_enter, calib_tick, calib_render };
// Implement calibration helpers now that `calib` exists
static bool calibAvailable() { return calib.magic == 0xC01A1B00; }
static float mapAdsToCv(float adsV) {
  if (calibAvailable()) return (adsV - calib.ads_bias) * calib.ads_gain * 1.25f;
  return (adsV - cvBias) * cvGain;
}
static uint16_t voltsToDac(int ch, float v) {
  if (isnan(v)) return 0u; if (v < -5.0f) v = -5.0f; if (v > 5.0f) v = 5.0f;
  float codef;
  if (calibAvailable()) {
    float scale = calib.dac_scale[ch] * 1.25f;
    codef = calib.dac_offset[ch] + (v * scale);
  } else {
    float scale = 4095.0f / 10.0f;
    codef = (v * scale) + 2047.0f;
  }
  if (codef < 0) codef = 0; if (codef > 4095) codef = 4095;
  return (uint16_t)(codef + 0.5f);
}

// ---- Quant patch (Simple): 2 inputs to 2 outputs, semitone 1V/oct ----
void quant_enter() {}
void quant_tick() {}
void quant_render() {
  oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE); oled.setTextWrap(false);
  ui::printClipped(0, 0, 64, "Quant");
  oled.setCursor(66,0); oled.print("Simple");

  float vin0 = NAN, vin1 = NAN;
  if (haveADS) {
    int16_t a0 = ads.readADC_SingleEnded(AD0_CH);
    int16_t a1 = ads.readADC_SingleEnded(AD1_CH);
    float v0 = ads.computeVolts(a0);
    float v1 = ads.computeVolts(a1);
    vin0 = mapAdsToCv(v0);
    vin1 = mapAdsToCv(v1);
  }

  auto quantize_voct = [](float v){
    if (isnan(v)) return v;
    if (v < -5.0f) v = -5.0f; if (v > 5.0f) v = 5.0f;
    float st = roundf(v * 12.0f);
    return st / 12.0f;
  };
  float vq0 = quantize_voct(vin0);
  float vq1 = quantize_voct(vin1);

  if (haveMCP) {
    uint16_t c0 = voltsToDac(CV0_DA_CH, vq0);
    uint16_t c1 = voltsToDac(CV1_DA_CH, vq1);
    mcp_values[CV0_DA_CH] = c0; mcp_values[CV1_DA_CH] = c1;
    mcp.fastWrite(
      mcp_values[CV3_DA_CH],
      mcp_values[CV0_DA_CH],
      mcp_values[CV2_DA_CH],
      mcp_values[CV1_DA_CH]
    );
  }

  oled.setCursor(0,16); oled.print("In0 "); if (isnan(vin0)) oled.print("--"); else { oled.print(vin0,2); }
  oled.setCursor(64,16); oled.print("Out0 "); if (isnan(vq0)) oled.print("--"); else { oled.print(vq0,2); }
  oled.setCursor(0,26); oled.print("In1 "); if (isnan(vin1)) oled.print("--"); else { oled.print(vin1,2); }
  oled.setCursor(64,26); oled.print("Out1 "); if (isnan(vq1)) oled.print("--"); else { oled.print(vq1,2); }
  oled.setCursor(0,36); oled.print("CV0 "); oled.print(mcp_values[CV0_DA_CH]);
  oled.setCursor(64,36); oled.print("CV1 "); oled.print(mcp_values[CV1_DA_CH]);
  oled.display();
}
Patch patch_quant = { "Quant", quant_enter, quant_tick, quant_render };

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
  int16_t a0 = ads.readADC_SingleEnded(AD0_CH);
  scope_buf[scope_idx++] = a0;
  if (scope_idx >= SCOPE_SAMPLES) scope_idx = 0;
}

void scope_render() {
  oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
  ui::printClipped(0, 0, 64, "Scope");

  // Zoom controls: Pot1 = vertical gain, Pot2 = horizontal window (visible samples)
  int rawV = 4095 - analogRead(PIN_POT1);
  int rawH = 4095 - analogRead(PIN_POT2);
  int rawM = 4095 - analogRead(PIN_POT3);
  float vgain = 0.25f + (3.75f * (float)rawV / 4095.0f);      // ~0.25x .. 4x
  int visible = 32 + (rawH * (128 - 32)) / 4095; if (visible < 2) visible = 2; // 32..128

  // Show zoom + midpoint status in title bar (right side)
  oled.setCursor(66,0);
  oled.print("Vx"); oled.print(vgain, 1); oled.print(" H"); oled.print(visible); oled.print(" M"); oled.print((int)(rawM/256));

  // plotting area: from y = 16 .. OLED_H-1
  const int y0 = 16;
  const int h = OLED_H - y0 - 1;
  if (h <= 4) { oled.display(); return; }

  // draw center line
  const int cy = y0 + (h/2);
  oled.drawFastHLine(0, cy, OLED_W, SSD1306_WHITE);

  // Determine start index for last 'visible' samples
  int start = scope_idx - visible; while (start < 0) start += SCOPE_SAMPLES;
  int prevx = 0; int prevy = cy;
  for (int i=0;i<visible;i++) {
    int s = scope_buf[(start + i) % SCOPE_SAMPLES];
    // midpoint adjust: map rawM to ADC units and subtract
    int midpoint = (int)((rawM / 4095.0f) * 32767.0f) - 16384;
    int centered = s - midpoint; // ~-32767..+32767
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
Bank bank_util = { "Util", { &patch_clock, &patch_quant, &patch_euclid, &patch_mod, &patch_env, &patch_calib, &patch_scope, &patch_diag }, 8 };
Bank* banks[] = { &bank_util };
static uint8_t bankIdx  = 0;
static uint8_t patchIdx = 0;

// ---- Home menu + input state ----
// Home menu items (4x2 grid viewport). Order updated to the requested first-8 patches.
static const char* kHomeItems[] = { "Clock", "Quant", "Euclid", "LFO", "Env", "Calib", "Scope", "Diag" };
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

