#include <Arduino.h>
#include <SPI.h>
#include <CrashReport.h>

// ----- Additions for OLED + Audio passthrough -----
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Audio.h>

// ---------------- OLED ----------------
// Use Adafruit SSD1306 (I2C)
#define OLED_W 128
#define OLED_H 32
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);

// ---------------- Audio: pure passthrough ----------------
AudioInputI2S        i2sIn;        // SGTL5000 line-in (L/R)
AudioOutputI2S       i2sOut;       // SGTL5000 line-out (L/R)
AudioControlSGTL5000 sgtl5000;
// Wire L->L, R->R
AudioConnection patchCord1(i2sIn, 0, i2sOut, 0);
AudioConnection patchCord2(i2sIn, 1, i2sOut, 1);

// ---------------- Pins (as in your stable MIDI build) ----------------
#define PIN_BTN      2
#define PIN_CS_DAC1 33
#define PIN_CS_DAC2 34
#define PIN_CLOCK   39
#define PIN_RESET   37
#define PIN_GATE1   40
#define PIN_GATE2   38
#define GATE_WRITE(pin, s) digitalWrite((pin), (s)?LOW:HIGH)   // HCT14 invert

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

// ---------- PROTOTYPES (fix compile order) ----------
void onNoteOn(byte ch, byte note, byte vel);
void onNoteOff(byte ch, byte note, byte vel);
void onPitchBend(byte ch, int value);
void onControlChange(byte ch, byte cc, byte val);
void onStart();
void onStop();
void onClock();

// ---------- MCP4822 ----------
enum { CH_A=0, CH_B=1 };
static inline uint16_t frame4822(uint8_t ch, uint16_t v){ return (ch?0x8000:0)|0x1000|(v & 0x0FFF); }
static inline void mcp4822_write(uint8_t cs, uint8_t ch, uint16_t v){
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0)); // 4 MHz for signal integrity
  digitalWrite(cs, LOW);
  SPI.transfer16(frame4822(ch, v));
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}

// ---------- Calibration (your validated values) ----------
const float kPitchSlope  = 5.0f * (20.0f / (22.0f + 20.0f));   // ≈2.381
const float kPitchOffset = -(39.0f/10.0f) * 0.750f;            // ≈-2.925
const float kModSlope    = 5.0f * (20.0f / (8.2f + 20.0f));    // ≈2.439
const float kModOffset   = -(20.0f/8.2f) * 2.050f;             // ≈-5.000

static inline uint16_t pitchVolt_to_code(float vOut){
  float vDac=(vOut-kPitchOffset)/kPitchSlope;
  float code=vDac*(4095.0f/4.096f);
  if(code<0)code=0; else if(code>4095)code=4095;
  return (uint16_t)(code+0.5f);
}
static inline uint16_t modVolt_to_code(float vOut){
  float vDac=(vOut-kModOffset)/kModSlope;
  float code=vDac*(4095.0f/4.096f);
  if(code<0)code=0; else if(code>4095)code=4095;
  return (uint16_t)(code+0.5f);
}

// ---------- State ----------
struct Voice { int8_t note=-1; float bend=0, modV=0, pitchHeldV=0, calib=0; };
static Voice v1, v2;

static inline float midiNote_to_volts(int note){ return (note-36)/12.0f; }
static inline void updatePitch(Voice& v){
  float base=midiNote_to_volts(v.note<0?36:v.note);
  v.pitchHeldV = base + v.bend/12.0f + v.calib;
}

// event flags to minimize SPI
static volatile bool dirtyPitch1=true, dirtyPitch2=true, dirtyMod1=true, dirtyMod2=true;

// realtime outputs
static volatile bool gate1=false, gate2=false, clk=false, rst=false;
static volatile uint32_t clkUntil=0, rstUntil=0;
const uint32_t PULSE_MS = 5;

// button + heartbeat
static uint32_t btnDownAt=0; static bool btnPrev=HIGH; const uint16_t LONG_MS=600;
static uint32_t lastBeat=0;

// ---------------- OLED helpers ----------------
static uint32_t lastOledPaintMs = 0;
const uint32_t OLED_FPS_MS = 80; // ~12.5 Hz
static inline void drawRow(uint8_t row,const char* s){ oled.setCursor(0,row*8); oled.print(s); }
static char lineBuf[64];

// ---------- MIDI callbacks ----------
void onNoteOn(byte ch, byte note, byte vel){
  if(!vel){ onNoteOff(ch, note, 0); return; }
  if(ch==1){
    v1.note=note; v1.modV=5.0f*(vel/127.0f); updatePitch(v1);
    gate1=true; dirtyMod1=true; dirtyPitch1=true;
  } else if(ch==2){
    v2.note=note; v2.modV=5.0f*(vel/127.0f); updatePitch(v2);
    gate2=true; dirtyMod2=true; dirtyPitch2=true;
  }
}
void onNoteOff(byte ch, byte note, byte){
  if(ch==1 && v1.note==note){ gate1=false; v1.note=-1; dirtyPitch1=true; }
  else if(ch==2 && v2.note==note){ gate2=false; v2.note=-1; dirtyPitch2=true; }
}
void onPitchBend(byte ch, int value){
  float semis = 2.0f * (float)(value - 8192) / 8192.0f; // ±2 semis
  if(ch==1){ v1.bend=semis; if(v1.note>=0){ updatePitch(v1); dirtyPitch1=true; } }
  else if(ch==2){ v2.bend=semis; if(v2.note>=0){ updatePitch(v2); dirtyPitch2=true; } }
}
void onControlChange(byte, byte, byte){ /* ignored */ }

void onStart(){ rst = true; rstUntil = millis() + 8; }
void onStop(){ gate1=false; gate2=false; clk=false; rst=false; }
void onClock(){ clk = true; clkUntil = millis() + PULSE_MS; }  // 24 PPQN

// ---------- Setup ----------
void setup(){
  // Crash report (if last boot faulted)
  if (CrashReport) {
    while (!Serial && millis() < 1500) {}
    Serial.print(CrashReport);
  }

  pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, LOW);

  pinMode(PIN_BTN,INPUT_PULLUP);

  pinMode(PIN_CS_DAC1,OUTPUT); digitalWrite(PIN_CS_DAC1,HIGH);
  pinMode(PIN_CS_DAC2,OUTPUT); digitalWrite(PIN_CS_DAC2,HIGH);

  pinMode(PIN_CLOCK,OUTPUT); pinMode(PIN_RESET,OUTPUT);
  pinMode(PIN_GATE1,OUTPUT); pinMode(PIN_GATE2,OUTPUT);
  GATE_WRITE(PIN_CLOCK,false); GATE_WRITE(PIN_RESET,false);
  GATE_WRITE(PIN_GATE1,false); GATE_WRITE(PIN_GATE2,false);

  SPI.begin();

  // Safe defaults
  mcp4822_write(PIN_CS_DAC1, CH_A, modVolt_to_code(0.0f));   // Mod1
  mcp4822_write(PIN_CS_DAC1, CH_B, pitchVolt_to_code(0.0f)); // Pitch1
  mcp4822_write(PIN_CS_DAC2, CH_A, modVolt_to_code(0.0f));   // Mod2
  mcp4822_write(PIN_CS_DAC2, CH_B, pitchVolt_to_code(0.0f)); // Pitch2

  // --- AUDIO passthrough init (minimal) ---
  AudioMemory(8);                           // small, low-latency
  sgtl5000.enable();
  sgtl5000.inputSelect(AUDIO_INPUT_LINEIN); // line in (not mic)
  sgtl5000.adcHighPassFilterDisable();      // cleanest path
  sgtl5000.lineInLevel(6);                  // raise if source is quiet (0..15)
  sgtl5000.lineOutLevel(29);                // 13..31
  sgtl5000.volume(0.8f);                    // headphone jack only

  // --- OLED init ---
  Wire.begin();
  if(!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    // OLED init failed; continue without display
  } else {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0,0);
    oled.display();
  }

  // MIDI handlers (callback style)
  usbMIDI.setHandleNoteOn(onNoteOn);
  usbMIDI.setHandleNoteOff(onNoteOff);
  usbMIDI.setHandlePitchChange(onPitchBend);
  usbMIDI.setHandleControlChange(onControlChange);
  usbMIDI.setHandleStart(onStart);
  usbMIDI.setHandleStop(onStop);
  usbMIDI.setHandleClock(onClock);
}

// ---------- Loop ----------
void loop(){
  // Feed USB MIDI (will call our handlers)
  usbMIDI.read();
  // (Optional) usbMIDI.send_now(); // not needed, but harmless

  // Manual reset button (long press)
  bool b=digitalRead(PIN_BTN);
  if(b!=btnPrev){
    if(b==LOW) btnDownAt=millis();
    else if(millis()-btnDownAt>=LONG_MS){ rst=true; rstUntil=millis()+8; }
    btnPrev=b;
  }

  // End pulses
  uint32_t now = millis();
  if(clkUntil && (int32_t)(now-(int32_t)clkUntil)>=0){ clk=false; clkUntil=0; }
  if(rstUntil && (int32_t)(now-(int32_t)rstUntil)>=0){ rst=false; rstUntil=0; }

  // Drive pins (HCT14 inverts)
  GATE_WRITE(PIN_CLOCK, clk);
  GATE_WRITE(PIN_RESET, rst);
  GATE_WRITE(PIN_GATE1, gate1);
  GATE_WRITE(PIN_GATE2, gate2);

  // Update DACs when dirty (event-driven)
  if(dirtyMod1){ mcp4822_write(PIN_CS_DAC1, CH_A, modVolt_to_code(v1.modV)); dirtyMod1=false; }
  if(dirtyPitch1){ mcp4822_write(PIN_CS_DAC1, CH_B, pitchVolt_to_code(v1.pitchHeldV)); dirtyPitch1=false; }
  if(dirtyMod2){ mcp4822_write(PIN_CS_DAC2, CH_A, modVolt_to_code(v2.modV)); dirtyMod2=false; }
  if(dirtyPitch2){ mcp4822_write(PIN_CS_DAC2, CH_B, pitchVolt_to_code(v2.pitchHeldV)); dirtyPitch2=false; }

  // Heartbeat
  if (now - lastBeat >= 1000) { lastBeat = now; digitalToggle(LED_BUILTIN); }

  // ---- OLED debug (lightweight, ~12.5 Hz) ----
  if (now - lastOledPaintMs >= OLED_FPS_MS) {
    // Compute the *expected* analog outputs based on current DAC codes
    float vP1 = kPitchSlope * ( (pitchVolt_to_code(v1.pitchHeldV) * 4.096f / 4095.0f) ) + kPitchOffset;
    float vP2 = kPitchSlope * ( (pitchVolt_to_code(v2.pitchHeldV) * 4.096f / 4095.0f) ) + kPitchOffset;
    float vM1 = kModSlope   * ( (modVolt_to_code(v1.modV)         * 4.096f / 4095.0f) ) + kModOffset;
    float vM2 = kModSlope   * ( (modVolt_to_code(v2.modV)         * 4.096f / 4095.0f) ) + kModOffset;

    oled.clearDisplay();
    snprintf(lineBuf,sizeof(lineBuf),"THRU CLK:%c G1:%c G2:%c R:%c",
      clk?'#':'-', gate1?'#':'-', gate2?'#':'-', rst?'#':'-');
    drawRow(0,lineBuf);
    snprintf(lineBuf,sizeof(lineBuf),"P1:%+.2fV  P2:%+.2fV", vP1, vP2); drawRow(1,lineBuf);
    snprintf(lineBuf,sizeof(lineBuf),"M1:%+.2fV  M2:%+.2fV", vM1, vM2); drawRow(2,lineBuf);
    drawRow(3,"Btn: long=RESET  24PPQN");
    oled.display();

    lastOledPaintMs = now;
  }
}
