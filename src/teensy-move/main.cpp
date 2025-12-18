// Clean version reassembled to restore structure, with expander channels 3-4 integrated and diagnostics removed.

#include <Arduino.h>
#include <SPI.h>
#include <CrashReport.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Audio.h>
#include "spi_bus.h"

#define OLED_W 128
#define OLED_H 32
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);

AudioInputI2S        i2sIn;
AudioOutputI2S       i2sOut;
AudioOutputUSB       usbOut;
AudioControlSGTL5000 sgtl5000;
AudioConnection patchCord1(i2sIn, 0, i2sOut, 0);
AudioConnection patchCord2(i2sIn, 1, i2sOut, 1);
AudioConnection patchUsbL(i2sIn, 0, usbOut, 0);
AudioConnection patchUsbR(i2sIn, 1, usbOut, 1);

#define PIN_BTN      2
#define PIN_CS_DAC1 33
#define PIN_CS_DAC2 34
#define PIN_CLOCK   39
#define PIN_RESET   37
#define PIN_GATE1   40
#define PIN_GATE2   38
#define PIN_595_LATCH 32
#define GATE_WRITE(pin, s) digitalWrite((pin), (s)?LOW:HIGH)   // HCT14 invert

static const uint8_t DRUM_BASE_NOTE = 36;
static const uint8_t DRUM_COUNT = 4;
static const uint32_t DRUM_TRIG_MS = 15;

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

// Prototypes
void onNoteOn(byte ch, byte note, byte vel);
void onNoteOff(byte ch, byte note, byte vel);
void onPitchBend(byte ch, int value);
void onControlChange(byte ch, byte cc, byte val);
void onStart();
void onStop();
void onClock();

// MCP4822
enum { CH_A=0, CH_B=1 };
static inline uint16_t frame4822(uint8_t ch, uint16_t v){ return (ch?0x8000:0)|0x1000|(v & 0x0FFF); }
static inline void mcp4822_write(uint8_t cs, uint8_t ch, uint16_t v){
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(cs, LOW);
  SPI.transfer16(frame4822(ch, v));
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}

// Expander DACs via Q6/Q7 CS
static inline void mcp4822_write_expander(uint8_t whichDac /*0->Q6,1->Q7*/, uint8_t ch, uint16_t v){
  uint8_t img = expanderImage();
  img |= (1u<<ExpanderBits::DAC1_CS) | (1u<<ExpanderBits::DAC2_CS);
  if (whichDac == 0) img &= ~(1u<<ExpanderBits::DAC1_CS); else img &= ~(1u<<ExpanderBits::DAC2_CS);
  expanderWrite(img);
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  SPI.transfer16(frame4822(ch, v));
  SPI.endTransaction();
  img |= (1u<<ExpanderBits::DAC1_CS) | (1u<<ExpanderBits::DAC2_CS);
  expanderWrite(img);
}

// Calibration
const float kPitchSlope  = 5.0f * (20.0f / (22.0f + 20.0f));
const float kPitchOffset = -(39.0f/10.0f) * 0.750f;
const float kModSlope    = 5.0f * (20.0f / (8.2f + 20.0f));
const float kModOffset   = -(20.0f/8.2f) * 2.050f;

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

struct Voice { int8_t note=-1; float bend=0, modV=0, pitchHeldV=0, calib=0; };
static Voice v1, v2, v3, v4;
static inline float midiNote_to_volts(int note){ return (note-36)/12.0f; }
static inline void updatePitch(Voice& v){ float base=midiNote_to_volts(v.note<0?36:v.note); v.pitchHeldV = base + v.bend/12.0f + v.calib; }

// Dirty flags
static volatile bool dirtyPitch1=true, dirtyPitch2=true, dirtyMod1=true, dirtyMod2=true;
static volatile bool dirtyPitch3=true, dirtyPitch4=true, dirtyMod3=true, dirtyMod4=true;

// Realtime outputs
static volatile bool gate1=false, gate2=false, clk=false, rst=false;
static volatile bool gate3=false, gate4=false;
static volatile uint32_t clkUntil=0, rstUntil=0; const uint32_t PULSE_MS=5;

// Drums
static volatile bool drumTrig[DRUM_COUNT] = {false,false,false,false};
static volatile uint32_t drumUntil[DRUM_COUNT] = {0,0,0,0};
static volatile bool drumDirty = false;

// Debug
static volatile uint8_t lastMidiCh=0, lastMidiNote=0, lastMidiVel=0; static volatile uint32_t lastMidiMs=0;

// Button/OLED
static uint32_t btnDownAt=0; static bool btnPrev=HIGH; const uint16_t LONG_MS=600; static uint32_t lastBeat=0;
static uint32_t lastOledPaintMs=0; const uint32_t OLED_FPS_MS=80; static inline void drawRow(uint8_t row,const char* s){ oled.setCursor(0,row*8); oled.print(s); }
static char lineBuf[64];

// MIDI callbacks
void onNoteOn(byte ch, byte note, byte vel){
  lastMidiCh=ch; lastMidiNote=note; lastMidiVel=vel; lastMidiMs=millis();
  if(!vel){ onNoteOff(ch,note,0); return; }
  if(ch==1){ v1.note=note; v1.modV=5.0f*(vel/127.0f); updatePitch(v1); gate1=true; dirtyMod1=true; dirtyPitch1=true; }
  else if(ch==2){ v2.note=note; v2.modV=5.0f*(vel/127.0f); updatePitch(v2); gate2=true; dirtyMod2=true; dirtyPitch2=true; }
  else if(ch==3){ v3.note=note; v3.modV=5.0f*(vel/127.0f); updatePitch(v3); gate3=true; dirtyMod3=true; dirtyPitch3=true; }
  else if(ch==4){ v4.note=note; v4.modV=5.0f*(vel/127.0f); updatePitch(v4); gate4=true; dirtyMod4=true; dirtyPitch4=true; }
  else if(ch==10){ int idx=(int)note-(int)DRUM_BASE_NOTE; if(idx>=0 && idx<(int)DRUM_COUNT){ drumTrig[idx]=true; drumUntil[idx]=millis()+DRUM_TRIG_MS; drumDirty=true; } }
}
void onNoteOff(byte ch, byte note, byte){
  lastMidiCh=ch; lastMidiNote=note; lastMidiVel=0; lastMidiMs=millis();
  if(ch==1 && v1.note==note){ gate1=false; v1.note=-1; dirtyPitch1=true; }
  else if(ch==2 && v2.note==note){ gate2=false; v2.note=-1; dirtyPitch2=true; }
  else if(ch==3 && v3.note==note){ gate3=false; v3.note=-1; dirtyPitch3=true; }
  else if(ch==4 && v4.note==note){ gate4=false; v4.note=-1; dirtyPitch4=true; }
}
void onPitchBend(byte ch, int value){
  float semis=2.0f*(float)(value-8192)/8192.0f;
  if(ch==1){ v1.bend=semis; if(v1.note>=0){ updatePitch(v1); dirtyPitch1=true; } }
  else if(ch==2){ v2.bend=semis; if(v2.note>=0){ updatePitch(v2); dirtyPitch2=true; } }
  else if(ch==3){ v3.bend=semis; if(v3.note>=0){ updatePitch(v3); dirtyPitch3=true; } }
  else if(ch==4){ v4.bend=semis; if(v4.note>=0){ updatePitch(v4); dirtyPitch4=true; } }
}
void onControlChange(byte, byte, byte){ }

// MIDI clock
static volatile uint32_t midiTickCount=0; static const uint8_t PPQN=24, BEAT_DIV=24;
static void resetMidiClockCounter(){ midiTickCount=0; }
void onStart(){ rst=true; rstUntil=millis()+8; resetMidiClockCounter(); }
void onStop(){ gate1=false; gate2=false; clk=false; rst=false; resetMidiClockCounter(); }
void onContinue(){ resetMidiClockCounter(); }
void onClock(){ midiTickCount++; if(midiTickCount % BEAT_DIV == 0){ clk=true; clkUntil=millis()+PULSE_MS; } }

// Setup
void setup(){
  if (CrashReport) { while (!Serial && millis() < 1500) {} Serial.print(CrashReport); }
  pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, LOW);
  pinMode(PIN_BTN,INPUT_PULLUP);
  pinMode(PIN_CS_DAC1,OUTPUT); digitalWrite(PIN_CS_DAC1,HIGH);
  pinMode(PIN_CS_DAC2,OUTPUT); digitalWrite(PIN_CS_DAC2,HIGH);
  pinMode(PIN_CLOCK,OUTPUT); pinMode(PIN_RESET,OUTPUT);
  pinMode(PIN_GATE1,OUTPUT); pinMode(PIN_GATE2,OUTPUT);
  GATE_WRITE(PIN_CLOCK,false); GATE_WRITE(PIN_RESET,false);
  GATE_WRITE(PIN_GATE1,false); GATE_WRITE(PIN_GATE2,false);
  SPI.begin();
  expanderInit(PIN_595_LATCH);
  mcp4822_write(PIN_CS_DAC1, CH_A, modVolt_to_code(0.0f));
  mcp4822_write(PIN_CS_DAC1, CH_B, pitchVolt_to_code(0.0f));
  mcp4822_write(PIN_CS_DAC2, CH_A, modVolt_to_code(0.0f));
  mcp4822_write(PIN_CS_DAC2, CH_B, pitchVolt_to_code(0.0f));
  AudioMemory(16);
  sgtl5000.enable();
  sgtl5000.inputSelect(AUDIO_INPUT_LINEIN);
  sgtl5000.adcHighPassFilterDisable();
  sgtl5000.lineInLevel(6);
  sgtl5000.lineOutLevel(29);
  sgtl5000.volume(0.8f);
  Wire.begin();
  if(oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE); oled.setCursor(0,0); oled.display();
  }
  usbMIDI.setHandleNoteOn(onNoteOn);
  usbMIDI.setHandleNoteOff(onNoteOff);
  usbMIDI.setHandlePitchChange(onPitchBend);
  usbMIDI.setHandleControlChange(onControlChange);
  usbMIDI.setHandleStart(onStart);
  usbMIDI.setHandleStop(onStop);
  usbMIDI.setHandleClock(onClock);
  usbMIDI.setHandleContinue(onContinue);
}

// Loop
void loop(){
  usbMIDI.read();
  bool b=digitalRead(PIN_BTN);
  if(b!=btnPrev){ if(b==LOW) btnDownAt=millis(); else if(millis()-btnDownAt>=LONG_MS){ rst=true; rstUntil=millis()+8; } btnPrev=b; }
  uint32_t now=millis();
  if(clkUntil && (int32_t)(now-(int32_t)clkUntil)>=0){ clk=false; clkUntil=0; }
  if(rstUntil && (int32_t)(now-(int32_t)rstUntil)>=0){ rst=false; rstUntil=0; }
  for (uint8_t i=0;i<DRUM_COUNT;i++){ if (drumUntil[i] && (int32_t)(now - (int32_t)drumUntil[i]) >= 0) { drumTrig[i]=false; drumUntil[i]=0; drumDirty=true; } }
  GATE_WRITE(PIN_CLOCK, clk); GATE_WRITE(PIN_RESET, rst); GATE_WRITE(PIN_GATE1, gate1); GATE_WRITE(PIN_GATE2, gate2);
  if(dirtyMod1){ mcp4822_write(PIN_CS_DAC1, CH_A, modVolt_to_code(v1.modV)); dirtyMod1=false; }
  if(dirtyPitch1){ mcp4822_write(PIN_CS_DAC1, CH_B, pitchVolt_to_code(v1.pitchHeldV)); dirtyPitch1=false; }
  if(dirtyMod2){ mcp4822_write(PIN_CS_DAC2, CH_A, modVolt_to_code(v2.modV)); dirtyMod2=false; }
  if(dirtyPitch2){ mcp4822_write(PIN_CS_DAC2, CH_B, pitchVolt_to_code(v2.pitchHeldV)); dirtyPitch2=false; }
  if(dirtyMod3){ mcp4822_write_expander(0, CH_A, modVolt_to_code(v3.modV)); dirtyMod3=false; }
  if(dirtyMod4){ mcp4822_write_expander(0, CH_B, modVolt_to_code(v4.modV)); dirtyMod4=false; }
  if(dirtyPitch3){ mcp4822_write_expander(1, CH_A, pitchVolt_to_code(v3.pitchHeldV)); dirtyPitch3=false; }
  if(dirtyPitch4){ mcp4822_write_expander(1, CH_B, pitchVolt_to_code(v4.pitchHeldV)); dirtyPitch4=false; }
  if (now - lastBeat >= 1000) { lastBeat = now; digitalToggle(LED_BUILTIN); }
  // Combined expander image update: gates + drums, keep CS high
  {
    uint8_t img = expanderImage(); uint8_t newImg = img;
    if (gate3) newImg &= ~(1u<<ExpanderBits::V1_GATE); else newImg |= (1u<<ExpanderBits::V1_GATE);
    if (gate4) newImg &= ~(1u<<ExpanderBits::V2_GATE); else newImg |= (1u<<ExpanderBits::V2_GATE);
    uint8_t drumsMask=(1u<<ExpanderBits::DRUM1)|(1u<<ExpanderBits::DRUM2)|(1u<<ExpanderBits::DRUM3)|(1u<<ExpanderBits::DRUM4);
    newImg &= ~drumsMask; for(uint8_t i=0;i<DRUM_COUNT;i++){ if(drumTrig[i]) newImg |= (1u<<(ExpanderBits::DRUM1+i)); }
    newImg |= (1u<<ExpanderBits::DAC1_CS) | (1u<<ExpanderBits::DAC2_CS);
    if(newImg!=img){ expanderWrite(newImg); drumDirty=false; }
  }
  if (now - lastOledPaintMs >= OLED_FPS_MS) {
    float vP1 = kPitchSlope * ( (pitchVolt_to_code(v1.pitchHeldV) * 4.096f / 4095.0f) ) + kPitchOffset;
    float vP2 = kPitchSlope * ( (pitchVolt_to_code(v2.pitchHeldV) * 4.096f / 4095.0f) ) + kPitchOffset;
    float vM1 = kModSlope   * ( (modVolt_to_code(v1.modV)         * 4.096f / 4095.0f) ) + kModOffset;
    float vM2 = kModSlope   * ( (modVolt_to_code(v2.modV)         * 4.096f / 4095.0f) ) + kModOffset;
    oled.clearDisplay();
    snprintf(lineBuf,sizeof(lineBuf),"THRU CLK:%c G1:%c G2:%c R:%c", clk?'#':'-', gate1?'#':'-', gate2?'#':'-', rst?'#':'-'); drawRow(0,lineBuf);
    snprintf(lineBuf,sizeof(lineBuf),"P1:%+.2fV  P2:%+.2fV", vP1, vP2); drawRow(1,lineBuf);
    snprintf(lineBuf,sizeof(lineBuf),"M1:%+.2fV  M2:%+.2fV", vM1, vM2); drawRow(2,lineBuf);
    if (now - lastMidiMs <= 1000) { snprintf(lineBuf,sizeof(lineBuf),"MIDI ch:%2u note:%3u vel:%3u", lastMidiCh,lastMidiNote,lastMidiVel); drawRow(3,lineBuf); }
    else { char d1=drumTrig[0]?'#':'-',d2=drumTrig[1]?'#':'-',d3=drumTrig[2]?'#':'-',d4=drumTrig[3]?'#':'-'; snprintf(lineBuf,sizeof(lineBuf),"CH10 Drums: D1:%c D2:%c D3:%c D4:%c",d1,d2,d3,d4); drawRow(3,lineBuf);}    
    oled.display(); lastOledPaintMs=now;
  }
}
  // Diagnostics removed; normal operation only
