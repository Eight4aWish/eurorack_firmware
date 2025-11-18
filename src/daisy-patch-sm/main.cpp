#include <Arduino.h>
#include <DaisyDuino.h>

DaisyHardware hw;

void AudioCallback(float **in, float **out, size_t size) {
  for (size_t i = 0; i < size; i++) {
    out[0][i] = in[0][i];
    out[1][i] = in[1][i];
  }
}

void setup() {
  // NOTE: this is the only difference from Seed
  hw = DAISY.init(DAISY_PATCH_SM, AUDIO_SR_48K);
  DAISY.begin(AudioCallback);
}

void loop() {}
