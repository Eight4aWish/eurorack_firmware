// Simple 2-operator FM oscillator example
// Adapted from the triplesaw Patch SM example

#include "daisy_patch_sm.h"
#include "daisysp.h"

using namespace daisy;
using namespace patch_sm;
using namespace daisysp;

DaisyPatchSM patch;
Oscillator   osc_carrier, osc_mod;

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    patch.ProcessAllControls();

    // Get Coarse, Fine, and V/OCT inputs from hardware
    float knob_coarse = patch.GetAdcValue(CV_1);
    float coarse_tune = fmap(knob_coarse, 12, 84);

    float knob_fine = patch.GetAdcValue(CV_2);
    float fine_tune = fmap(knob_fine, 0, 10);

    float cv_voct = patch.GetAdcValue(CV_5);
    float voct    = fmap(cv_voct, 0, 60);

    /** Convert from MIDI note number to frequency */
    float midi_nn = fclamp(coarse_tune + fine_tune + voct, 0.f, 127.f);
    float carrier_freq  = mtof(midi_nn);

    // CV_3 : modulation index (depth)
    float mod_index = patch.GetAdcValue(CV_3) * 10.0f; // 0..10

    // CV_4 : modulator ratio (0.25..8)
    float ratio = fmap(patch.GetAdcValue(CV_4), 0.25f, 8.0f);

    float mod_freq = carrier_freq * ratio;

    osc_carrier.SetFreq(carrier_freq);
    osc_mod.SetFreq(mod_freq);

    // Process each sample and send to outputs
    for(size_t i = 0; i < size; i++)
    {
        float m = osc_mod.Process() * mod_index * carrier_freq; // deviation in Hz
        osc_carrier.SetFreq(carrier_freq + m);
        float sig = osc_carrier.Process();

        OUT_L[i] = sig;
        OUT_R[i] = sig;
    }
}

int main(void)
{
    patch.Init();

    float sr = patch.AudioSampleRate();

    osc_carrier.Init(sr);
    osc_mod.Init(sr);

    osc_carrier.SetWaveform(Oscillator::WAVE_SIN);
    osc_mod.SetWaveform(Oscillator::WAVE_SIN);

    patch.StartAudio(AudioCallback);
    while(1) {}
}
