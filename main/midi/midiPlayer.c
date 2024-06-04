#include "midiPlayer.h"

#include <string.h>
#include <inttypes.h>

#include "waveTables.h"
#include "midiNoteFreqs.h"
#include "hdw-dac.h"
#include "fp_math.h"
#include "esp_log.h"
#include "drums.h"
#include "macros.h"

#define OSC_DITHER

//==============================================================================
// Generated by tools/midi/freqs.py
//==============================================================================
static const uq16_16 noteFreqTable[] = {
    FREQ_C_MINUS_2,
    FREQ_C_SHARP_MINUS_2,
    FREQ_D_MINUS_2,
    FREQ_D_SHARP_MINUS_2,
    FREQ_E_MINUS_2,
    FREQ_F_MINUS_2,
    FREQ_F_SHARP_MINUS_2,
    FREQ_G_MINUS_2,
    FREQ_G_SHARP_MINUS_2,
    FREQ_A_MINUS_1,
    FREQ_A_SHARP_MINUS_1,
    FREQ_B_MINUS_1,
    FREQ_C_MINUS_1,
    FREQ_C_SHARP_MINUS_1,
    FREQ_D_MINUS_1,
    FREQ_D_SHARP_MINUS_1,
    FREQ_E_MINUS_1,
    FREQ_F_MINUS_1,
    FREQ_F_SHARP_MINUS_1,
    FREQ_G_MINUS_1,
    FREQ_G_SHARP_MINUS_1,
    FREQ_A0,
    FREQ_A_SHARP_0,
    FREQ_B0,
    FREQ_C0,
    FREQ_C_SHARP_0,
    FREQ_D0,
    FREQ_D_SHARP_0,
    FREQ_E0,
    FREQ_F0,
    FREQ_F_SHARP_0,
    FREQ_G0,
    FREQ_G_SHARP_0,
    FREQ_A1,
    FREQ_A_SHARP_1,
    FREQ_B1,
    FREQ_C1,
    FREQ_C_SHARP_1,
    FREQ_D1,
    FREQ_D_SHARP_1,
    FREQ_E1,
    FREQ_F1,
    FREQ_F_SHARP_1,
    FREQ_G1,
    FREQ_G_SHARP_1,
    FREQ_A2,
    FREQ_A_SHARP_2,
    FREQ_B2,
    FREQ_C2,
    FREQ_C_SHARP_2,
    FREQ_D2,
    FREQ_D_SHARP_2,
    FREQ_E2,
    FREQ_F2,
    FREQ_F_SHARP_2,
    FREQ_G2,
    FREQ_G_SHARP_2,
    FREQ_A3,
    FREQ_A_SHARP_3,
    FREQ_B3,
    FREQ_C3,
    FREQ_C_SHARP_3,
    FREQ_D3,
    FREQ_D_SHARP_3,
    FREQ_E3,
    FREQ_F3,
    FREQ_F_SHARP_3,
    FREQ_G3,
    FREQ_G_SHARP_3,
    FREQ_A4,
    FREQ_A_SHARP_4,
    FREQ_B4,
    FREQ_C4,
    FREQ_C_SHARP_4,
    FREQ_D4,
    FREQ_D_SHARP_4,
    FREQ_E4,
    FREQ_F4,
    FREQ_F_SHARP_4,
    FREQ_G4,
    FREQ_G_SHARP_4,
    FREQ_A5,
    FREQ_A_SHARP_5,
    FREQ_B5,
    FREQ_C5,
    FREQ_C_SHARP_5,
    FREQ_D5,
    FREQ_D_SHARP_5,
    FREQ_E5,
    FREQ_F5,
    FREQ_F_SHARP_5,
    FREQ_G5,
    FREQ_G_SHARP_5,
    FREQ_A6,
    FREQ_A_SHARP_6,
    FREQ_B6,
    FREQ_C6,
    FREQ_C_SHARP_6,
    FREQ_D6,
    FREQ_D_SHARP_6,
    FREQ_E6,
    FREQ_F6,
    FREQ_F_SHARP_6,
    FREQ_G6,
    FREQ_G_SHARP_6,
    FREQ_A7,
    FREQ_A_SHARP_7,
    FREQ_B7,
    FREQ_C7,
    FREQ_C_SHARP_7,
    FREQ_D7,
    FREQ_D_SHARP_7,
    FREQ_E7,
    FREQ_F7,
    FREQ_F_SHARP_7,
    FREQ_G7,
    FREQ_G_SHARP_7,
    FREQ_A8,
    FREQ_A_SHARP_8,
    FREQ_B8,
    FREQ_C8,
    FREQ_C_SHARP_8,
    FREQ_D8,
    FREQ_D_SHARP_8,
    FREQ_E8,
    FREQ_F8,
    FREQ_F_SHARP_8,
    FREQ_G8,
};

// Multiply a frequency by this value to bend it by a number of cents
// uint32_t bentPitch = (uint32_t)(((uint64_t)pitch * bendTable[bendCents + 100]) >> 24)
static const uq24_8 bendTable[] = {
    0x00f1a1bf, // -100 cents => 0.94387
    0x00f1c57d, // -99 cents => 0.94442
    0x00f1e940, // -98 cents => 0.94497
    0x00f20d08, // -97 cents => 0.94551
    0x00f230d5, // -96 cents => 0.94606
    0x00f254a8, // -95 cents => 0.94660
    0x00f27880, // -94 cents => 0.94715
    0x00f29c5e, // -93 cents => 0.94770
    0x00f2c040, // -92 cents => 0.94825
    0x00f2e428, // -91 cents => 0.94879
    0x00f30816, // -90 cents => 0.94934
    0x00f32c08, // -89 cents => 0.94989
    0x00f35000, // -88 cents => 0.95044
    0x00f373fe, // -87 cents => 0.95099
    0x00f39800, // -86 cents => 0.95154
    0x00f3bc08, // -85 cents => 0.95209
    0x00f3e015, // -84 cents => 0.95264
    0x00f40428, // -83 cents => 0.95319
    0x00f42840, // -82 cents => 0.95374
    0x00f44c5d, // -81 cents => 0.95429
    0x00f47080, // -80 cents => 0.95484
    0x00f494a8, // -79 cents => 0.95539
    0x00f4b8d5, // -78 cents => 0.95595
    0x00f4dd08, // -77 cents => 0.95650
    0x00f50140, // -76 cents => 0.95705
    0x00f5257d, // -75 cents => 0.95760
    0x00f549c0, // -74 cents => 0.95816
    0x00f56e08, // -73 cents => 0.95871
    0x00f59255, // -72 cents => 0.95926
    0x00f5b6a8, // -71 cents => 0.95982
    0x00f5db00, // -70 cents => 0.96037
    0x00f5ff5e, // -69 cents => 0.96093
    0x00f623c1, // -68 cents => 0.96148
    0x00f64829, // -67 cents => 0.96204
    0x00f66c97, // -66 cents => 0.96259
    0x00f6910a, // -65 cents => 0.96315
    0x00f6b582, // -64 cents => 0.96371
    0x00f6da00, // -63 cents => 0.96426
    0x00f6fe84, // -62 cents => 0.96482
    0x00f7230c, // -61 cents => 0.96538
    0x00f7479a, // -60 cents => 0.96594
    0x00f76c2e, // -59 cents => 0.96649
    0x00f790c7, // -58 cents => 0.96705
    0x00f7b565, // -57 cents => 0.96761
    0x00f7da09, // -56 cents => 0.96817
    0x00f7feb2, // -55 cents => 0.96873
    0x00f82361, // -54 cents => 0.96929
    0x00f84815, // -53 cents => 0.96985
    0x00f86cce, // -52 cents => 0.97041
    0x00f8918d, // -51 cents => 0.97097
    0x00f8b651, // -50 cents => 0.97153
    0x00f8db1b, // -49 cents => 0.97209
    0x00f8ffea, // -48 cents => 0.97265
    0x00f924bf, // -47 cents => 0.97322
    0x00f94999, // -46 cents => 0.97378
    0x00f96e78, // -45 cents => 0.97434
    0x00f9935d, // -44 cents => 0.97490
    0x00f9b848, // -43 cents => 0.97547
    0x00f9dd38, // -42 cents => 0.97603
    0x00fa022d, // -41 cents => 0.97660
    0x00fa2728, // -40 cents => 0.97716
    0x00fa4c28, // -39 cents => 0.97772
    0x00fa712e, // -38 cents => 0.97829
    0x00fa9639, // -37 cents => 0.97885
    0x00fabb4a, // -36 cents => 0.97942
    0x00fae060, // -35 cents => 0.97999
    0x00fb057c, // -34 cents => 0.98055
    0x00fb2a9d, // -33 cents => 0.98112
    0x00fb4fc4, // -32 cents => 0.98169
    0x00fb74f0, // -31 cents => 0.98225
    0x00fb9a21, // -30 cents => 0.98282
    0x00fbbf59, // -29 cents => 0.98339
    0x00fbe495, // -28 cents => 0.98396
    0x00fc09d7, // -27 cents => 0.98453
    0x00fc2f1f, // -26 cents => 0.98509
    0x00fc546c, // -25 cents => 0.98566
    0x00fc79bf, // -24 cents => 0.98623
    0x00fc9f17, // -23 cents => 0.98680
    0x00fcc475, // -22 cents => 0.98737
    0x00fce9d8, // -21 cents => 0.98794
    0x00fd0f41, // -20 cents => 0.98851
    0x00fd34b0, // -19 cents => 0.98909
    0x00fd5a23, // -18 cents => 0.98966
    0x00fd7f9d, // -17 cents => 0.99023
    0x00fda51c, // -16 cents => 0.99080
    0x00fdcaa0, // -15 cents => 0.99137
    0x00fdf02a, // -14 cents => 0.99195
    0x00fe15ba, // -13 cents => 0.99252
    0x00fe3b4f, // -12 cents => 0.99309
    0x00fe60ea, // -11 cents => 0.99367
    0x00fe868a, // -10 cents => 0.99424
    0x00feac30, // -9 cents => 0.99481
    0x00fed1dc, // -8 cents => 0.99539
    0x00fef78d, // -7 cents => 0.99596
    0x00ff1d43, // -6 cents => 0.99654
    0x00ff42ff, // -5 cents => 0.99712
    0x00ff68c1, // -4 cents => 0.99769
    0x00ff8e88, // -3 cents => 0.99827
    0x00ffb455, // -2 cents => 0.99885
    0x00ffda28, // -1 cents => 0.99942
    0x01000000, // +0 cents => 1.00000
    0x010025de, // +1 cents => 1.00058
    0x01004bc1, // +2 cents => 1.00116
    0x010071aa, // +3 cents => 1.00173
    0x01009798, // +4 cents => 1.00231
    0x0100bd8d, // +5 cents => 1.00289
    0x0100e386, // +6 cents => 1.00347
    0x01010986, // +7 cents => 1.00405
    0x01012f8b, // +8 cents => 1.00463
    0x01015595, // +9 cents => 1.00521
    0x01017ba5, // +10 cents => 1.00579
    0x0101a1bb, // +11 cents => 1.00637
    0x0101c7d7, // +12 cents => 1.00696
    0x0101edf8, // +13 cents => 1.00754
    0x0102141f, // +14 cents => 1.00812
    0x01023a4b, // +15 cents => 1.00870
    0x0102607d, // +16 cents => 1.00928
    0x010286b5, // +17 cents => 1.00987
    0x0102acf2, // +18 cents => 1.01045
    0x0102d335, // +19 cents => 1.01104
    0x0102f97e, // +20 cents => 1.01162
    0x01031fcc, // +21 cents => 1.01220
    0x01034620, // +22 cents => 1.01279
    0x01036c7a, // +23 cents => 1.01337
    0x010392d9, // +24 cents => 1.01396
    0x0103b93e, // +25 cents => 1.01455
    0x0103dfa9, // +26 cents => 1.01513
    0x01040619, // +27 cents => 1.01572
    0x01042c8f, // +28 cents => 1.01630
    0x0104530b, // +29 cents => 1.01689
    0x0104798d, // +30 cents => 1.01748
    0x0104a014, // +31 cents => 1.01807
    0x0104c6a1, // +32 cents => 1.01866
    0x0104ed33, // +33 cents => 1.01924
    0x010513cb, // +34 cents => 1.01983
    0x01053a69, // +35 cents => 1.02042
    0x0105610d, // +36 cents => 1.02101
    0x010587b6, // +37 cents => 1.02160
    0x0105ae65, // +38 cents => 1.02219
    0x0105d51a, // +39 cents => 1.02278
    0x0105fbd5, // +40 cents => 1.02337
    0x01062295, // +41 cents => 1.02397
    0x0106495b, // +42 cents => 1.02456
    0x01067027, // +43 cents => 1.02515
    0x010696f8, // +44 cents => 1.02574
    0x0106bdd0, // +45 cents => 1.02633
    0x0106e4ad, // +46 cents => 1.02693
    0x01070b8f, // +47 cents => 1.02752
    0x01073278, // +48 cents => 1.02811
    0x01075966, // +49 cents => 1.02871
    0x0107805a, // +50 cents => 1.02930
    0x0107a754, // +51 cents => 1.02990
    0x0107ce53, // +52 cents => 1.03049
    0x0107f558, // +53 cents => 1.03109
    0x01081c64, // +54 cents => 1.03168
    0x01084374, // +55 cents => 1.03228
    0x01086a8b, // +56 cents => 1.03288
    0x010891a7, // +57 cents => 1.03347
    0x0108b8ca, // +58 cents => 1.03407
    0x0108dff1, // +59 cents => 1.03467
    0x0109071f, // +60 cents => 1.03526
    0x01092e53, // +61 cents => 1.03586
    0x0109558c, // +62 cents => 1.03646
    0x01097ccb, // +63 cents => 1.03706
    0x0109a410, // +64 cents => 1.03766
    0x0109cb5b, // +65 cents => 1.03826
    0x0109f2ac, // +66 cents => 1.03886
    0x010a1a02, // +67 cents => 1.03946
    0x010a415e, // +68 cents => 1.04006
    0x010a68c0, // +69 cents => 1.04066
    0x010a9028, // +70 cents => 1.04126
    0x010ab796, // +71 cents => 1.04186
    0x010adf09, // +72 cents => 1.04247
    0x010b0683, // +73 cents => 1.04307
    0x010b2e02, // +74 cents => 1.04367
    0x010b5587, // +75 cents => 1.04427
    0x010b7d12, // +76 cents => 1.04488
    0x010ba4a2, // +77 cents => 1.04548
    0x010bcc39, // +78 cents => 1.04608
    0x010bf3d5, // +79 cents => 1.04669
    0x010c1b78, // +80 cents => 1.04729
    0x010c4320, // +81 cents => 1.04790
    0x010c6ace, // +82 cents => 1.04850
    0x010c9282, // +83 cents => 1.04911
    0x010cba3c, // +84 cents => 1.04972
    0x010ce1fb, // +85 cents => 1.05032
    0x010d09c1, // +86 cents => 1.05093
    0x010d318c, // +87 cents => 1.05154
    0x010d595d, // +88 cents => 1.05214
    0x010d8135, // +89 cents => 1.05275
    0x010da912, // +90 cents => 1.05336
    0x010dd0f5, // +91 cents => 1.05397
    0x010df8dd, // +92 cents => 1.05458
    0x010e20cc, // +93 cents => 1.05519
    0x010e48c1, // +94 cents => 1.05580
    0x010e70bb, // +95 cents => 1.05641
    0x010e98bc, // +96 cents => 1.05702
    0x010ec0c2, // +97 cents => 1.05763
    0x010ee8cf, // +98 cents => 1.05824
    0x010f10e1, // +99 cents => 1.05885
    0x010f38f9, // +100 cents => 1.05946
};

#ifdef OSC_DITHER
// Apply a random offset to each oscillator to maybe make it less likely for waves to "stack" exactly
static const uint8_t oscDither[] = {
139, 227,   5, 103, 241,  67, 251, 109, 197,  59,
 61,   3,  53, 229, 127,  23,  73, 223,  13,  19,
 47,   7, 181,  37,   2, 239,  29, 113, 167, 131,
 41, 151,  83, 137,  11, 193, 107,  17, 191,  43,
101,  71, 233, 179,  97,  79,  31, 211, 163, 157,
 89, 199, 149, 173,
};
#endif
//==============================================================================
// End generated code section
//==============================================================================

// For MIDI values with coarse and fine bytes, each 7 bits
#define UINT14_MAX (0x3FFF)

/// @brief Convert the sample count to MIDI ticks
#define SAMPLES_TO_MIDI_TICKS(n, tempo, div) ((n) * 1000000 * (div) / DAC_SAMPLE_RATE_HZ / (tempo))

//#define MIDI_MULTI_STATE
#ifdef MIDI_MULTI_STATE
#define VS_ANY(statePtr) ((statePtr)->attack | (statePtr)->decay | (statePtr)->release | (statePtr)->sustain)
#define VS_RELEASABLE(statePtr) ((statePtr)->attack | (statePtr)->decay | (statePtr)->sustain)
#define VS_HOLDABLE(statePtr) VS_RELEASABLE(statePtr)
#else
#define VS_ANY(statePtr) ((statePtr)->on)
#define VS_RELEASABLE(statePtr) ((statePtr)->on)
#define VS_HOLDABLE(statePtr) ((statePtr)->on)
#endif
#define MS_TO_TICKS(ms) ((ms) * DAC_SAMPLE_RATE_HZ / 1000)


static uint32_t allocVoice(midiPlayer_t* player, voiceStates_t* states, uint8_t voiceCount);
static uq16_16 bendPitch(uint8_t noteId, uint16_t pitchWheel);
static void midiGmOn(midiPlayer_t* player);
static int32_t midiSumPercussion(midiPlayer_t* player);
static void handleEvent(midiPlayer_t* player, midiEvent_t* event);

static const midiTimbre_t defaultDrumkitTimbre = {
    .type = NOISE,
    .flags = TF_PERCUSSION,
    .percussion = {
        .playFunc = defaultDrumkitFunc,
        // TODO: Define the data and put it here!
        .data = NULL,
    },
    .envelope = { 0 },
    .name = "Swadge Drums 0",
};

static const midiTimbre_t acousticGrandPianoTimbre = {
    .type = WAVETABLE,
    .flags = TF_NONE,
    .waveIndex = 0,
    .envelope = {
        // TODO: Just realized I forgot how ADSR actually works halfway through writing everything else...
        // So go make sure the rest of everything makes sense, maybe rename everything to {a,d,r}Time and {s}Level for clarity
        // Pretty fast attack
        .attack = MS_TO_TICKS(24),
        // Take a good long while to reach the sustain level
        .decay = MS_TO_TICKS(750),
        // Sustain at about 75% of initial volume
        .sustain = 192,
        // And a not-too-short release time
        .release = MS_TO_TICKS(100),
        // Yup, I'm sure it will sound exactly like a grand piano now!
    },
    .name = "Acoustic Grand Piano",
};

// Check for the first unused note, then try to steal one in order of less to more bad, and return INT32_MAX if none are available
static uint32_t allocVoice(midiPlayer_t* player, voiceStates_t* states, uint8_t voiceCount)
{
#define CHECK_VOICE(val) ((val) ? (__builtin_ctz((val))) : 0)
    uint32_t allStates = VS_ANY(states) | states->held; //states->attack | states->decay | states->release | states->sustain | states->held;

    // Find the first voice index which is not used
    // Oh, this was BACKWARDS
    // I'm always allocating the first voice woops
    // Set up a bitflag which has a 1 set for every voice that is NOT being used
    //                         /- flip the bits so a 1 represents an unused voice and a 0 represents an in-use voice
    //                        /                /- mask the bits to only go up to the number of voices we have
    //                       v                v
    uint32_t unusedVoices = (~allStates) & (0xFFFFFFFFu >> (32 - voiceCount));

    if (unusedVoices != 0)
    {
        // Return whatever the first voice that's not allocated is
        return __builtin_ctz(unusedVoices);
    }
    else
    {
        // Gotta steal a note, so steal the first one
        return 0;
    }

    //return firstFreeIdx;
#undef CHECK_VOICE
}

/**
 * @brief Calculate pitch bend
 *
 * @param noteId The MIDI note ID to bend
 * @param pitchWheel The 14-bit MIDI pitch wheel value
 * @return uint32_t The note frequency as a UQ16.16 value
 */
static uq16_16 bendPitch(uint8_t noteId, uint16_t pitchWheel)
{
    // TODO: Should we have a special case when the pitch wheel is centered? Probably better not to branch at all even if we do a bit of unnecessary math
    // First, convert the pitch wheel value to +/-cents
    int32_t bendCents = (((int16_t)-0x2000) + pitchWheel) * 100 / 0x1FFF;
    uint64_t freq = noteFreqTable[noteId];
    freq *= bendTable[bendCents + 100];
    uq16_16 trimmedFreq = ((freq >> 24) & (0xFFFFFFFFu));
    //ESP_LOGI("MIDI", "Bent note #%d %+dc from freq %08x (%d) to %08x (%d)", noteId, bendCents, noteFreqTable[noteId], noteFreqTable[noteId] >> 16, trimmedFreq, trimmedFreq >> 16);
    return trimmedFreq;
}

/**
 * @brief Activate General MIDI mode for a MIDI player
 *
 * @param player The MIDI player to set to General MIDI mode
 */
static void midiGmOn(midiPlayer_t* player)
{
    bool percOscSetup = false;

    for (uint8_t chanIdx = 0; chanIdx < 16; chanIdx++)
    {
        midiChannel_t* chan = &player->channels[chanIdx];

        chan->volume = UINT14_MAX;
        chan->pitchBend = 0x2000;
        chan->program = 0;

        if (chanIdx == 9)
        {
            // Channel 10 is reserved for percussion.
            chan->percussion = true;

            // TODO: Should we just have a pointer instead? That will work great as long as we don't need to modify the timbre in-place (which MIDI does technically allow)
            memcpy(&chan->timbre, &defaultDrumkitTimbre, sizeof(midiTimbre_t));
        }
        else
        {
            chan->percussion = false;
            memcpy(&chan->timbre, &acousticGrandPianoTimbre, sizeof(midiTimbre_t));
        }

        midiVoice_t* voices = chan->percussion ? player->percVoices : chan->voices;
        uint8_t voiceCount = chan->percussion ? PERCUSSION_VOICES : VOICE_PER_CHANNEL;

        for (uint8_t voiceIdx = 0; voiceIdx < voiceCount; voiceIdx++)
        {
            midiVoice_t* voice = &voices[voiceIdx];
            voice->timbre = &chan->timbre;

            for (uint8_t oscIdx = 0; oscIdx < OSC_PER_VOICE; oscIdx++)
            {
                switch (chan->timbre.type)
                {
                    case WAVETABLE:
                    {
                        swSynthInitOscillatorWave(&voice->oscillators[oscIdx], waveTableFunc, (void*)((uint32_t)(voice->timbre->waveIndex)), 0, 0);
                        // Apply a random offset to the oscillator so that similar waves aren't exactly in sync
                        // TODO figure out if this does literally anything
#ifdef OSC_DITHER
                        voice->oscillators[oscIdx].accumulator.bytes[3] = (oscDither[player->oscillatorCount % ARRAY_SIZE(oscDither)]) & 0xFF;
#endif
                        // Make sure we don't count the percussion oscillators multiple times
                        if (!chan->percussion || !percOscSetup)
                        {
                            player->allOscillators[player->oscillatorCount++] = &voice->oscillators[oscIdx];
                        }
                        break;
                    }

                    case SAMPLE:
                    {
                        // TODO: Sample support
                        // player->allSamplers[player->samplerCount++] = ...
                        break;
                    }

                    case NOISE:
                    {
                        swSynthInitOscillator(&voice->oscillators[oscIdx], SHAPE_NOISE, 0, 0);
                        // Apply a random offset to the oscillator so that similar waves aren't exactly in sync
                        // TODO figure out if this does literally anything
#ifdef OSC_DITHER
                        voice->oscillators[oscIdx].accumulator.bytes[3] = (oscDither[player->oscillatorCount]) & 0xFF;
#endif

                        // Make sure we don't count the percussion oscillators multiple times
                        if (!chan->percussion || !percOscSetup)
                        {
                            player->allOscillators[player->oscillatorCount++] = &voice->oscillators[oscIdx];
                        }
                        break;
                    }
                }
            }

            if (chan->percussion)
            {
                // If this was a percussion channel, then all the percussion oscillators were set up
                percOscSetup = true;
            }
        }
    }
    if (player->oscillatorCount > 0)
    {
        player->oscillatorCount--;
    }
}

static int32_t midiSumPercussion(midiPlayer_t* player)
{
    voiceStates_t* states = &player->percVoiceStates;
    midiVoice_t* voices = player->percVoices;
    uint8_t voiceCount = PERCUSSION_VOICES;

    int32_t sum = 0;

    // Ignore the 'held' flag, this is percussion!
    uint32_t playingVoices = states->on;
    while (playingVoices != 0)
    {
        uint8_t voiceIdx = __builtin_ctz(playingVoices);
        playingVoices &= ~(1 << voiceIdx);

        bool done = false;
        sum += voices[voiceIdx].timbre->percussion.playFunc(
            voices[voiceIdx].note,
            voices[voiceIdx].sampleTick++,
            &done,
            voices[voiceIdx].percScratch,
            voices[voiceIdx].timbre->percussion.data);

        if (done)
        {
            states->on &= ~(1 << voiceIdx);
            voices[voiceIdx].sampleTick = 0;
            memset(voices[voiceIdx].percScratch, 0, 4 * sizeof(uint32_t));
        }
    }

    return sum;
}

static void handleMidiEvent(midiPlayer_t* player, midiStatusEvent_t* event)
{
    if (event->status & 0x80)
    {
        // Normal status message
        uint8_t channel = event->status & 0x0F;
        uint8_t cmd = (event->status >> 4) & 0x0F;

        switch (cmd)
        {
            // Note OFF
            case 0x8:
            {
                uint8_t midiKey = event->data[0];
                uint8_t velocity = event->data[1];
                midiNoteOff(player, channel, midiKey, velocity);
                break;
            }

            // Note ON
            case 0x9:
            {
                uint8_t midiKey = event->data[0];
                uint8_t velocity = event->data[1];
                midiNoteOn(player, channel, midiKey, velocity);
                break;
            }

            // AfterTouch
            case 0xA: break;

            // Control change
            case 0xB:
            {
                uint8_t controlId = event->data[0];
                uint8_t controlVal = event->data[1];
                switch (controlId)
                {
                    // Sustain
                    case 0x40:
                    {
                        midiSustain(player, channel, controlVal);
                        break;
                    }

                    // All sounds off (120)
                    case 0x78:
                    {
                        midiAllSoundOff(player);
                        break;
                    }

                    // All notes off (123)
                    case 0x7B:
                    {
                        midiAllNotesOff(player, channel);
                        break;
                    }

                    default: break;
                }

                break;
            }

            // Program Select
            case 0xC:
            {
                uint8_t program = event->data[0];
                midiSetProgram(player, channel, program);
                break;
            }

            // Channel Pressure
            case 0xD: break;

            // Pitch bend
            case 0xE:
            {
                uint16_t range = ((event->data[1] & 0x7F) << 7) | (event->data[0] & 0x7F);
                midiPitchWheel(player, channel, range);
                break;
            }

            default: break;
        }
    }
    else if (event->status & 0xF0)
    {
        // System Message
    }
}

static void handleMetaEvent(midiPlayer_t* player, midiMetaEvent_t* event)
{
    switch (event->type)
    {
        case SEQUENCE_NUMBER: break;

        // Text events
        case TEXT:
        case COPYRIGHT:
        case SEQUENCE_OR_TRACK_NAME:
        case INSTRUMENT_NAME:
        case LYRIC:
        case MARKER:
        case CUE_POINT:
        {
            // Handle text, if the callback is set
            if (player->textMessageCallback)
            {
                player->textMessageCallback(event->type, event->text);
            }
            break;
        }

        // Obsolete
        case CHANNEL_PREFIX: break;

        case END_OF_TRACK:
        {
            // TODO: handle track end
            break;
        }

        case TEMPO:
        {
            player->tempo = event->tempo;
            break;
        }

        case SMPTE_OFFSET:
        {
            // TODO: Tempo support?
            break;
        }

        // These are informational only, we won't do anything with them here.
        case TIME_SIGNATURE: break;
        case KEY_SIGNATURE: break;

        // None supported
        case PROPRIETARY: break;
    }
}

static void handleSysexEvent(midiPlayer_t* player, midiSysexEvent_t* sysex)
{
    // TODO: Support SysEx commands - find some RGB ones we can yoink
    // Actually we can assign a non-registered control to R, G, and B
    // I think there's enough for every LED too assuming there's still like, 7 or so
    // AND: if possible have a sysex command (hmm) that sets all the LEDs to individual values at once
}

static void handleEvent(midiPlayer_t* player, midiEvent_t* event)
{
    switch (event->type)
    {
        case MIDI_EVENT:
        {
            handleMidiEvent(player, &event->midi);
            break;
        }

        case  META_EVENT:
        {
            handleMetaEvent(player, &event->meta);
            break;
        }

        case SYSEX_EVENT:
        {
            handleSysexEvent(player, &event->sysex);
            break;
        }
    }
}


void midiPlayerInit(midiPlayer_t* player)
{
    // Zero out EVERYTHING
    memset(player, 0, sizeof(midiPlayer_t));

    // We need the tempo to not be zero, so set it to the default of 120BPM until we get a tempo event
    // 120 BPM == 500,000 microseconds per quarter note
    player->tempo = 500000;

    midiGmOn(player);
}

void midiPlayerFillBuffer(midiPlayer_t* player, uint8_t* samples, int16_t len)
{
    // First, do a quick check to see if we'll have to handle an event within (len) samples of now
    // TODO: This goes very wrong if you change the tempo while the song is going.
    bool checkEvents = false;
    if (player->mode == MIDI_FILE)
    {
        if (!player->eventAvailable)
        {
            player->eventAvailable = midiNextEvent(player->reader, &player->pendingEvent);
        }

        if (player->eventAvailable)
        {
            if (player->pendingEvent.absTime <= SAMPLES_TO_MIDI_TICKS(player->sampleCount + len, player->tempo, player->reader->division))
            {
                checkEvents = true;
            }
        }
    }

    for (int16_t n = 0; n < len; n++)
    {
        // Use a while loop since we may need to handle multiple events at the exact same time
        while (checkEvents && player->pendingEvent.absTime <= SAMPLES_TO_MIDI_TICKS(player->sampleCount, player->tempo, player->reader->division))
        {
            // It's time, so handle the event now
            handleEvent(player, &player->pendingEvent);

            // Try and grab the next event, and if we got one, keep checking
            player->eventAvailable = midiNextEvent(player->reader, &player->pendingEvent);
            checkEvents = player->eventAvailable;
        }

        // TODO: Sample support
        // sample += samplerSumSamplers(player->allSamplers, player->samplerCount)
        int32_t sample = swSynthSumOscillators(player->allOscillators, player->oscillatorCount);

        sample += midiSumPercussion(player);

        // multiply by the rock constant, very important
        sample *= 0x6666;
        sample >>= 16;

        if (sample < -128)
        {
            samples[n] = 0;
            player->clipped++;
        }
        else if (sample > 127)
        {
            samples[n] = 255;
            player->clipped++;
        }
        else
        {
            samples[n] = sample + 128;
        }

        player->sampleCount++;
    }
}

void midiAllSoundOff(midiPlayer_t* player)
{
    // TODO: It is unclear whether this applies to every channel or just one
    // Seems like people "agree" it's special and applies to every channel
    // But also people say the spec is deficient in this area.
    // So if it's up to us, let's just do them all!

    for (int chanIdx = 0; chanIdx < 16; chanIdx++)
    {
        midiChannel_t* chan = &player->channels[chanIdx];
        // Here we don't bother to check whether this channel is using its own voices
        // or the percussion voices, since we want to turn them off no matter what.
        // This is because All Sounds Off is often used as a "panic" button to stop any
        // stuck notes. So just in case we get into a bad state where notes are playing
        // but we don't think they are, this will always stop them anyway.
        for (uint8_t voiceIdx = 0; voiceIdx < VOICE_PER_CHANNEL; voiceIdx++)
        {
            // TODO: Maybe move this all into a stopVoice() function
            chan->voices[voiceIdx].transitionTicks = 0;
            chan->voices[voiceIdx].targetVol = 0;
            chan->voiceStates.held = 0;
            chan->voiceStates.on = 0;
            // TODO: Handle samplers
            for (uint8_t oscIdx = 0; oscIdx < OSC_PER_VOICE; oscIdx++)
            {
                swSynthSetVolume(&chan->voices[voiceIdx].oscillators[oscIdx], 0);
                swSynthSetFreqPrecise(&chan->voices[voiceIdx].oscillators[oscIdx], 0);
            }
        }
    }

    for (uint8_t voiceIdx = 0; voiceIdx < PERCUSSION_VOICES; voiceIdx++)
    {
        player->percVoices[voiceIdx].transitionTicks = 0;
        player->percVoiceStates.held = 0;
        player->percVoiceStates.on = 0;
        // TODO: Handle samplers
        for (uint8_t oscIdx = 0; oscIdx < OSC_PER_VOICE; oscIdx++)
        {
            player->percVoices[voiceIdx].targetVol = 0;
            swSynthSetVolume(&player->percVoices[voiceIdx].oscillators[oscIdx], 0);
            swSynthSetFreqPrecise(&player->percVoices[voiceIdx].oscillators[oscIdx], 0);
        }
    }
}


void midiAllNotesOff(midiPlayer_t* player, uint8_t channel)
{
    midiChannel_t* chan = &player->channels[channel];
    voiceStates_t* states = chan->percussion ? &player->percVoiceStates : &chan->voiceStates;

    uint32_t playingVoices = VS_ANY(states) | states->held;
    while (playingVoices != 0)
    {
        // TODO / FIXME: This causes an additional search for the playing channel which is unnecessary
        // Instead, refactor the core of midiNoteOff() into an internal midiVoiceOff() function
        // That's probably a good idea anyway with how complicated midiNoteOff() is getting
        uint8_t voiceIdx = __builtin_ctz(playingVoices);
        midiNoteOff(player, channel, chan->voices[voiceIdx].note, 0x7F);

        playingVoices &= ~(1 << voiceIdx);
    }
}


void midiNoteOn(midiPlayer_t* player, uint8_t chanId, uint8_t note, uint8_t velocity)
{
    if (velocity == 0)
    {
        // MIDI note on with a value of 0 is considered a note off
        midiNoteOff(player, chanId, note, 0x7F);
        return;
    }

    midiChannel_t* chan = &player->channels[chanId];
    // Use the appropriate voice pool for the instrument type
    // Percussion gets its own
    voiceStates_t* states = chan->percussion ? &player->percVoiceStates : &chan->voiceStates;
    midiVoice_t* voices = chan->percussion ? player->percVoices : chan->voices;
    uint8_t voiceCount = chan->percussion ? PERCUSSION_VOICES : VOICE_PER_CHANNEL;
    uint32_t voiceIdx = (chan->timbre.flags & TF_MONO) ? 0 : allocVoice(player, states, voiceCount);

    if (voiceIdx >= voiceCount)
    {
        // no voices available and we couldn't find an appropriate one to steal
        // if this happens often we should just allocate more voices
        // (or make the stealing algorithm always succeed)
        return;
    }

    uint32_t voiceBit = (1 << voiceIdx);

    states->on |= voiceBit;
    voices[voiceIdx].note = note;

    // TODO: Add a note -> voice map in the channel?
    uint8_t targetVol = velocity << 1 | 1;
    voices[voiceIdx].targetVol = targetVol;

    if (chan->timbre.flags & TF_PERCUSSION)
    {
        // TODO: For proper GM percussion support:
        // - basically ignore note off entirely, EXCEPT for short and long whistle
        // - the 3 different hi-hats (open, closed, pedal) should preempt/preclude each other
        // - short and long whistle should also preempt/preclude each other
        voices[voiceIdx].sampleTick = 0;
    }
    else
    {
        switch (chan->timbre.type)
        {
            case WAVETABLE:
            case NOISE:
            {
                // TODO: velocity should affect attack time instead of directly affecting volume
                swSynthSetVolume(&voices[voiceIdx].oscillators[0], targetVol);
                swSynthSetFreqPrecise(&voices[voiceIdx].oscillators[0], bendPitch(note, chan->pitchBend));

                //ESP_LOGI("MIDI", "Velocity is %" PRIu8, velocity << 1);
                //voices[voiceIdx].transitionTicks = chan->timbre.envelope.attack;
                break;
            }

            case SAMPLE:
            {
                // TODO: Sample support!
                break;
            }
        }
    }
}


void midiNoteOff(midiPlayer_t* player, uint8_t channel, uint8_t note, uint8_t velocity)
{
    midiChannel_t* chan = &player->channels[channel];
    voiceStates_t* states = chan->percussion ? &player->percVoiceStates : &chan->voiceStates;
    midiVoice_t* voices = chan->percussion ? player->percVoices : chan->voices;

    // check the bitmaps to see if there's any note to release
    uint32_t playingVoices = VS_ANY(&chan->voiceStates) | chan->voiceStates.held;


    // Find the channel playing this note
    while (playingVoices != 0)
    {
        uint8_t voiceIdx = __builtin_ctz(playingVoices);
        uint32_t voiceBit = (1 << voiceIdx);

        if (voices[voiceIdx].note == note)
        {
            // This is the one we want!

            // Unset the on-ness of this note
            states->on &= ~voiceBit;
            if (chan->held)
            {
                states->held |= voiceBit;
            }
            else
            {
                voices[voiceIdx].targetVol = 0;
                for (uint8_t i = 0; i < OSC_PER_VOICE; i++)
                {
                    swSynthSetVolume(&voices[voiceIdx].oscillators[i], 0);
                }
            }

            return;
        }

        // Move on to the next voice
        playingVoices &= ~voiceBit;
    }
}

void midiSetProgram(midiPlayer_t* player, uint8_t channel, uint8_t program)
{
    // TODO: Handle Set Program
    // TODO: If a channel is set to/unset from percussion make sure we update allOscillators
    player->channels[channel].program = program;

    midiChannel_t* chan = &player->channels[channel];
    midiVoice_t* voices = chan->percussion ? player->percVoices : chan->voices;
    voiceStates_t* states = chan->percussion ? &player->percVoiceStates : &chan->voiceStates;
    uint8_t voiceCount = chan->percussion ? PERCUSSION_VOICES : VOICE_PER_CHANNEL;

    for (uint8_t voiceIdx = 0; voiceIdx < voiceCount; voiceIdx++)
    {
        midiVoice_t* voice = &voices[voiceIdx];

        for (uint8_t oscIdx = 0; oscIdx < OSC_PER_VOICE; oscIdx++)
        {
            swSynthSetVolume(&voice->oscillators[oscIdx], 0);
            swSynthSetWaveFunc(&voice->oscillators[oscIdx], waveTableFunc, (void*)((uint32_t)program));
        }
    }
}


void midiSustain(midiPlayer_t* player, uint8_t channel, uint8_t val)
{
    midiChannel_t* chan = &player->channels[channel];
    bool newHold = MIDI_TO_BOOL(val);

    if (chan->held != newHold)
    {
        voiceStates_t* voiceStates = chan->percussion ? &player->percVoiceStates : &chan->voiceStates;
        midiVoice_t* voices = chan->percussion ? player->percVoices : chan->voices;
        if (newHold)
        {
            // Just set the held state for all the currently on notes.
            voiceStates->held |= voiceStates->on;
        }
        else
        {
            // for now what we do is just, if the note is held and not on, turn it off
            // if the note is on, just unset held
            // We should cancel all the notes which are not currently being held
            uint32_t notesToCancel = voiceStates->held & ~(voiceStates->on);

            // unset the hold flag for all
            // TODO: Isn't this going to always be 0?
            uint32_t newHold = (voiceStates->held & ~notesToCancel);

            while (notesToCancel != 0)
            {
                uint8_t voiceIdx = __builtin_ctz(notesToCancel);
                uint32_t voiceBit = (1 << voiceIdx);

                // unset the note's bit and move on to the next one
                notesToCancel &= ~voiceBit;

                for (uint8_t i = 0; i < OSC_PER_VOICE; i++)
                {
                    swSynthSetVolume(&voices[voiceIdx].oscillators[i], 0);
                }
            }

            voiceStates->held = newHold;
        }
        chan->held = newHold;
    }
}


void midiControlChange(midiPlayer_t* player, uint8_t channel, uint8_t control, uint8_t val)
{
    midiAllNotesOff(player, channel);
    // TODO maybe some sort of resetChannel() function?
}


void midiPitchWheel(midiPlayer_t* player, uint8_t channel, uint16_t value)
{
    // Save the pitch bend value
    player->channels[channel].pitchBend = value;

    // Find all the voices currently sounding and update their frequencies
    uint32_t playingVoices = VS_ANY(&player->channels[channel].voiceStates) | player->channels[channel].voiceStates.held;

    // Thought this was a bug but actually
    while (playingVoices != 0)
    {
        uint8_t voiceIdx = __builtin_ctz(playingVoices);
        uint32_t voiceBit = (1 << voiceIdx);

        for (uint8_t oscIdx = 0; oscIdx < OSC_PER_VOICE; oscIdx++)
        {
            // Apply the pitch bend to all this channel's oscillators
            // TODO: If each voice has multiple oscillators, we would obviously
            // want to be able to control them separately here.
            // Maybe we only apply that for like, chorus?
            swSynthSetFreqPrecise(
                &player->channels[channel].voices[voiceIdx].oscillators[oscIdx],
                bendPitch(player->channels[channel].voices[voiceIdx].note, value));
        }

        // Next!
        playingVoices &= ~voiceBit;
    }
}

void midiSetFile(midiPlayer_t* player, midiFileReader_t* reader)
{
    player->mode = MIDI_FILE;
    player->reader = reader;
}
