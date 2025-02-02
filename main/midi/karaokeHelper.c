#include "karaokeHelper.h"

#include <esp_heap_caps.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "linked_list.h"
#include "macros.h"

bool analyzeKaraokeFile(karaokeInfo_t* karInfo, const midiFile_t* midiFile)
{
    bool karFormat     = false;
    uint32_t eventMask = (1 << COPYRIGHT) | (1 << SEQUENCE_OR_TRACK_NAME) | (1 << LYRIC) | (1 << TEXT);
    midiFileReader_t reader;

    if (initMidiParser(&reader, midiFile))
    {
        reader.handleMetaEvents = true;
        uint32_t tempo          = 500000;

        midiTextInfo_t* lastInfo = NULL;
        uint8_t textTrack        = 0;
        bool skipOtherTracks     = false;
        midiEvent_t event;
        

        while (midiNextEvent(&reader, &event))
        {
            if (event.type == META_EVENT && event.meta.type < 0xF && 0 != ((1 << event.meta.type) & eventMask))
            {
                if (!karFormat && (event.meta.type == LYRIC || event.meta.type == TEXT) && event.meta.length > 0)
                {
                    char start = event.meta.text[0];
                    char end   = event.meta.text[event.meta.length - 1];
                    if (start == '\\' || end == '\\' || start == '/' || end == '/')
                    {
                        karFormat = true;
                    }
                }

                if (!skipOtherTracks && lastInfo && textTrack != event.track)
                {
                    if (lastInfo->timestamp == event.absTime
                        && !strncmp(lastInfo->text, event.meta.text, MIN(lastInfo->length, event.meta.length)))
                    {
                        printf("Duplicated text events (%s) on channel %" PRIu8 " and %" PRIu8 "! Skipping.\n",
                               event.meta.text, textTrack, event.track);
                        skipOtherTracks = true;
                        continue;
                    }
                }
                else if (skipOtherTracks && event.track != textTrack)
                {
                    continue;
                }

                // TODO we could save a couple bytes if we parsed the file an additional time to check how many events
                // there are in total...
                midiTextInfo_t* info = (midiTextInfo_t*)heap_caps_malloc(sizeof(midiTextInfo_t), MALLOC_CAP_SPIRAM);

                if (info)
                {
                    info->text      = event.meta.text;
                    info->length    = event.meta.length;
                    info->type      = event.meta.type;
                    info->tempo     = tempo;
                    info->timestamp = event.absTime;
                    info->expiration = event.absTime + karInfo->timeSignature.midiClocksPerMetronomeTick;

                    push(&karInfo->lyrics, info);

                    if (event.absTime != 0)
                    {
                        lastInfo  = info;
                        textTrack = event.track;
                    }
                }
            }
            else if (event.type == META_EVENT && event.meta.type == TEMPO)
            {
                tempo = event.meta.tempo;
            }
            else if (event.type == META_EVENT && event.meta.type == TIME_SIGNATURE)
            {
                /*uint8_t beatsPerMeasure = event.meta.timeSignature.numerator;
                // I don't understand how this isn't the same thing as the denominator?
                uint32_t quarterNotesPerBeat = event.meta.timeSignature.num32ndNotesPerBeat / 8;
                //uint32_t typeOfNotes = (1 << event.meta.timeSignature.denominator);
                karInfo->measureLength = beatsPerMeasure;
                karInfo->noteLength = quarterNotesPerBeat;*/

                memcpy(&karInfo->timeSignature, &event.meta.timeSignature, sizeof(midiTimeSignature_t));
            }
            else if (event.type == MIDI_EVENT && ((event.midi.status & 0xF0) == 0x90))
            {
                // Note on
                if (lastInfo->timestamp == event.absTime)
                {
                    lastInfo->
                }
            }
            else if (event.type == MIDI_EVENT && ((event.midi.status & 0xF0) == 0x80))
            {
                // Note off
            }
        }

        deinitMidiParser(&reader);
        karInfo->karFormat = karFormat;

        if (!karInfo->timeSignature.midiClocksPerMetronomeTick)
        {
            karInfo->timeSignature.midiClocksPerMetronomeTick = 24;
        }

        if (!karInfo->timeSignature.num32ndNotesPerBeat)
        {
            karInfo->timeSignature.num32ndNotesPerBeat = 8;
        }

        return (karInfo->lyrics.length > 0);
    }
    else
    {
        return false;
    }
}