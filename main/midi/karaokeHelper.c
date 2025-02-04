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
        uint8_t melodyTrack      = 0;
        bool skipOtherTracks     = false;
        midiEvent_t event;

        midiTextInfo_t* noteStartInfo = NULL;
        midiTextInfo_t* prevStartInfo = NULL;
        int8_t curMelodyNote = -1;
        
        // ok ok we're going to have to do multiple passes
        // it will be fine, karaoke needs some time to load...

        // Pass #1: Get all the lyrics into a list
        // Pass #2: Figure out which tracks could possibly be the melody track
        // Pass #3: Use the note events in the melody track to set the karaoke text times

        uint32_t lyricMatches[midiFile->trackCount];
        uint32_t totalNotes[midiFile->trackCount];
        for (int i = 0; i < midiFile->trackCount; i++)
        {
            lyricMatches[i] = 0;
            totalNotes[i] = 0;
        }

        node_t* textNode = NULL;

        for (int pass = 0; pass < 3; pass++)
        {
            while (midiNextEvent(&reader, &event))
            {
                if (pass == 0)
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
                            info->track = event.track;

                            info->expiration = event.absTime + 1;// + karInfo->timeSignature.midiClocksPerMetronomeTick;

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
                }
                else if (pass == 1 || pass == 2)
                {
                    if (textNode == NULL)
                    {
                        textNode = karInfo->lyrics.first;
                    }

                    midiTextInfo_t* lastInfo = textNode ? (midiTextInfo_t*)textNode->val : NULL;
                    while (lastInfo && lastInfo->timestamp < event.absTime)
                    {
                        textNode = textNode->next;
                        if (textNode != NULL)
                        {
                            lastInfo = (midiTextInfo_t*)textNode->val;
                        }
                        else
                        {
                            lastInfo = NULL;
                        }
                    }                           

                    if (pass == 1)
                    {
                        if (event.type == MIDI_EVENT && ((event.midi.status & 0xF0) == 0x90))
                        {
                            // Note on
                            if (lastInfo && lastInfo->timestamp == event.absTime)
                            {
                                lyricMatches[event.track]++;
                            }
                            totalNotes[event.track]++;
                        }
                    }
                    else if (pass == 2)
                    {
                        // Note on
                        if (event.type == MIDI_EVENT && ((event.midi.status & 0xF0) == 0x90) && event.midi.data[1] != 0)
                        {
                            if (lastInfo && lastInfo->timestamp == event.absTime && event.track == melodyTrack)
                            {
                                textTrack = lastInfo->track;
                                noteStartInfo = lastInfo;
                                curMelodyNote = event.midi.data[0];

                                // Stop the previous note if this one is somehow slower
                                if (prevStartInfo && prevStartInfo->expiration > event.absTime && prevStartInfo->timestamp != event.absTime)
                                {
                                    //printf("Trimming %.*s!\n", prevStartInfo->length, prevStartInfo->text);
                                    prevStartInfo->expiration = event.absTime;
                                }
                            }
                        }
                        else if ((event.type == MIDI_EVENT && ((event.midi.status & 0xF0) == 0x80))
                                || (event.type == MIDI_EVENT && ((event.midi.status & 0xF0) == 0x90) && event.midi.data[1] == 0))
                        {
                            // Note off
                            if (noteStartInfo && event.track == melodyTrack && curMelodyNote == event.midi.data[0])
                            {
                                // Stop the previous note if this one is somehow slower
                                if (prevStartInfo && prevStartInfo->expiration > event.absTime && prevStartInfo->timestamp != event.absTime)
                                {
                                    //printf("Trimming %.*s!\n", prevStartInfo->length, prevStartInfo->text);
                                    prevStartInfo->expiration = event.absTime;
                                }

                                noteStartInfo->expiration = event.absTime;
                                //printf("Last note (%.*s) has length of %"PRIu64"\n", noteStartInfo->length, noteStartInfo->text, noteStartInfo->expiration - noteStartInfo->timestamp);
                                prevStartInfo = noteStartInfo;
                                noteStartInfo = NULL;
                                curMelodyNote = -1;
                            }
                        }
                    }
                }
            }
            textNode = NULL;
            resetMidiParser(&reader);

            if (pass == 1)
            {
                int maxMatches = 0;
                int maxTrack = -1;
                float bestRatio = 0;
                int bestRatioTrack = -1;
                
                for (int i = 0; i < midiFile->trackCount; i++)
                {
                    if (lyricMatches[i] > maxMatches)
                    {
                        maxTrack = i;
                        maxMatches = lyricMatches[i];
                    }

                    float ratio = 1.0 * lyricMatches[i] / totalNotes[i];
                    if (ratio > bestRatio)
                    {
                        bestRatio = ratio;
                        bestRatioTrack = i;
                    }
                }

                if (maxTrack >= 0)
                {
                    //printf("Setting melodyTrack to %d\n", maxTrack);
                    melodyTrack = maxTrack;
                }

                if (bestRatioTrack >= 0)
                {
                    printf("best ratio track is %d, max match track is %d\n", bestRatioTrack, maxTrack);
                    melodyTrack = bestRatioTrack;
                    if (maxTrack >= 0)
                    {
                        printf("best ratio: %.3f, max match track ratio: %.3f\n", bestRatio, 1.0 * lyricMatches[maxTrack] / totalNotes[maxTrack]);
                    }
                }
            }
        }

        for (node_t* node = karInfo->lyrics.first; node != NULL;)
        {
            midiTextInfo_t* textInfo = (midiTextInfo_t*)node->val;
            if (textInfo->track != textTrack)
            {
                //printf("Trimming text '%.*s'\n", textInfo->length, textInfo->text);
                node_t* tmp = node;
                node = node->next;
                void* data = removeEntry(&karInfo->lyrics, tmp);
                free(data);
                continue;
            }

            node = node->next;
        }

        for (int i = 0; i < midiFile->trackCount; i++)
        {
            //printf("Track %d lyric matches: %" PRIu32 "\n", i, lyricMatches[i]);
        }

        deinitMidiParser(&reader);
        karInfo->karFormat = karFormat;

        karInfo->leadTrack = melodyTrack;

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