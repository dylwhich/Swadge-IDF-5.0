#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "midiFileParser.h"
#include "linked_list.h"

typedef struct
{
    union
    {
        struct
        {
            const char* text;
            uint32_t length;
            uint32_t tempo;
        };
    };

    metaEventType_t type;
    uint64_t timestamp;
    uint64_t expiration;
    uint16_t track;
} midiTextInfo_t;

typedef struct
{
    /// @brief True if the MIDI file's text is in .KAR format
    bool karFormat;

    /// @brief Preloaded list of lyrics, in order
    list_t lyrics;

    /// @brief The file's tempo (TODO it can change)
    uint32_t tempo;

    /// @brief Number of MIDI ticks in a single note (lyric)
    uint32_t noteLength;

    /// @brief The length, in MIDI ticks, of one measure
    uint32_t measureLength;

    /// @brief The track identified as the one containing the lead vocalist notes.
    /// Typically there will be one track where each note corresponds with a new lyric event
    uint16_t leadTrack;

    /// @brief The track identified as the one containing backup vocalist notes
    /// Some songs will have an additional track with lyrics, such as duets or backup vocals
    uint16_t backupTrack;

    midiTimeSignature_t timeSignature;
} karaokeInfo_t;

bool analyzeKaraokeFile(karaokeInfo_t* karInfo, const midiFile_t* midiFile);