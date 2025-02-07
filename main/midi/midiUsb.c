#include "midiUsb.h"

#include <esp_log.h>

#include "swadge2024.h"
#include "tinyusb.h"

//==============================================================================
// Defines
//==============================================================================

// Define the total length of the MIDI USB Device Descriptorg
#define MIDI_CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_MIDI_DESC_LEN)

//==============================================================================
// Enums
//==============================================================================

// Interface counter
enum interface_count
{
    ITF_NUM_MIDI = 0,
    ITF_NUM_MIDI_STREAMING,

    ITF_COUNT
};

// USB Endpoint numbers
enum usb_endpoints
{
    // Available USB Endpoints: 5 IN/OUT EPs and 1 IN EP
    EP_EMPTY = 0,
    EPNUM_MIDI,
};

// Static function declarations
static bool handlePacket(midiEvent_t* event, const uint8_t packet[4]);

//==============================================================================
// Variables
//==============================================================================

/**
 * @brief MIDI Device String descriptor
 */
static const char* midiStringDescriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},    // 0: is supported language is English (0x0409)
    "MAGFest",               // 1: Manufacturer
    "Swadge Synthesizer",    // 2: Product
    "123456",                // 3: Serials
    "Swadge MIDI interface", // 4: MIDI
};

/**
 * @brief MIDI Device Config Descriptor
 */
static const uint8_t midiConfigDescriptor[]
    = {TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, MIDI_CONFIG_TOTAL_LEN, 0, 100),
       TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI, 4, EPNUM_MIDI, (0x80 | EPNUM_MIDI), 64)};

static uint8_t sysexBuffer[1024];
static size_t sysexBuflen = 0;

//==============================================================================
// Functions
//==============================================================================

static void setLedsForPacket(const uint8_t* packet, int len)
{
    const led_t ledVals[16] = {
        {.r = 0x7F, .g = 0, .b = 0}, // Light Red
        {.r = 0xFF, .g = 0, .b = 0}, // Red
        {.r = 0x7F, .g = 0x3F, .b = 0}, // Light Orange
        {.r = 0xFF, .g = 0x7F, .b = 0}, // Orange
        {.r = 0x7F, .g = 0x7F, .b = 0}, // Light Yellow
        {.r = 0xFF, .g = 0xFF, .b = 0}, // Yellow
        {.r = 0, .g = 0x7F, .b = 0}, // Light Green
        {.r = 0, .g = 0xFF, .b = 0}, // Green
        {.r = 0, .g = 0x7F, .b = 0x00}, // Light Cyan
        {.r = 0, .g = 0xFF, .b = 0x00}, // Cyan
        {.r = 0, .g = 0, .b = 0x7F}, // Light Blue
        {.r = 0, .g = 0, .b = 0xFF}, // Blue
        {.r = 0x7F, .g = 0, .b = 0x7F}, // Light Magenta
        {.r = 0xFF, .g = 0, .b = 0xFF}, // Magenta
        {.r = 0x7F, .g = 0x7F, .b = 0x7F}, // Light White
        {.r = 0xFF, .g = 0xFF, .b = 0xFF}, // White
    };

    led_t out[8] = {0};
    int offset = 0;

    for (int i = 0; i < len; i++)
    {
        memcpy(&out[i * 2], &ledVals[(packet[i] & 0xF0) >> 4], sizeof(led_t));
        memcpy(&out[i * 2 + 1], &ledVals[packet[i] & 0x0F], sizeof(led_t));
        offset += snprintf(synthDebug + offset, 1024 - offset, "%02" PRIX8 " ", packet[i]);
    }
    setLeds(out, 8);
}

static void writeSysexBuffer(void)
{
    int offset = 0;
    for (int i = 0; i < sysexBuflen; i++)
    {
        offset += snprintf(synthDebug + offset, 1024 - offset, "%02" PRIX8 " ", sysexBuffer[i]);
    }
}

/**
 * @brief Attempt to convert a TinyUSB MIDI packet to a midiEvent_t
 *
 * @param[out] event A pointer to the MIDI event to update
 * @param[in] packet The TinyUSB MIDI packet to convert
 * @return true if an event was written
 * @return false if the packet could not be parsed into an event
 */
static bool handlePacket(midiEvent_t* event, const uint8_t packet[4])
{
    uint8_t header = packet[0];
    uint8_t cmd    = packet[1];

    switch (header)
    {

        // No MIDI data
        case 0x0:
        setLedsForPacket(NULL, 0);
            return false;

        // Statuses with two data bytes
        case 0x8: // Note OFF
        case 0x9: // Note ON
        case 0xA: // AfterTouch
        case 0xB: // Control Change
        case 0xE: // Pitch bend
        {
            sysexBuflen = 0;

            event->type         = MIDI_EVENT;
            event->midi.status  = cmd;
            event->midi.data[0] = packet[2];
            event->midi.data[1] = packet[3];
            setLedsForPacket(packet, 4);
            return true;
        }

        // Statuses with one data byte
        case 0xC: // Program Select
        case 0xD: // Channel Pressure
        {
            sysexBuflen = 0;

            event->type         = MIDI_EVENT;
            event->midi.status  = cmd;
            event->midi.data[0] = packet[2];
            setLedsForPacket(packet, 3);
            return true;
        }

        // SysEx starts or continue
        case 0x4:
        {
            if (packet[1] == 0xF0)
            {
                sysexBuflen = 0;
            }

            if (sysexBuflen + 3 <= sizeof(sysexBuffer))
            {
                memcpy(sysexBuffer + sysexBuflen, packet + 1, 3);
                sysexBuflen += 3;
                writeSysexBuffer();
            }
            return false;
        }

        // SysEx FINISH
        // SysEx ends with 1 data, or 1 byte system common message
        case 0x5:
        case 0x6:
        case 0x7:
        {
            if (packet[1] == 0xF0)
            {
                sysexBuflen = 0;
            }

            int len = header - 0x04;

            if (sysexBuflen + len <= sizeof(sysexBuffer))
            {
                memcpy(sysexBuffer + sysexBuflen, packet + 1, len);
                sysexBuflen += len;
                writeSysexBuffer();

                event->type         = SYSEX_EVENT;
                event->sysex.data   = sysexBuffer + 1;
                event->sysex.length = sysexBuflen - 2;
                event->sysex.prefix = 0;

                if (sysexBuflen > 1)
                {
                    uint16_t manufacturer = sysexBuffer[1];
                    if (!manufacturer)
                    {
                        if (sysexBuflen > 3)
                        {
                            // A manufacturer ID of 0 means the ID is actually in the next 2 bytes
                            manufacturer = sysexBuffer[2];
                            manufacturer <<= 7;
                            manufacturer |= sysexBuffer[3];
                            //event->sysex.data = &sysexBuffer[3];
                            //event->sysex.length = sysexBuflen - 3;
                        }
                    }
                    else
                    {
                        // Technically 0x00 0x00 0x41 is considered a different manufacturer from the single-byte value 0x41
                        // So in that case just put a 1 in the 15th bit that's otherwise unused
                        manufacturer |= (1 << 15);
                        //event->sysex.data = &sysexBuffer[1];
                        //event->sysex.length = sysexBuflen - 1;
                    }

                    event->sysex.manufacturerId = manufacturer;
                }
                else
                {
                    event->sysex.manufacturerId = 0x00;
                }
            }
            else
            {
                ESP_LOGE("MIDI-USB", "SysEx buffer full! Cannot handle remaining sysex data");
            }
            //setLedsForPacket(packet, 4);
            return true;
        }

        default:
        {
            sysexBuflen = 0;

            setLedsForPacket(packet, 1);
            // Idk?
            return false;
        }
    }
}

bool usbMidiCallback(midiEvent_t* event)
{
    uint8_t packet[4] = {0, 0, 0, 0};
    while (tud_ready() && tud_midi_available())
    {
        
        if (tud_midi_packet_read(packet))
        {
            if (packet[0])
            {
                return handlePacket(event, packet);
            }
        }
    }

    return false;
}

bool usbMidiSend(midiEvent_t* event)
{
    uint8_t smallBuffer[3];
    
    if (!tud_ready())
    {
        return false;
    }

    switch (event->type)
    {
        case MIDI_EVENT:
        {
/*            switch (event->midi.status & 0xF0)
            {
                case 0x80: // Note OFF
                case 0x90: // Note ON
                case 0xA0: // AfterTouch
                case 0xB0: // Control Change
                case 0xE0: // Pitch bend
                {
                    // 2 Data Bytes
                    
                    smallBuffer[0] = event->midi.status;
                    smallBuffer[1] = event->midi.data[0];
                    smallBuffer[2] = event->midi.data[1];

                    break;
                }

                case 0xC0: // Program select
                case 0xD0: // Channel pressure
                {
                    smallBuffer[0] = event->midi.status;
                    smallBuffer[1] = event->midi.data[0];
                    // 1 Data Byte
                    break;
                }
            }*/
            uint32_t len = midiWriteEvent(smallBuffer, sizeof(smallBuffer), event);
            return (len == tud_midi_stream_write(0, smallBuffer, len));
        }

        case SYSEX_EVENT:
        {
            smallBuffer[0] = 0xF0;
            if (event->sysex.prefix)
            {
                smallBuffer[0] = event->sysex.prefix;
            }
            smallBuffer[1] = 0xF7;
            int len = tud_midi_stream_write(0, smallBuffer, 1);
            len += tud_midi_stream_write(0, event->sysex.data, event->sysex.length);
            len += tud_midi_stream_write(0, &smallBuffer[1], 1);
            return len == (event->sysex.length + 2);
        }

        case META_EVENT:
        {
            // These cannot be sent over USB!
            return false;
        }
    }

    return false;
}

bool installMidiUsb(void)
{
    tinyusb_config_t const tusb_cfg = {
        .device_descriptor        = NULL,
        .string_descriptor        = midiStringDescriptor,
        .string_descriptor_count  = sizeof(midiStringDescriptor) / sizeof(midiStringDescriptor[0]),
        .external_phy             = false,
        .configuration_descriptor = midiConfigDescriptor,
    };
    esp_err_t result = tinyusb_driver_install(&tusb_cfg);

    if (result != ESP_OK)
    {
        ESP_LOGE("MIDI-USB", "Cannot install MIDI USB driver: error %d", result);

        return false;
    }

    return true;
}
