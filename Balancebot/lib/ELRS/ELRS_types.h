/*
 *  Title: ELRS types
 *  Description: Map of CRSF protocol fields
 *  Author: Mani Magnusson
 */

#pragma once

#define ELRS_SYNC_BYTE 0xC8
#define ELRS_MAX_CHANNEL 16

namespace ELRS_ADDRESS {
    const uint8_t BROADCAST         = 0x00;
    const uint8_t USB               = 0x10;
    const uint8_t TBS_CORE_PNP_PRO  = 0x80;
    const uint8_t RESERVED1         = 0x8A;
    const uint8_t CURRENT_SENSOR    = 0xC0;
    const uint8_t GPS               = 0xC2;
    const uint8_t TBS_BLACKBOX      = 0xC4;
    const uint8_t FLIGHT_CONTROLLER = 0xC8;
    const uint8_t RESERVED_2        = 0xCA;
    const uint8_t RACE_TAG          = 0xCC;
    const uint8_t RADIO_TRANSMITTER = 0xEA;
    const uint8_t CRSF_RECEIVER     = 0xEC;
    const uint8_t CRSF_TRANSMITTER  = 0xEE;
};

namespace ELRS_FRAMETYPE {
    const uint8_t GPS                       = 0x02;
    const uint8_t VARIO                     = 0x07;
    const uint8_t BATTERY_SENSOR            = 0x08;
    const uint8_t BARO_ALTITUDE             = 0x09;
    const uint8_t OPENTX_SYNC               = 0x10;
    const uint8_t LINK_STATISTICS           = 0x14;
    const uint8_t RC_CHANNELS_PACKED        = 0x16;
    const uint8_t SUBSET_RC_CHANNELS_PACKED = 0x17;
    const uint8_t LINK_STATISTICS_RX        = 0x1C;
    const uint8_t LINK_STATISTICS_TX        = 0x1D;
    const uint8_t ATTITUDE                  = 0x1E;
    const uint8_t FLIGHT_MODE               = 0x21;
    const uint8_t DEVICE_PING               = 0x28;
    const uint8_t DEVICE_INFO               = 0x29;
    const uint8_t REQUEST_SETTINGS          = 0x2A;
    const uint8_t PARAMETER_SETTINGS_ENTRY  = 0x2B;
    const uint8_t PARAMETER_READ            = 0x2C;
    const uint8_t PARAMETER_WRITE           = 0x2D;
    const uint8_t COMMAND                   = 0x32;
    const uint8_t RADIO_ID                  = 0x3A;

    // MSP commands
    const uint8_t MSP_REQ                   = 0x7A; // Reponse request using msp sequence as command
    const uint8_t MSP_RESP                  = 0x7B; // Reply with 58 byte chunked binary
    const uint8_t MSP_WRITE                 = 0x7C; // Write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
    const uint8_t DISPLAYPORT_CMD           = 0x7D; // Displayport control command
};

namespace ELRS_COMMAND_SUBCMD {
    const uint8_t GENERAL                        = 0x0A; // General command
    const uint8_t GENERAL_CRSF_SPEED_PROPOSAL    = 0x70; // Proposed new CRSF port speed
    const uint8_t GENERAL_CRSF_SPEED_RESPONSE    = 0x7a; // Response to the proposed CRSF port speed
};

namespace ELRS_DISPLAYPORT_SUBCMD {
    const uint8_t UPDATE    = 0x01; // Transmit displayport buffer to remote
    const uint8_t CLEAR     = 0x02; // Clear client screen
    const uint8_t OPEN      = 0x03; // Client request to open cms menu
    const uint8_t CLOSE     = 0x04; // Client request to close cms menu
    const uint8_t POLL      = 0x05; // Client request to poll/refresh cms menu
};

namespace ELRS_DISPLAYPORT {
    const uint8_t OPEN_ROWS_OFFSET = 1;
    const uint8_t OPEN_COLS_OFFSET = 2;
};

namespace ELRS_FRAME_PAYLOAD_SIZE {
    const uint8_t GPS               = 15;
    const uint8_t BATTERY_SENSOR    = 8;
    const uint8_t STATISTICS        = 10;
    const uint8_t STATISTICS_TX     = 6;
    const uint8_t RC_CHANNELS       = 22; // 11 bits per channel * 16 channels
    const uint8_t ATTITUDE          = 6;
};

namespace ELRS_FRAME_LENGTH {
    const uint8_t ADDRESS       = 1; // ADDRESS field
    const uint8_t FRAMELENGTH   = 1; // FRAMELENGTH field
    const uint8_t TYPE          = 1; // TYPE field
    const uint8_t CRC           = 1; // CRC field
    const uint8_t TYPE_CRC      = 2; // TYPE and CRC fields combined
    const uint8_t EXT_TYPE_CRC  = 4; // Extended dest/origin, TYPE and CRC fields combined
    const uint8_t NON_PAYLOAD   = 4; // Combined length of all fields except payload
};

namespace ELRS_FRAME_SIZE {
    const uint8_t TX_MSP        = 58;
    const uint8_t RX_MSP        = 8;
    const uint8_t ORIGIN_DEST   = 2;
};

/*
namespace ELRS_HEADER {
    uint8_t DEVICE_ADDR;
    uint8_t FRAME_SIZE;
    uint8_t TYPE;
    uint8_t data[0];
};
*/

struct raw_packet_t {
    uint8_t length;
    uint8_t type;
    uint8_t data[64];
    uint8_t crc;
};

typedef union {
    struct __packed {
        unsigned ch0 : 11;
        unsigned ch1 : 11;
        unsigned ch2 : 11;
        unsigned ch3 : 11;
        unsigned ch4 : 11;
        unsigned ch5 : 11;
        unsigned ch6 : 11;
        unsigned ch7 : 11;
        unsigned ch8 : 11;
        unsigned ch9 : 11;
        unsigned ch10 : 11;
        unsigned ch11 : 11;
        unsigned ch12 : 11;
        unsigned ch13 : 11;
        unsigned ch14 : 11;
        unsigned ch15 : 11;
    };
    uint8_t packed[22];
} elrs_channels_t;