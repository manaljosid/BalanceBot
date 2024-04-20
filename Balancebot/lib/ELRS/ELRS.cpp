/*
 *  Title: ELRS library
 *  Description:
 *  Author: Mani Magnusson
 */

#include <hardware/uart.h>
#include <hardware/gpio.h>
#include <cstring>
#include <stdio.h>

#include "ELRS.h"

uint8_t crc_table[] = {
    0x00, 0xd5, 0x7f, 0xaa, 0xfe, 0x2b, 0x81, 0x54,
    0x29, 0xfc, 0x56, 0x83, 0xd7, 0x02, 0xa8, 0x7d,
    0x52, 0x87, 0x2d, 0xf8, 0xac, 0x79, 0xd3, 0x06,
    0x7b, 0xae, 0x04, 0xd1, 0x85, 0x50, 0xfa, 0x2f,
    0xa4, 0x71, 0xdb, 0x0e, 0x5a, 0x8f, 0x25, 0xf0,
    0x8d, 0x58, 0xf2, 0x27, 0x73, 0xa6, 0x0c, 0xd9,
    0xf6, 0x23, 0x89, 0x5c, 0x08, 0xdd, 0x77, 0xa2,
    0xdf, 0x0a, 0xa0, 0x75, 0x21, 0xf4, 0x5e, 0x8b,
    0x9d, 0x48, 0xe2, 0x37, 0x63, 0xb6, 0x1c, 0xc9,
    0xb4, 0x61, 0xcb, 0x1e, 0x4a, 0x9f, 0x35, 0xe0,
    0xcf, 0x1a, 0xb0, 0x65, 0x31, 0xe4, 0x4e, 0x9b,
    0xe6, 0x33, 0x99, 0x4c, 0x18, 0xcd, 0x67, 0xb2,
    0x39, 0xec, 0x46, 0x93, 0xc7, 0x12, 0xb8, 0x6d,
    0x10, 0xc5, 0x6f, 0xba, 0xee, 0x3b, 0x91, 0x44,
    0x6b, 0xbe, 0x14, 0xc1, 0x95, 0x40, 0xea, 0x3f,
    0x42, 0x97, 0x3d, 0xe8, 0xbc, 0x69, 0xc3, 0x16,
    0xef, 0x3a, 0x90, 0x45, 0x11, 0xc4, 0x6e, 0xbb,
    0xc6, 0x13, 0xb9, 0x6c, 0x38, 0xed, 0x47, 0x92,
    0xbd, 0x68, 0xc2, 0x17, 0x43, 0x96, 0x3c, 0xe9,
    0x94, 0x41, 0xeb, 0x3e, 0x6a, 0xbf, 0x15, 0xc0,
    0x4b, 0x9e, 0x34, 0xe1, 0xb5, 0x60, 0xca, 0x1f,
    0x62, 0xb7, 0x1d, 0xc8, 0x9c, 0x49, 0xe3, 0x36,
    0x19, 0xcc, 0x66, 0xb3, 0xe7, 0x32, 0x98, 0x4d,
    0x30, 0xe5, 0x4f, 0x9a, 0xce, 0x1b, 0xb1, 0x64,
    0x72, 0xa7, 0x0d, 0xd8, 0x8c, 0x59, 0xf3, 0x26,
    0x5b, 0x8e, 0x24, 0xf1, 0xa5, 0x70, 0xda, 0x0f,
    0x20, 0xf5, 0x5f, 0x8a, 0xde, 0x0b, 0xa1, 0x74,
    0x09, 0xdc, 0x76, 0xa3, 0xf7, 0x22, 0x88, 0x5d,
    0xd6, 0x03, 0xa9, 0x7c, 0x28, 0xfd, 0x57, 0x82,
    0xff, 0x2a, 0x80, 0x55, 0x01, 0xd4, 0x7e, 0xab,
    0x84, 0x51, 0xfb, 0x2e, 0x7a, 0xaf, 0x05, 0xd0,
    0xad, 0x78, 0xd2, 0x07, 0x53, 0x86, 0x2c, 0xf9
};

/// @brief Initialize the ELRS driver
/// @param uart uart0 or uart1
/// @param pin_tx TX pin
/// @param pin_rx RX pin
/// @param baudrate Baudrate, default 420000
void ELRS::init(uart_inst_t* uart, uint8_t pin_tx, uint8_t pin_rx, unsigned int baudrate, unsigned int timeout_threshold) {
    _uart = uart;
    _timeout_threshold = timeout_threshold;
    new_data = false;

    gpio_set_function(pin_tx, GPIO_FUNC_UART);
    gpio_set_function(pin_rx, GPIO_FUNC_UART);
    uart_init(_uart, baudrate);

    _irq_number = _uart == uart0 ? UART0_IRQ : UART1_IRQ;

    irq_set_exclusive_handler(_irq_number, irq_handler);
    irq_set_enabled(_irq_number, true);
    uart_set_irq_enables(_uart, true, false);
}

void ELRS::irq_handler() {
    interrupt_instance->rx_interrrupt_handler();
}

/// @brief Interrupt handler for handling receiving data
void ELRS::rx_interrrupt_handler() {
    while (uart_is_readable(_uart)) {
        while (uart_getc(_uart) != ELRS_SYNC_BYTE); // Ensure we only start on a sync bytep
        raw_packet.length = uart_getc(_uart);
        if (raw_packet.length > 63) return; // Something failed!
        raw_packet.type = uart_getc(_uart);
        for (unsigned int i = 0; i < raw_packet.length - 2; i++) {
            raw_packet.data[i] = uart_getc(_uart);
        }
        raw_packet.crc = uart_getc(_uart);
    }
    parse_raw_packet();
    return;
}

/// @brief Parse a raw captured packet
void ELRS::parse_raw_packet() {
    copy_raw_packet(&raw_packet, &raw_packet_copy);
    if (!checkCRC(&raw_packet_copy.type, raw_packet_copy.length - 1, raw_packet.crc)) return; 
    switch (raw_packet_copy.type)
    {
    case ELRS_FRAMETYPE::RC_CHANNELS_PACKED:
        if (raw_packet_copy.length != 24) break;
        for (unsigned int i = 0; i < 22; i++) channels.packed[i] = raw_packet_copy.data[i];
        new_data = true;
        break;
    }
}

/// @brief Copy a raw packet from volatile to non-volatile, blocking all IRQ
/// @param in Pointer to volatile raw packet to copy from
/// @param out Pointer to raw packet to copy into
void ELRS::copy_raw_packet(volatile raw_packet_t* in, raw_packet_t* out) {
    out->length = in->length;
    out->type = in->type;
    out->crc = in->crc;
    for (unsigned int i = 0; i < 64; i++) out->data[i] = in->data[i];
}

/// @brief Check if the CRC checksum is correct
/// @param data Pointer to the data to calculate CRC from
/// @param len Length of the data
/// @param CRC CRC byte to compare to
/// @return True if CRC matches, false if not
bool ELRS::checkCRC(uint8_t* data, uint8_t len, uint8_t CRC) {
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc = crc_table[data[i] ^ crc];
    }
    return crc == CRC;
}