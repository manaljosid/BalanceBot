/*
 *  Title: ELRS library
 *  Description:
 *  Author: Mani Magnusson
 */
#pragma once
#include <hardware/uart.h>
#include <pico/sync.h>
#include "ELRS_types.h"

class ELRS {
public:
    explicit ELRS() {interrupt_instance = this;};

    void init(
        uart_inst_t* uart,
        uint8_t pin_tx,
        uint8_t pin_rx,
        unsigned int baudrate = 420000,
        unsigned int timeout_treshold = 20
    );

    elrs_channels_t channels;
    raw_packet_t raw_packet_copy;
    bool new_data;

private:
    const uint8_t ELRS_MAX_PACKET_LEN = 64;

    uart_inst_t* _uart;
    int _irq_number;
    unsigned int _timeout_counter = 0;
    unsigned int _timeout_threshold;

    volatile raw_packet_t raw_packet;
    volatile bool data_ready;

    static void irq_handler();
    static ELRS* interrupt_instance;
    void rx_interrrupt_handler();
    
    void parse_raw_packet();
    void copy_raw_packet(volatile raw_packet_t* in, raw_packet_t* out);
    bool checkCRC(uint8_t* data, uint8_t len, uint8_t CRC);
};