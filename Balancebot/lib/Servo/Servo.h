/*
 *  Title: Servo library
 *  Description:
 *  Author: Mani Magnusson
 */
#pragma once
#include <hardware/pwm.h>
#include <hardware/gpio.h>
#include <hardware/clocks.h>

class Servo {
public:
    Servo() {};
    /// @brief Constructor for the servo object
    /// @param pin GPIO pin the servo is connected to
    Servo(uint8_t pin) : _pin(pin) {};

    void init(uint32_t freq = 50UL);
    float update_frequency(uint32_t new_freq);
    void set_pulse_width(uint16_t microseconds);
    void set_angle(float angle);
    void set_throttle(float throttle);
    void set_raw_level(uint16_t level);
private:
    float _frequency;
    uint8_t _pin;
    uint slice;
    uint channel;
    // The maximum value the PWM slice will go before wrapping, also known as the TOP register
    uint16_t wrapvalue;
};