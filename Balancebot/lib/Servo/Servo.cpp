#include "Servo.h"
#include <hardware/gpio.h>
#include <hardware/clocks.h>
#include <hardware/pwm.h>
#include <stdio.h>
#include <math.h>

template <typename T> T constrain(T amt, T low, T high) {
    if(amt < low) return low;
    if(amt > high) return high;
    return amt;
}

/// @brief Initialize the servo, setting hardware configuration
/// @param freq Frequency of the signal to the servo, default 50 Hz
void Servo::init(uint32_t freq) {
    gpio_set_function(_pin, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(_pin);
    channel = pwm_gpio_to_channel(_pin);

    pwm_set_phase_correct(slice, false);

    update_frequency(freq);

    pwm_set_chan_level(slice, channel, 0);

    pwm_set_enabled(slice, true);
}

/// @brief Update the frequency of the signal to the servo
/// @param new_freq New frequency to set
/// @return The actual frequency set
float Servo::update_frequency(uint32_t new_freq) {
    uint32_t sys_clk = clock_get_hz(clk_sys);
    
    wrapvalue = constrain((sys_clk / new_freq) - 1L, 0UL, 0xffffUL);
    pwm_set_wrap(slice, wrapvalue);

    float divider = ((float)sys_clk)/(((float)new_freq) * ((float)wrapvalue));
    uint32_t div_int = floor(divider);
    uint32_t div_frac = floor((divider-div_int)*16);

    pwm_set_clkdiv_int_frac(slice, div_int, div_frac);

    // Finally set the actual frequency
    _frequency = ((float)sys_clk) / (((float)wrapvalue) * (((float)div_int) + ((float)div_frac)));
    return _frequency;
}

/// @brief Set the pulse width to the servo in microseconds
/// @param microseconds Time to keep the pulse high in microseconds
void Servo::set_pulse_width(uint16_t microseconds) {
    uint16_t level = (uint16_t)((((float)wrapvalue) * ((float)microseconds) * _frequency) / 1000000.0f);
    pwm_set_chan_level(slice, channel, level);
}

/// @brief Set the angle of the servo
/// @param angle Angle between 0 and 180
void Servo::set_angle(float angle) {
    angle = constrain(angle, 0.0f, 180.0f);
    uint16_t microseconds = 1000 + (1000.0f * angle / 180.0f);
    set_pulse_width(microseconds);
}

/// @brief Set a throttle level for an ESC
/// @param throttle Throttle level between 0 and 1
void Servo::set_throttle(float throttle) {
    throttle = constrain(throttle, 0.0f, 1.0f);
    uint16_t microseconds = 1000 + (throttle * 1000.0f);
    set_pulse_width(microseconds);
}

/// @brief Set a raw level for the PWM channel. Useful for guaranteeing output will be off
/// @param level Counter compare level for the output compare unit, range [0, TOP+1]
void Servo::set_raw_level(uint16_t level) {
    pwm_set_chan_level(slice, channel, level);
}