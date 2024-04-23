/*
 *  Title: Balance Bot Main code
 *  Description:
 *  Author: Mani Magnusson & Tinna Osk Traustadottir
 * 
 *  Features:
 *   - RC timeout function to cut motor power if connection lost
 *   - RC RSSI/link quality threshold to cut motor power if connection fails
 *   - 
 */

// Includes
#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/sync.h>
#include <pico/multicore.h>
#include <pico/util/queue.h>
#include <pico/time.h>
#include <hardware/i2c.h>
#include <hardware/uart.h>
#include <hardware/gpio.h>
#include <ELRS.h>
#include <Servo.h>

// Defines
#define THROTTLE_CHANNEL ch0
#define TURNING_CHANNEL ch3
#define ARMING_CHANNEL ch5

#define MAX_THROTTLE 0.3f;
#define MAX_TURNING_THROTTLE 0.1f;

#define ARM_DISARM_RAMP_RATE 0.001f;

// Pinouts
const uint8_t pin_left_motor_pwm = 19;
const uint8_t pin_right_motor_pwm = 8;

const uint8_t pin_elrs_tx = 0;
const uint8_t pin_elrs_rx = 1;

// Constructors
ELRS* ELRS::interrupt_instance = nullptr; // Necessary for the ELRS interrupts to work
ELRS receiver;

Servo left_motor(pin_left_motor_pwm);
Servo right_motor(pin_right_motor_pwm);

// Global variables and data structures
queue_t rc_queue;
struct repeating_timer timer;
critical_section_t crit_sec;
elrs_channels_t rc_channels;

bool armed = false;
bool last_armed = false;
bool arm_ramping = false;
bool disarm_ramping = false;

uint16_t signal_loss_timeout = 0;
float throttle_command, left_motor_throttle, right_motor_throttle;
float arming_ramp = 0.0f;

// Forward declarations and utility functions
void print_raw_packet(raw_packet_t packet);
bool repeating_timer_callback(struct repeating_timer* t);

void core_0_init();
void core_0_loop();
void core_1_entry();
void core_1_init();
void core_1_loop();

/// @brief Constrain a value
/// @tparam T The type to be used
/// @param amt Input value
/// @param low Floor value
/// @param high Ceiling value
/// @return Constrained value ranging [low, high]
template <typename T> T constrain(T amt, T low, T high) {
    if(amt < low) return low;
    if(amt > high) return high;
    return amt;
}

/// @brief Get the sign of a value
/// @param num Input value
/// @return True if the number is larger than or equal to zero
bool sign(float num) {
    return num >= 0.0f;
}

void core_0_init() {
    /* TODO:
        - Add initialization for direction pins
        - Add wait for ELRS signal acquisition
        - Add startup sequence for direction pins
    */
    stdio_init_all();

    left_motor.init();
    right_motor.init();
    busy_wait_ms(500);

    // Run ESC initialization sequence
    left_motor.set_pulse_width(2000);
    right_motor.set_pulse_width(2000);
    busy_wait_ms(3500);
    left_motor.set_pulse_width(1000);
    right_motor.set_pulse_width(1000);
    // Allow the ESCs to actually enable before continuing
    busy_wait_ms(5000);

    queue_init(&rc_queue, sizeof(rc_channels), 4);
    multicore_launch_core1(core_1_entry);

    add_repeating_timer_us(-1000, repeating_timer_callback, NULL, &timer);
}

void core_0_loop() {
    uint32_t ch0 = (uint32_t)rc_channels.THROTTLE_CHANNEL; //(((uint32_t)rc_channels.THROTTLE_CHANNEL) * 1024UL / 1639UL) + 881UL;
    uint32_t ch1 = (uint32_t)rc_channels.TURNING_CHANNEL; //(((uint32_t)rc_channels.TURNING_CHANNEL) * 1024UL / 1639UL) + 881UL;
    printf("%u, %u, %u, ", ch0, ch1, rc_channels.ARMING_CHANNEL);
    if (armed) printf("ARMED, ");
    else printf("NOT ARMED, ");
    printf("%f, %f, %f, %f\n", throttle_command, left_motor_throttle, right_motor_throttle, arming_ramp);
    busy_wait_ms(20);
}

bool repeating_timer_callback(struct repeating_timer* t) {
    if (queue_is_empty(&rc_queue)) signal_loss_timeout++;
    else signal_loss_timeout = 0;

    while(!queue_is_empty(&rc_queue)) {
        queue_try_remove(&rc_queue, &rc_channels);
    }
    
    // RC value ranges
    // 0%   = 174
    // 50%  = 992
    // 100% = 1811

    left_motor_throttle = 0.0f;
    right_motor_throttle = 0.0f;

    // Notice there's a deadband between 985 and 995
    if (rc_channels.TURNING_CHANNEL > 994) {
        // Turn right
        left_motor_throttle += ((((float)rc_channels.TURNING_CHANNEL) - 992.0f) / 819.0f) * MAX_TURNING_THROTTLE;
    } else if (rc_channels.TURNING_CHANNEL < 990) {
        // Turn left
        right_motor_throttle -= ((((float)rc_channels.TURNING_CHANNEL) - 992.0f) / 819.0f) * MAX_TURNING_THROTTLE;
    }

    throttle_command = ((((float)rc_channels.THROTTLE_CHANNEL) - 174.0f) / 1637.0f) * MAX_THROTTLE;
    left_motor_throttle += throttle_command;
    right_motor_throttle += throttle_command;

    // Arming logic
    armed = rc_channels.ARMING_CHANNEL >= 900;

    // If 100 ms have passed since last RC signal cut the motors
    if (signal_loss_timeout > 100) {
        armed = false;
    }

    if (armed & !last_armed) {
        // Entering armed state
        arm_ramping = true;
        arming_ramp = 0.0f;
    } else if (armed & last_armed) {
        // In armed state
        if (arm_ramping) {
            if (arming_ramp < 1.0f) {
                left_motor_throttle *= arming_ramp;
                right_motor_throttle *= arming_ramp;
                arming_ramp += ARM_DISARM_RAMP_RATE;
            } else {
                arm_ramping = false;
                arming_ramp = 0.0f;
            }
        }
    } else if (!armed & last_armed) {
        // Exiting armed state
        disarm_ramping = true;
        arming_ramp = 1.0f;
    } else {
        // Not in armed state
        if (disarm_ramping) {
            if (arming_ramp > 0.0f) {
                left_motor_throttle *= arming_ramp;
                right_motor_throttle *= arming_ramp;
                arming_ramp -= ARM_DISARM_RAMP_RATE;
            } else {
                disarm_ramping = false;
                arming_ramp = 0.0f;
            }
        } else {
            left_motor_throttle = 0.0f;
            right_motor_throttle = 0.0f;
        }
    }
    last_armed = armed;
    
    left_motor.set_throttle(left_motor_throttle);
    right_motor.set_throttle(right_motor_throttle);
    return true;
}

void core_1_init() {
    receiver.init(uart0, pin_elrs_tx, pin_elrs_rx);
    critical_section_init(&crit_sec);
}

void core_1_loop() {
    critical_section_enter_blocking(&crit_sec);
    if (receiver.new_data) {
        queue_try_add(&rc_queue, &receiver.channels);
        receiver.new_data = false;
    }
    critical_section_exit(&crit_sec);
}

void core_1_entry() {
    core_1_init();
    while (1) {
        core_1_loop();
    }
}

int main() {
    core_0_init();
    while (1) {
        core_0_loop();
    }
    return 0;
}