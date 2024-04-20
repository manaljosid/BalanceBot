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
#include <BNO055.h>
#include <PID.h>

// Defines
#define THROTTLE_CHANNEL ch0
#define TURNING_CHANNEL ch1
#define ARMING_CHANNEL ch5

#define PID_KP 1.0f
#define PID_KI 5.0f

#define ROLL_VARIABLE angle_y

// Pinouts
const uint8_t pin_left_motor_pwm = 19;
const uint8_t pin_right_motor_pwm = 8;

const uint8_t pin_left_motor_dir = 18;
const uint8_t pin_right_motor_dir = 7;

const uint8_t pin_elrs_tx = 0;
const uint8_t pin_elrs_rx = 1;

const uint8_t pin_scl = 3;
const uint8_t pin_sda = 2;

// Constructors
ELRS* ELRS::interrupt_instance = nullptr; // Necessary for the ELRS interrupts to work
ELRS receiver;
BNO055 bno055(i2c1, BNO055_ADDRESS_A);
PID pid(PID_KP, PID_KI, 0.0f);

Servo left_motor(pin_left_motor_pwm);
Servo right_motor(pin_right_motor_pwm);
Servo left_motor_dir(pin_left_motor_dir);
Servo right_motor_dir(pin_right_motor_dir);

// Global variables and data structures
queue_t rc_queue;
struct repeating_timer timer;
critical_section_t crit_sec;
elrs_channels_t rc_channels;

bool armed = false;
bool last_armed = false;

uint16_t signal_loss_timeout = 0;
float pid_output, left_motor_throttle, right_motor_throttle;
float angle_x, angle_y, angle_z;

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
    left_motor_dir.init();
    right_motor_dir.init();

    busy_wait_ms(3000);

    // Run ESC initialization sequence
    for (unsigned int i = 10; i > 0; i--) {
        left_motor_dir.set_pulse_width(1400 + 10 * i);
        right_motor_dir.set_pulse_width(1400 + 10 * i);
        busy_wait_ms(50);
    }
    left_motor.set_pulse_width(2000);
    right_motor.set_pulse_width(2000);

    busy_wait_ms(3500);

    left_motor.set_pulse_width(1000);
    right_motor.set_pulse_width(1000);

    bno055.init(pin_sda, pin_scl);
    bno055.set_ext_crystal_use(true);

    pid.antiwindup = 1.0f / PID_KI;
    pid.enableAntiwindup = true;
    pid.errorMode = ErrorMode::LINEAR;
    pid.derivativeMode = DerivativeMode::DERIVATIVE_ON_ERROR;
    pid.setpoint = 0.0f;

    // Allow the ESCs to actually enable before continuing
    busy_wait_ms(5000);

    queue_init(&rc_queue, sizeof(rc_channels), 4);
    multicore_launch_core1(core_1_entry);

    add_repeating_timer_us(-1000, repeating_timer_callback, NULL, &timer);
}

void core_0_loop() {
    uint32_t ch0 = (((uint32_t)rc_channels.THROTTLE_CHANNEL) * 1024UL / 1639UL) + 881UL;
    uint32_t ch1 = (((uint32_t)rc_channels.TURNING_CHANNEL) * 1024UL / 1639UL) + 881UL;
    printf("%u, %u, %u, %f, %f, ", ch0, ch1, rc_channels.ARMING_CHANNEL, angle_y, pid_output);
    if (armed) printf("ARMED\n");
    else printf("NOT ARMED\n");
    busy_wait_ms(20);
}
/*
FROM THE LEFT MOTOR:
Clockwise is positive roation on y axis
Positive on left motor is counter clockwise robot rotation

Positive on left motor is clockwise from the motor's perspective
*/

bool repeating_timer_callback(struct repeating_timer* t) {
    if (queue_is_empty(&rc_queue)) signal_loss_timeout++;
    else signal_loss_timeout = 0;

    while(!queue_is_empty(&rc_queue)) {
        queue_try_remove(&rc_queue, &rc_channels);
    }

    bno055.read(&angle_x, &angle_y, &angle_z);

    // Run PID control
    pid_output = pid.update(ROLL_VARIABLE / 180.0f, 0.001f);

    // Make sure the throttle command is positive between 0 and 1
    bool direction = sign(pid_output);
    pid_output = direction ? pid_output : -pid_output;
    pid_output = constrain(pid_output, 0.0f, 1.0f);

    // Arming logic
    armed = rc_channels.ARMING_CHANNEL >= 900;
    if (!armed) {
        pid_output = left_motor_throttle = right_motor_throttle = 0.0f;
    } else if (armed & !last_armed) {
        pid.reset();
    }
    last_armed = armed;

    left_motor_throttle = pid_output;
    right_motor_throttle = pid_output;

    // If 100 ms have passed since last RC signal cut the motors
    if (signal_loss_timeout > 100) {
        // Cut off motors, set counter compare level to 0 - will make ESCs complain
        left_motor.set_raw_level(0);
        right_motor.set_raw_level(0);
        left_motor_dir.set_raw_level(0);
        right_motor_dir.set_raw_level(0);
    } else {
        // Set motor throttle values
        // 0 = 190
        // 1 = 1791
        //pid_output = (((float)rc_channels.ch0) - 190.0f) / 1601.0f;
        left_motor.set_throttle(left_motor_throttle);
        right_motor.set_throttle(right_motor_throttle);
        left_motor_dir.set_pulse_width(direction ? 1600 : 1400);
        right_motor_dir.set_pulse_width(direction ? 1600 : 1400);
    }
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