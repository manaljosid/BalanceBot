/*
 *  Title: BNO055 Library definitions
 *  Description: A direct rip of the Adafruit BNO055 library, ported to pico sdk
 *  Author: Mani Magnusson
 */

#include <hardware/i2c.h>
#include <hardware/gpio.h>

#include "BNO055.h"

bool BNO055::init(uint8_t pin_sda, uint8_t pin_scl, adafruit_bno055_opmode_t mode) {
    // Initialize i2c
    gpio_set_function(pin_sda, GPIO_FUNC_I2C);
    gpio_set_function(pin_scl, GPIO_FUNC_I2C);
    i2c_init(_i2c, 400000);

    // Try to detect the BNO055 chip
    uint8_t id = readRegister(BNO055_CHIP_ID_ADDR);
    if (id != BNO055_ID) {
        busy_wait_ms(1000);
        id = readRegister(BNO055_CHIP_ID_ADDR);
        if (id != BNO055_ID) return false;
    }

    // Set the mode to config mode
    set_mode(OPERATION_MODE_CONFIG);

    // Reset the BNO
    writeRegister(BNO055_SYS_TRIGGER_ADDR, 0x20);
    busy_wait_ms(30);

    // Read the chip ID again
    while (readRegister(BNO055_CHIP_ID_ADDR) != BNO055_ID) {
        busy_wait_ms(10);
    }
    busy_wait_ms(50);

    // Set power mode to normal
    writeRegister(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
    busy_wait_ms(30);

    // Set page ID to 0
    writeRegister(BNO055_PAGE_ID_ADDR, 0);

    // Set BNO_SYS_TRIGGER_ADDR to 0
    writeRegister(BNO055_SYS_TRIGGER_ADDR, 0x0);
    busy_wait_ms(30);

    // Set the mode to the mode passed
    set_mode(mode);

    // Done
    return true;
}

/// @brief Set the operation mode of the BNO055 sensor
/// @param mode The mode to be set
void BNO055::set_mode(adafruit_bno055_opmode_t mode) {
    _mode = mode;
    writeRegister(BNO055_OPR_MODE_ADDR, _mode);
    busy_wait_ms(30); // Allow the sensor to change mode
}

/// @brief Set the BNO055 to use an external crystal or not
/// @param use True to use an external crystal, false to use internal reference
void BNO055::set_ext_crystal_use(bool use) {
    adafruit_bno055_opmode_t mode_copy = _mode;
    set_mode(OPERATION_MODE_CONFIG);
    busy_wait_ms(30);
    
    writeRegister(BNO055_PAGE_ID_ADDR, 0);
    busy_wait_ms(30);

    if(use) writeRegister(BNO055_SYS_TRIGGER_ADDR, 0x80);
    else    writeRegister(BNO055_SYS_TRIGGER_ADDR, 0x00);
    busy_wait_ms(300);

    set_mode(mode_copy);
}

/// @brief Read the orientation from the sensor
/// @param x Pointer to the location to place the x component
/// @param y Pointer to the location to place the y component
/// @param z Pointer to the location to place the z component
void BNO055::read(float* x, float* y, float* z) {
    uint8_t buffer[6]{0};
    int16_t _x, _y, _z;
    _x = _y = _z = 0;

    readMultiple(BNO055_EULER_H_LSB_ADDR, buffer, 6);

    _x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
    _y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
    _z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

    *x = ((float)_x) / 16.0;
    *y = ((float)_y) / 16.0;
    *z = ((float)_z) / 16.0;
}

/* Private methods */

/// @brief Write data to a register
/// @param address Register address
/// @param data Data to write to register
void BNO055::writeRegister(uint8_t address, uint8_t data) {
    uint8_t buffer[2]{0};
    buffer[0] = address;
    buffer[1] = data;

    i2c_write_timeout_us(_i2c, _address, buffer, 2, false, 10000);
}

/// @brief Read a single register
/// @param address Address to read from
/// @return Data in the register
uint8_t BNO055::readRegister(uint8_t address) {
    uint8_t buffer[1] = { address };
    
    i2c_write_timeout_us(_i2c, _address, buffer, 1, true, 10000);
    busy_wait_us_32(10);

    i2c_read_timeout_us(_i2c, _address, buffer, 1, false, 10000);

    return buffer[0];
}

/// @brief Read multiple contiguous registers
/// @param address Starting address to read from
/// @param data Pointer to data array to put read data into
/// @param len Size of data array, also number of registers to read from
void BNO055::readMultiple(uint8_t address, uint8_t* data, uint8_t len) {
    uint8_t buffer[1] = { address };

    i2c_write_timeout_us(_i2c, _address, buffer, 1, true, 10000);
    busy_wait_us_32(10);

    i2c_read_timeout_us(_i2c, _address, data, len, false, 10000);
}

