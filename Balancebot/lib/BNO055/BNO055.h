/*
 *  Title: BNO055 Library header
 *  Description: A direct rip of the Adafruit BNO055 library, ported to pico sdk
 *  Author: Mani Magnusson
 */

#include <hardware/i2c.h>

#include "BNO055_regs.h"

class BNO055 {
public:
    BNO055() {};
    BNO055(i2c_inst_t* i2c, uint8_t address = BNO055_ADDRESS_A) : _i2c(i2c), _address(address) {};

    bool init(uint8_t pin_sda, uint8_t pin_scl, adafruit_bno055_opmode_t mode = OPERATION_MODE_NDOF);
    void set_mode(adafruit_bno055_opmode_t mode);
    void set_ext_crystal_use(bool use);

    void read(float* x, float* y, float* z);

private:
    i2c_inst_t* _i2c;
    uint8_t _address;

    adafruit_bno055_opmode_t _mode;

    void writeRegister(uint8_t address, uint8_t data);
    uint8_t readRegister(uint8_t address);
    void readMultiple(uint8_t address, uint8_t* data, uint8_t len);
};