# Balance Bot
This is the code for a balancing robot for a mechatronics course. It uses the Pico SDK from Raspberry Pi and is meant to run on an Adafruit Feather RP2040.

The code implements the following:
- Interrupt driven ELRS/CRSF decoding
- BNO055 IMU for orientation (directly ripped off the [Adafruit BNO055 library](https://github.com/adafruit/Adafruit_BNO055))
- Servo library using the pico-sdk to allow writing angles, throttle and microsecond values to servos and ESCs with programmable frequency
- PID controller allowing dervivative filtering and anti-windup clamping
- Multi-core operation running ELRS/CRSF decoding on one core and PID control on the other