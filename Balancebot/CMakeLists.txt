cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

set(PICO_BOARD "adafruit_feather_rp2040") # Uncomment if using adafruit feather rp2040 board

project(Balancebot C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(main main.cpp)

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)

pico_add_extra_outputs(main)

add_subdirectory(lib)

target_link_libraries(main
    pico_stdlib
    pico_multicore
    pico_sync
    pico_util
    pico_time
    hardware_gpio
    hardware_i2c
    hardware_uart
    ELRS
    Servo
    BNO055
    PID
    ) # Insert libraries used in here