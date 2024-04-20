add_library(BNO055 INTERFACE)

target_sources(BNO055 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BNO055.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BNO055_regs.h
)

target_include_directories(BNO055 INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(BNO055 INTERFACE hardware_i2c hardware_gpio pico_time)