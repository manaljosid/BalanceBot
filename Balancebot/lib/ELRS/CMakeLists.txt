add_library(ELRS INTERFACE)

target_sources(ELRS INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/ELRS.cpp
    ${CMAKE_CURRENT_LIST_DIR}/ELRS_types.h
)

target_include_directories(ELRS INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(ELRS INTERFACE hardware_uart hardware_gpio hardware_irq pico_time pico_sync)