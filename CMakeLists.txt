cmake_minimum_required(VERSION 3.13)

project(rfm69_radio_hal C ASM)

add_library(rfm69_radio_hal INTERFACE)

target_sources(rfm69_radio_hal INTERFACE
	src/hal_radio.c
)

target_include_directories(rfm69_radio_hal INTERFACE
	src
)

# Option to build standalone executable for testing
option(RFM69_RADIO_HAL_TEST "Build standalone executable for rfm69_radio_hal" OFF)

if(RFM69_RADIO_HAL_TEST)
    set(CMAKE_C_COMPILER gcc)

    # Add standalone executable for testing rfm69_radio_hal
    add_executable(test_rfm69_radio_hal test/test_rfm69_radio_hal.c)

    # Link the rfm69_radio_hal library to the standalone executable
    target_link_libraries(test_rfm69_radio_hal PRIVATE rfm69_radio_hal)

    # Optionally, add any specific compiler options for testing
    target_compile_options(test_rfm69_radio_hal PRIVATE -Wall -Wextra -pedantic)
endif()