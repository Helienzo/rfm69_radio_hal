cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(hal_radio_rx C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(../../../hal_radio)
add_subdirectory(../../submodules/c_buffer)
add_subdirectory(../../submodules/pi_pico_gpio_hal)
add_subdirectory(../../submodules/rfm69_rp2040)

# Add executable. Default name is the project name, version 0.1

add_executable(hal_radio_rx
    radio_rx.c
)

# pull in common dependencies
target_link_libraries(hal_radio_rx pico_stdlib hardware_spi rfm69_rp2040 c_buffer pi_pico_gpio_hal rfm69_radio_hal)

if (PICO_CYW43_SUPPORTED)
    target_link_libraries(hal_radio_rx pico_cyw43_arch_none)
endif()

# create map/bin/hex file etc.
pico_add_extra_outputs(hal_radio_rx)