# Generated Cmake Pico project file

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
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(radio_tx C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()
include(FetchContent)
FetchContent_Declare(
  rfm69_rp2040
  GIT_REPOSITORY https://github.com/Helienzo/rfm69_rp2040.git
  GIT_TAG        main
)
FetchContent_MakeAvailable(rfm69_rp2040)

FetchContent_Declare(
  c_buffer
  GIT_REPOSITORY https://github.com/Helienzo/c_buffer.git
  GIT_TAG        feature/build_and_test
)
FetchContent_MakeAvailable(c_buffer)

FetchContent_Declare(
  pi_pico_gpio_hal
  GIT_REPOSITORY https://github.com/Helienzo/pi_pico_gpio_hal.git
  GIT_TAG        feature/build_and_test
)
FetchContent_MakeAvailable(pi_pico_gpio_hal)


# Add executable. Default name is the project name, version 0.1

add_executable(radio_tx radio_tx.c )

pico_set_program_name(radio_tx "radio_tx")
pico_set_program_version(radio_tx "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(radio_tx 0)
pico_enable_stdio_usb(radio_tx 0)

# Add the standard library to the build
target_link_libraries(radio_tx
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(radio_tx PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(radio_tx 
        hardware_spi
        rfm69_rp2040
        c_buffer
        pi_pico_gpio_hal
        )

pico_add_extra_outputs(radio_tx)

