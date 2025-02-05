cmake_minimum_required(VERSION 3.13)

project(rfm69_radio_hal C ASM)

add_library(rfm69_radio_hal INTERFACE)

target_sources(rfm69_radio_hal INTERFACE
	src/hal_radio.c
)

target_include_directories(rfm69_radio_hal INTERFACE
	src
)