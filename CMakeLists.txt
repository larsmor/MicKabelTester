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
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(kabel_tester C CXX ASM)

pico_sdk_init()

# TDR USB serial debug (115200): one line per tdr_measure_stable in TDR view
option(TDR_DEBUG "Enable TDR USB serial debug prints" OFF)
# Calibration USB serial debug (115200): one line per calibrate measure (also on when TDR_DEBUG=ON)
option(CALIB_DEBUG "Enable calibration USB serial debug prints" OFF)

add_executable(kabel_tester 
    src/main.cpp
	src/input.cpp
	src/mic_test.cpp
	src/tdr.cpp
	src/twistcontroller.cpp
	src/ui.cpp
	src/battery.cpp
	src/settings_store.cpp
	)

pico_set_program_name(kabel_tester "kabel_tester")
pico_set_program_version(kabel_tester "0.1")

pico_enable_stdio_uart(kabel_tester 0)
pico_enable_stdio_usb(kabel_tester 1)

target_link_libraries(kabel_tester
    pico_stdlib
	hardware_adc
    hardware_i2c
    hardware_pio
    hardware_clocks
    hardware_flash
)

target_include_directories(kabel_tester PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

if(TDR_DEBUG)
    target_compile_definitions(kabel_tester PRIVATE TDR_DEBUG=1)
endif()
if(CALIB_DEBUG OR TDR_DEBUG)
    target_compile_definitions(kabel_tester PRIVATE CALIB_DEBUG=1)
endif()

pico_add_extra_outputs(kabel_tester)
