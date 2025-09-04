# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")


cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(ov2640_pico_usb_raw C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(ov2640_pico_usb_raw
    src/main.c
    src/camera_capture.pio
)

pico_generate_pio_header(ov2640_pico_usb_raw ${CMAKE_CURRENT_LIST_DIR}/src/camera_capture.pio)

target_include_directories(ov2640_pico_usb_raw PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(ov2640_pico_usb_raw
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_i2c
)

# USB CDC for stdio
pico_enable_stdio_usb(ov2640_pico_usb_raw 1)
pico_enable_stdio_uart(ov2640_pico_usb_raw 0)

pico_add_extra_outputs(ov2640_pico_usb_raw)
