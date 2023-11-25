cmake_minimum_required(VERSION 3.13)

# initialize pico-sdk from submodule
# note: this must happen before project()
include(pico-sdk/pico_sdk_init.cmake)

project(pilsbot_indicators_firmware)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project

add_executable(hello_world
    src/main.cpp
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(hello_world pico_stdlib)

# enable USB serial output
pico_enable_stdio_usb(hello_world 1)
pico_enable_stdio_uart(hello_world 0)

# compile pio file
pico_generate_pio_header(hello_world ${CMAKE_CURRENT_LIST_DIR}/include/ws2812.pio
        OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/include
)
# make hardware pio available for interpreting pio file
target_link_libraries(hello_world
        hardware_pio
        )

pico_set_program_name(hello_world "Pilsbot Indicators")
pico_set_program_version(hello_world "0.1")

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(hello_world)