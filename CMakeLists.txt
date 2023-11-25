cmake_minimum_required(VERSION 3.13)

# initialize pico-sdk from submodule
# note: this must happen before project()
include(pico-sdk/pico_sdk_init.cmake)

project(pilsbot_indicators_firmware)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project
file(GLOB indicator_LIB
        "lib/*.h"
        "lib/*.c"
)

file(GLOB indicator_SRC
        "src/*.h"
        "src/*.cpp"
)

file(GLOB indicator_PIOS
        "lib/*.pio"
        "src/*.pio"
)
add_executable(indicator
        ${indicator_SRC}
        ${indicator_LIB}
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(indicator
        pico_stdlib
)

# enable USB serial output
pico_enable_stdio_usb(indicator 1)
pico_enable_stdio_uart(indicator 0)

# compile pio file
pico_generate_pio_header(indicator

        ${indicator_PIOS}
        OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated
)
# make hardware pio available for interpreting pio file
target_link_libraries(indicator
        hardware_pio
        hardware_dma
        hardware_irq
        )

pico_set_program_name(indicator "Pilsbot Indicators")
pico_set_program_version(indicator "0.1")

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(indicator)