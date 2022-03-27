cmake_minimum_required(VERSION 3.13)

include(pico-sdk/pico_sdk_init.cmake)

project(capihm)

pico_sdk_init()

FILE(GLOB capihm_sources
    src/**/*.cpp
    src/*.cpp
)
add_executable(capihm ${capihm_sources})

pico_generate_pio_header(capihm ${CMAKE_CURRENT_LIST_DIR}/src/capsens/capsens.pio)

target_include_directories(capihm PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src/
)

target_link_libraries(capihm
    pico_stdlib
    hardware_pio
    hardware_i2c
)

pico_add_extra_outputs(capihm)