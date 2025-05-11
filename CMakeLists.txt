cmake_minimum_required(VERSION 3.16.0)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(EXTRA_COMPONENT_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}/ascent-drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/ascent-drivers/patch/adc
    ${CMAKE_CURRENT_SOURCE_DIR}/ascent-drivers/comm/i2c
    ${CMAKE_CURRENT_SOURCE_DIR}/ascent-drivers/comm/spi
    ${CMAKE_CURRENT_SOURCE_DIR}/ascent-drivers/psu
    ${CMAKE_CURRENT_SOURCE_DIR}/ascent-drivers/bno055
    ${CMAKE_CURRENT_SOURCE_DIR}/ascent-drivers/w25qxx
    ${CMAKE_CURRENT_SOURCE_DIR}/ascent-drivers/bmp390l
    ${CMAKE_CURRENT_SOURCE_DIR}/ascent-drivers/h3lis331dl
    ${CMAKE_CURRENT_SOURCE_DIR}/ascent-drivers/sam_m10q
    ${CMAKE_CURRENT_SOURCE_DIR}/ascent-drivers/hardware_io/pyro
    ${CMAKE_CURRENT_SOURCE_DIR}/ascent-drivers/indicator
    ${CMAKE_CURRENT_SOURCE_DIR}/ascent-drivers/lora
    ${CMAKE_CURRENT_SOURCE_DIR}/ascent-drivers/neopixel
    ${CMAKE_CURRENT_SOURCE_DIR}/ascent-drivers/wireless
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Inform the build system that your main component lives in the 'src' directory.
set(PROJECT_COMPONENTS src)
idf_build_set_property(MINIMAL_BUILD ON)

project(ascent-firmware)
