cmake_minimum_required(VERSION 3.12)

set(PICO_SDK_PATH
    "${CMAKE_CURRENT_LIST_DIR}/external/pico-sdk"
    CACHE PATH "Path to the Raspberry Pi Pico SDK"
)

include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")

project(soil_sampler_pico C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

set(MICRO_ROS_PICO_SDK_DIR
    ${CMAKE_CURRENT_LIST_DIR}/build/micro_ros
)

link_directories(
    ${MICRO_ROS_PICO_SDK_DIR}/libmicroros
)

add_executable(soil_sampler_pico
    src/main.c
    src/load_cell.c
    src/actuator.c
    ${MICRO_ROS_PICO_SDK_DIR}/pico_uart_transport.c
)

target_include_directories(soil_sampler_pico PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${MICRO_ROS_PICO_SDK_DIR}
    ${MICRO_ROS_PICO_SDK_DIR}/libmicroros/include
)

target_link_libraries(soil_sampler_pico
    pico_stdlib
    ${MICRO_ROS_PICO_SDK_DIR}/libmicroros/libmicroros.a
    hardware_i2c
)

target_compile_options(soil_sampler_pico PRIVATE
    -ffunction-sections
    -fdata-sections
)

pico_enable_stdio_usb(soil_sampler_pico 1)
pico_enable_stdio_uart(soil_sampler_pico 0)

add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

pico_add_extra_outputs(soil_sampler_pico)