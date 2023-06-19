cmake_minimum_required(VERSION 3.14)

set(PROJECT_NAME "RealTimeRelayController")
set(BUILD_NUMBER "0")

set(APP_NAME "RTRC")
set(APP_VERSION_NUMBER "1.0.0")
set(APP_SRC_DIRECTORY "${CMAKE_SOURCE_DIR}/App")

set(COMMON_CODE_DIRECTORY "${CMAKE_SOURCE_DIR}/Src")

set(FREERTOS_CFG_DIRECTORY "${CMAKE_SOURCE_DIR}/Config")
set(FREERTOS_SRC_DIRECTORY "${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel")

include(pico_sdk_import.cmake)

project(${PROJECT_NAME}
        LANGUAGES C CXX ASM
        DESCRIPTION "FreeRTOS-based RP2040 applications"
)

pico_sdk_init()

add_library(FreeRTOS STATIC
    ${FREERTOS_SRC_DIRECTORY}/event_groups.c
    ${FREERTOS_SRC_DIRECTORY}/list.c
    ${FREERTOS_SRC_DIRECTORY}/queue.c
    ${FREERTOS_SRC_DIRECTORY}/stream_buffer.c
    ${FREERTOS_SRC_DIRECTORY}/tasks.c
    ${FREERTOS_SRC_DIRECTORY}/timers.c
    ${FREERTOS_SRC_DIRECTORY}/portable/MemMang/heap_3.c
    ${FREERTOS_SRC_DIRECTORY}/portable/GCC/ARM_CM0/port.c
)

target_include_directories(FreeRTOS PUBLIC
    ${FREERTOS_CFG_DIRECTORY}/
    ${FREERTOS_SRC_DIRECTORY}/include
    ${FREERTOS_SRC_DIRECTORY}/portable/GCC/ARM_CM0
)

add_executable(${APP_NAME}
    ${APP_SRC_DIRECTORY}/main.cpp
    ${APP_SRC_DIRECTORY}/static_memory_rtos.c
)

target_include_directories(${APP_NAME} PRIVATE
    ${APP_SRC_DIRECTORY}/Include
)

target_link_libraries(${APP_NAME} PUBLIC
    pico_stdlib
    pico_i2c_slave
    hardware_i2c
    FreeRTOS)

pico_enable_stdio_usb(${APP_NAME} 1)
pico_add_extra_outputs(${APP_NAME})

