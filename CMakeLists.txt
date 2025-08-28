
cmake_minimum_required(VERSION 3.23)
project(embedded_repo C)

set(MCU_FAMILY "stm32f4" CACHE STRING "stm32f1|stm32f4|stm32f7")
set(BOARD "stm32f407-custom" CACHE STRING "Board selection")
set(OS_PORT "baremetal" CACHE STRING "baremetal|freertos")

add_subdirectory(core)
add_subdirectory(services)

add_library(platform STATIC)
target_include_directories(platform PUBLIC
    platform/mcu/${MCU_FAMILY}/include
    platform/boards/${BOARD}
    platform/os/${OS_PORT}/include
)

file(GLOB PLATFORM_SOURCES
    platform/mcu/${MCU_FAMILY}/src/*.c
    platform/boards/${BOARD}/*.c
    platform/os/${OS_PORT}/src/*.c
)
target_sources(platform PRIVATE ${PLATFORM_SOURCES})

add_executable(uart_echo apps/uart_echo/src/main.c)
target_include_directories(uart_echo PRIVATE apps/uart_echo/cfg platform/boards/${BOARD})
target_link_libraries(uart_echo PRIVATE core services platform)
