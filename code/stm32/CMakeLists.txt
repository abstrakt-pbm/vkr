cmake_minimum_required(VERSION 3.16)
project(stm32 C ASM)

set(STM32_SOURCES
    startup_stm32f411xe.s
    system_stm32f4xx.c
)

add_library(${PROJECT_NAME} STATIC ${STM32_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
    Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
)

