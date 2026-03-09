cmake_minimum_required(VERSION 3.16)
project(stm32_platform C ASM)

set(STM32_PLATFORM_SOURCES
	hal/stm32f4xx_hal.c
    hal/stm32f4xx_hal_cortex.c
    hal/stm32f4xx_hal_dma.c
    hal/stm32f4xx_hal_dma_ex.c
    hal/stm32f4xx_hal_flash.c
    hal/stm32f4xx_hal_flash_ex.c
    hal/stm32f4xx_hal_flash_ramfunc.c
    hal/stm32f4xx_hal_gpio.c
    hal/stm32f4xx_hal_i2c.c
    hal/stm32f4xx_hal_i2c_ex.c
    hal/stm32f4xx_hal_pwr.c
    hal/stm32f4xx_hal_pwr_ex.c
    hal/stm32f4xx_hal_rcc.c
    hal/stm32f4xx_hal_rcc_ex.c
    hal/stm32f4xx_hal_tim.c
    hal/stm32f4xx_hal_tim_ex.c
    hal/stm32f4xx_hal_uart.c
    platform/startup_stm32f411xe.s
    platform/system_stm32f4xx.c)


add_library(${PROJECT_NAME} STATIC ${STM32_PLATFORM_SOURCES})

target_include_directories(stm32_platform PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/cmsis/include
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/include
    ${CMAKE_CURRENT_SOURCE_DIR}/hal/include)

target_compile_definitions(stm32_platform PUBLIC
    USE_HAL_DRIVER
    STM32F411xE)

