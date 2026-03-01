cmake_minimum_required(VERSION 3.16)
project(f411_4wd C CXX ASM)

enable_language(ASM)

add_executable(firmware.elf
    src/main.cpp
	#    src/stm32/startup_stm32f4xx.s
	#src/stm32/system_stm32f4xx.c
)

target_include_directories(firmware.elf PRIVATE 
    inc inc/math inc/robot inc/stm32 inc/stm32/CMSIS inc/stm32/CMSIS/Core
)

target_compile_options(firmware.elf PRIVATE
    -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
    -DSTM32F411xE -Os -Wall 
    $<$<COMPILE_LANGUAGE:CXX>:-std=c++17 -fno-rtti -fno-exceptions>
    $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp>
)

target_link_options(firmware.elf PRIVATE
    -mcpu=cortex-m4 -mthumb 
    -T${CMAKE_SOURCE_DIR}/STM32F411CEUx.ld
    -Wl,--gc-sections
)

# HEX/BIN + SIZE
add_custom_command(TARGET firmware.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:firmware.elf> firmware.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:firmware.elf> firmware.bin
    COMMAND ${CMAKE_SIZEUTIL} $<TARGET_FILE:firmware.elf>
)

