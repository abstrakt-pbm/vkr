cmake_minimum_required(VERSION 3.16)

project(f411_4wd C CXX ASM)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F411CEUx.ld)

add_subdirectory(stm32)
add_subdirectory(hil_stm32)

add_executable(firmware.elf
    main.cpp
)

target_include_directories(firmware.elf PRIVATE
    stm32/
)

target_link_libraries(firmware.elf PRIVATE
    robot
    robotmath
    robot_hil
    robot_hil_stm32
    stm32_platform
)

target_link_options(firmware.elf PRIVATE
    -T ${LINKER_SCRIPT}
)

add_custom_command(TARGET firmware.elf POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary firmware.elf firmware.bin
    COMMAND arm-none-eabi-objcopy -O ihex firmware.elf firmware.hex
    COMMAND arm-none-eabi-size firmware.elf
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Создаём .bin/.hex"
)
