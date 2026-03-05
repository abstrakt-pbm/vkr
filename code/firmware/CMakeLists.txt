cmake_minimum_required(VERSION 3.16)
project(f411_4wd C CXX ASM)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F411CEUx.ld)

add_subdirectory(stm32)
add_subdirectory(hal)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../math math_arm_build)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../robot robot_arm_build)

# Основная прошивка
add_executable(firmware.elf
    main.cpp
)

target_include_directories(firmware.elf PRIVATE
    stm32/
)

# Линкуем ARM-версии библиотек
target_link_libraries(firmware.elf PRIVATE 
    math 
    robot 
    stm32
	hal
)

target_link_options(firmware.elf PRIVATE -T ${LINKER_SCRIPT})

