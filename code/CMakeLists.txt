cmake_minimum_required(VERSION 3.16)
project(f411_4wd C CXX ASM)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F411CEUx.ld)

# Поддиректории
add_subdirectory(math)
#add_subdirectory(robot)
add_subdirectory(stm32)

# Основная прошивка
add_executable(firmware.elf
    main.cpp
)

target_include_directories(firmware.elf PRIVATE
    math/inc
	#robot/inc
    stm32/
    stm32/
)

# Компилятор (из toolchain)
target_link_libraries(firmware.elf PRIVATE 
    math 
	#robot 
    stm32
)

target_link_options(firmware.elf PRIVATE -T ${LINKER_SCRIPT})

