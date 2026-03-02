set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm-eabi)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(CMAKE_SIZEUTIL arm-none-eabi-size)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16")
set(CMAKE_C_FLAGS "${CPU_FLAGS} -DSTM32F411xE -Os -Wall" CACHE STRING "")
set(CMAKE_CXX_FLAGS "${CPU_FLAGS} -DSTM32F411xE -Os -Wall -std=c++17 -fno-rtti -fno-exceptions" CACHE STRING "")
set(CMAKE_ASM_FLAGS "${CPU_FLAGS} -DSTM32F411xE" CACHE STRING "")
set(CMAKE_EXE_LINKER_FLAGS "-specs=nano.specs -lc -lm -lnosys -Wl,--gc-sections" CACHE STRING "")

set(CMAKE_C_COMPILER_WORKS 1 CACHE BOOL "")
set(CMAKE_CXX_COMPILER_WORKS 1 CACHE BOOL "")

