cmake_minimum_required(VERSION 3.16)
project(vkr_robot CXX C ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# GTest для тестов Robot
option(BUILD_TESTS "Build tests" ON)
if(BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
	set(gtest_disable_pthreads ON CACHE BOOL "" FORCE)
	set(BUILD_GMOCK ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
    enable_testing()
endif()

# Подключаем модули
add_subdirectory(robot)
add_subdirectory(math)
add_subdirectory(hal)
#add_subdirectory(firmware)

# ARM Firmware (ExternalProject)
include(ExternalProject)
ExternalProject_Add(firmware_arm
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/firmware
    CMAKE_ARGS 
        -DCMAKE_TOOLCHAIN_FILE=${CMAKE_CURRENT_SOURCE_DIR}/toolchain-arm.cmake
        -DCMAKE_BUILD_TYPE=Release
        -DBUILD_TESTS=OFF
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/firmware_build
    INSTALL_COMMAND ""
    BUILD_BYPRODUCTS firmware_build/firmware.elf
)

# Удобные таргеты
add_custom_target(firmware ALL DEPENDS firmware_arm)
add_custom_target(run_tests ALL
    COMMAND ctest --output-on-failure -j8
    DEPENDS test_robot
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Running robot tests"
)

