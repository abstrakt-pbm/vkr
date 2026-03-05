cmake_minimum_required(VERSION 3.16)
project(hal CXX)


set(HAL_SOURCES
    hal_encoder.cpp
	hal_imu.cpp
	hal_motor.cpp)

set(HAL_HEADERS
	hal_encoder.hpp
	hal_imu.hpp
	hal_motor.hpp)

add_library(${PROJECT_NAME} STATIC ${HAL_SOURCES} ${HAL_HEADERS})

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..)

