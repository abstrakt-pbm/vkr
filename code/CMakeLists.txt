cmake_minimum_required(VERSION 3.16)
project(vkr_robot CXX C ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Подключаем модули
add_subdirectory(robot)
add_subdirectory(tests)
add_subdirectory(firmware)

