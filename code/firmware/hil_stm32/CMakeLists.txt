add_library(robot_hil_stm32
    src/uart_transport.cpp
)

target_include_directories(robot_hil_stm32 PUBLIC
    include
)

target_link_libraries(robot_hil_stm32 PUBLIC
    robot_hil
    stm32_platform
)

target_compile_features(robot_hil_stm32 PUBLIC
    cxx_std_17
)
