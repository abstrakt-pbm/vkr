add_library(robot_hil
    src/session.cpp
)

target_include_directories(robot_hil PUBLIC
    include
)

target_compile_features(robot_hil PUBLIC
    cxx_std_17
)
