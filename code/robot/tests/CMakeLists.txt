add_executable(test_robot 
	mocks/hal/mock_motor.cpp
    motor_tests.hpp
    motor_tests.cpp
)

target_link_libraries(test_robot PRIVATE 
    robot 
	hal
	gtest_main
	gmock_main
    gtest 
    gmock
)

target_compile_features(test_robot PRIVATE cxx_std_17)


