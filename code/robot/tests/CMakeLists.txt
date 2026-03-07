add_executable(test_robot 
	mocks/hal/mock_motor.cpp
	mocks/hal/mock_encoder.cpp
	mocks/hal/mock_imu.cpp
    motor_tests.cpp
    encoder_tests.cpp
    imu_tests.cpp
    robot_tests.cpp
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


