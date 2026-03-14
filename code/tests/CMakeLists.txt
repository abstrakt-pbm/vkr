
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

add_executable(test_robot 
	mocks/hal/mock_motor.cpp
	mocks/hal/mock_encoder.cpp
	mocks/hal/mock_imu.cpp
	ekf_tests.cpp
    motor_tests.cpp
    odometry_tests.cpp
    pid_tests.cpp
    encoder_tests.cpp
    ffmodel_tests.cpp
    imu_tests.cpp
    robot_tests.cpp
    robot_control_tests.cpp)

target_link_libraries(test_robot PRIVATE 
    robot 
	robotmath
	hal
	gtest_main
    gtest)

target_compile_features(test_robot PRIVATE cxx_std_17)


