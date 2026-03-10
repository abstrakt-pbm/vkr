project(robot CXX)


add_subdirectory(math)
add_subdirectory(hal)

set (ROBOT_INC
	"robot.hpp"
	"robot_control.hpp"
	"robot_ekf.hpp"
	"encoder.hpp"
	"imu.hpp"
	"motor.hpp"
	"odometry.hpp")

set (ROBOT_SRC
	"robot.cpp"
	"robot_control.cpp"
	"robot_ekf.cpp"
	"encoder.cpp"
	"imu.cpp"
	"motor.cpp"
	"odometry.cpp")

add_library(${PROJECT_NAME} STATIC
	${ROBOT_INC}
	${ROBOT_SRC})

target_link_libraries(robot PRIVATE 
	robotmath
	hal)


target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..
	${CMAKE_CURRENT_SOURCE_DIR}/hal/
	${CMAKE_CURRENT_SOURCE_DIR}/math/)

