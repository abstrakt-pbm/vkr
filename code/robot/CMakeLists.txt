project(robot CXX)

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

set (ROBOT_INC
	"robot.hpp"
	"robot_control.hpp"
	"encoder.hpp"
	"imu.hpp"
	"motor.hpp")

set (ROBOT_SRC
	"robot.cpp"
	"robot_control.cpp"
	"encoder.cpp"
	"imu.cpp"
	"motor.cpp")

add_library(${PROJECT_NAME} STATIC
	${ROBOT_INC}
	${ROBOT_SRC})

target_link_libraries(robot PRIVATE 
	hal
)


target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..)

