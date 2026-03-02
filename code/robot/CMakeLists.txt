project(robot CXX)

set (ROBOT_INC
	"robot.hpp"
	"robot_control.hpp")

set (ROBOT_SRC
	"robot.cpp"
	"robot_control.cpp")

add_library(${PROJECT_NAME} STATIC
	${ROBOT_INC}
	${ROBOT_SRC})

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..)

