project(robotmath CXX)

set (MATH_INC
	"mathr.hpp")

set (MATH_SRC
	"mathr.cpp")

add_library(${PROJECT_NAME} STATIC
	${MATH_INC}
	${MATH_SRC})


target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..)
