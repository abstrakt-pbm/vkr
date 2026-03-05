project(robotmath CXX)

set (MATH_INC
	"math.hpp")

set (MATH_SRC
	"math.cpp")

add_library(${PROJECT_NAME} STATIC
	${MATH_INC}
	${MATH_SRC})


target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..)
