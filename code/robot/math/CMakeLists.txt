project(robotmath CXX)


set (MATH_SRC
	"pid.cpp"
	"ffmodel.cpp")

add_library(${PROJECT_NAME} STATIC	
	${MATH_SRC})

target_link_libraries(robotmath PRIVATE 
	robot)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
