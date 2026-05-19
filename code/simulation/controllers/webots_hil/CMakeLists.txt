set(WEBOTS_SEARCH_PATHS
    $ENV{WEBOTS_HOME}
    "/usr/local/webots"
    "/opt/webots"
    "/usr/share/webots"
)

find_path(WEBOTS_HOME_DIR
    NAMES include/controller/c/webots/robot.h
    PATHS ${WEBOTS_SEARCH_PATHS}
    NO_DEFAULT_PATH
)

if(NOT WEBOTS_HOME_DIR)
    message(FATAL_ERROR "Webots installation not found! Please set WEBOTS_HOME environment variable or install Webots to /usr/local/webots.")
else()
    message(STATUS "Found Webots at: ${WEBOTS_HOME_DIR}")
endif()

set(TARGET_NAME webots_hil)

add_executable(${TARGET_NAME}
    main.cpp
)

target_include_directories(${TARGET_NAME} PRIVATE
    ${WEBOTS_HOME_DIR}/include/controller/cpp
    ${WEBOTS_HOME_DIR}/include/controller/c
)

target_link_directories(${TARGET_NAME} PRIVATE
    ${WEBOTS_HOME_DIR}/lib/controller
)

target_link_libraries(${TARGET_NAME} PRIVATE
    CppController
    Controller
    robot_hil
)

target_compile_features(${TARGET_NAME} PRIVATE cxx_std_17)

set_target_properties(${TARGET_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
