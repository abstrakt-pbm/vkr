
set(WEBOTS_SEARCH_PATHS
    $ENV{WEBOTS_HOME}          # Высший приоритет: если пользователь явно задал env var
    "/usr/local/webots"        # Дефолт для официального .deb пакета
    "/opt/webots"              # Дефолт для ручной распаковки архива
    "/usr/share/webots"        # Дефолт для некоторых менеджеров пакетов (AUR и т.д.)
)

find_path(WEBOTS_HOME_DIR 
    NAMES include/controller/c/webots/robot.h
    PATHS ${WEBOTS_SEARCH_PATHS}
    NO_DEFAULT_PATH)

if(NOT WEBOTS_HOME_DIR)
    message(FATAL_ERROR "Webots installation not found! Please set WEBOTS_HOME environment variable or install Webots to /usr/local/webots.")
else()
    message(STATUS "Found Webots at: ${WEBOTS_HOME_DIR}")
endif()

set(TARGET_NAME webots_sil)

add_executable(${TARGET_NAME} 
    main.cpp 
	webots_encoder.cpp
    webots_hal.cpp
    webots_imu.cpp
    webots_motor.cpp)

# 5. Подключаем заголовочные файлы по найденному пути
target_include_directories(${TARGET_NAME} PRIVATE
    ${WEBOTS_HOME_DIR}/include/controller/cpp
    ${WEBOTS_HOME_DIR}/include/controller/c
	${CMAKE_CURRENT_SOURCE_DIR}/..
)

# 6. Подключаем директорию с библиотеками (.so)
target_link_directories(${TARGET_NAME} PRIVATE
    ${WEBOTS_HOME_DIR}/lib/controller
)

# 7. Линкуем Webots API и твою бизнес-логику
target_link_libraries(${TARGET_NAME} PRIVATE
    CppController
    Controller
    robot      # Твоя библиотека
	robotmath
)

# 8. Заставляем CMake положить бинарник прямо в папку контроллера (критично для Webots)
set_target_properties(${TARGET_NAME} PROPERTIES 
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

