cmake_minimum_required(VERSION 3.11)
project(micro_ros_agent_ble)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

find_package(ament_cmake REQUIRED)
find_package(micro_ros_agent REQUIRED)
find_package(microxrcedds_agent REQUIRED)

include(FetchContent)
FetchContent_Declare(simpleble
    GIT_REPOSITORY https://github.com/OpenBluetoothToolbox/SimpleBLE.git
    GIT_TAG v0.10.4
    SOURCE_SUBDIR simpleble
)
FetchContent_MakeAvailable(simpleble)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/ble_transport.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    include
)

ament_target_dependencies(${PROJECT_NAME}
    micro_ros_agent
)

target_link_libraries(${PROJECT_NAME}
    micro_ros_agent::micro_ros_agent_graph_manager  # Graph manager library from micro_ros_agent
    microxrcedds_agent
    simpleble::simpleble
    bluetooth
    pthread
    $<$<PLATFORM_ID:Linux>:rt>
    $<$<PLATFORM_ID:Linux>:dl>
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()