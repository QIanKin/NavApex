cmake_minimum_required(VERSION 3.16)
project(navapex_ros2 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

get_filename_component(NAVAPEX_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
add_subdirectory("${NAVAPEX_ROOT}/src" "${CMAKE_CURRENT_BINARY_DIR}/navapex")
add_executable(navapex_node src/navapex_node.cpp)
target_link_libraries(navapex_node planner::core planner::perception planner::log planner::replay)
ament_target_dependencies(navapex_node rclcpp nav_msgs sensor_msgs geometry_msgs std_msgs std_srvs
  tf2 tf2_ros tf2_geometry_msgs)
target_compile_options(navapex_node PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS navapex_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config worlds rviz DESTINATION share/${PROJECT_NAME})

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/maps")
find_package(turtlebot3_gazebo REQUIRED)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  scripts/generate_map.py worlds/navapex.world config/global_map.json)
execute_process(COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_map.py"
  --world "${CMAKE_CURRENT_SOURCE_DIR}/worlds/navapex.world"
  --output "${CMAKE_CURRENT_BINARY_DIR}/maps/navapex.json"
  --robot "${turtlebot3_gazebo_DIR}/../models/turtlebot3_burger/model.sdf"
  --robot-output "${CMAKE_CURRENT_BINARY_DIR}/burger.sdf" COMMAND_ERROR_IS_FATAL ANY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/maps/navapex.json" DESTINATION share/${PROJECT_NAME}/maps)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/burger.sdf" DESTINATION share/${PROJECT_NAME}/models)
ament_package()
