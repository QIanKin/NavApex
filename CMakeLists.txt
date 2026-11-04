cmake_minimum_required(VERSION 3.16)
project(NavApex VERSION 0.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(nlohmann_json 3.2 REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc)

add_library(planner_utils INTERFACE)
add_library(planner::utils ALIAS planner_utils)
target_compile_features(planner_utils INTERFACE cxx_std_17)
set_target_properties(planner_utils PROPERTIES EXPORT_NAME utils)
target_include_directories(planner_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/planner/traj_opt>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(planner_utils INTERFACE Eigen3::Eigen)

add_library(planner_map INTERFACE)
add_library(planner::map ALIAS planner_map)
set_target_properties(planner_map PROPERTIES EXPORT_NAME map)
target_include_directories(planner_map INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<INSTALL_INTERFACE:include>)
target_link_libraries(planner_map INTERFACE Eigen3::Eigen opencv_core opencv_imgproc)

add_library(planner_robot_model STATIC robot_model/robot_model.cpp)
add_library(planner::robot_model ALIAS planner_robot_model)
set_target_properties(planner_robot_model PROPERTIES EXPORT_NAME robot_model)
target_include_directories(planner_robot_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<INSTALL_INTERFACE:include>)
target_link_libraries(planner_robot_model PUBLIC planner::map)
target_compile_options(planner_robot_model PRIVATE -Wall -Wextra -Wpedantic)

set(G3_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tools/g3log/src)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/generated/g3log/generated_definitions.hpp
  "#pragma once\n#define DISABLE_FATAL_SIGNALHANDLING\n#define G3LOG_DEBUG DEBUG\n")
add_library(planner_log STATIC
  tools/log.cpp
  ${G3_DIR}/g3log.cpp ${G3_DIR}/logcapture.cpp ${G3_DIR}/loglevels.cpp
  ${G3_DIR}/logmessage.cpp ${G3_DIR}/logworker.cpp ${G3_DIR}/time.cpp
  ${G3_DIR}/filesink.cpp ${G3_DIR}/crashhandler_unix.cpp)
add_library(planner::log ALIAS planner_log)
set_target_properties(planner_log PROPERTIES EXPORT_NAME log)
target_compile_features(planner_log PUBLIC cxx_std_17)
target_include_directories(planner_log PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<INSTALL_INTERFACE:include>
  PRIVATE ${G3_DIR} ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_link_libraries(planner_log PRIVATE planner::utils PUBLIC Threads::Threads)
set_source_files_properties(tools/log.cpp PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Wpedantic")

add_library(planner_params INTERFACE)
add_library(planner::params ALIAS planner_params)
set_target_properties(planner_params PROPERTIES EXPORT_NAME params)
target_link_libraries(planner_params INTERFACE planner::utils nlohmann_json::nlohmann_json)

add_library(planner_omni3d STATIC
  planner/traj_opt/optimizer/omni_3d_opt/src/omni_3d_optimizer.cpp
  planner/traj_opt/optimizer/omni_3d_opt/src/local_astar.cpp)
add_library(planner::omni3d ALIAS planner_omni3d)
set_target_properties(planner_omni3d PROPERTIES EXPORT_NAME omni3d)
target_include_directories(planner_omni3d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/planner/traj_opt/optimizer/omni_3d_opt/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(planner_omni3d PUBLIC planner::params planner::map
  PRIVATE planner::log)
target_compile_options(planner_omni3d PRIVATE -Wall -Wextra -Wpedantic)

add_library(planner_diff STATIC
  planner/traj_opt/optimizer/diff_opt/src/diff_optimizer.cpp)
add_library(planner::diff ALIAS planner_diff)
set_target_properties(planner_diff PROPERTIES EXPORT_NAME diff)
target_include_directories(planner_diff PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/planner/traj_opt/optimizer/diff_opt/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(planner_diff PUBLIC planner::params planner::map planner::robot_model
  PRIVATE planner::log)
target_compile_options(planner_diff PRIVATE -Wall -Wextra -Wpedantic)

# 对外接口封装；只有实现文件依赖具体规划器。
add_library(planner_optimizer STATIC planner/traj_opt/optimizer_interface.cpp)
add_library(planner::optimizer ALIAS planner_optimizer)
set_target_properties(planner_optimizer PROPERTIES EXPORT_NAME optimizer)
target_link_libraries(planner_optimizer PUBLIC planner::params planner::map
  PRIVATE planner::omni3d planner::diff planner::log)
target_compile_options(planner_optimizer PRIVATE -Wall -Wextra -Wpedantic)

include(CTest)
if(BUILD_TESTING)
  add_executable(occupancy_optimizer_test tests/occupancy_optimizer_test.cpp)
  target_link_libraries(occupancy_optimizer_test PRIVATE planner::optimizer planner::omni3d planner::log)
  target_compile_options(occupancy_optimizer_test PRIVATE -UNDEBUG)
  add_test(NAME occupancy_optimizer COMMAND occupancy_optimizer_test)
  add_executable(occupancy_gradient_test tests/occupancy_gradient_test.cpp)
  target_link_libraries(occupancy_gradient_test PRIVATE planner::optimizer planner::omni3d planner::log)
  add_test(NAME occupancy_gradient COMMAND occupancy_gradient_test)
  add_executable(obstacle_constraints_test tests/obstacle_constraints_test.cpp)
  target_link_libraries(obstacle_constraints_test PRIVATE planner::omni3d planner::log)
  target_compile_options(obstacle_constraints_test PRIVATE -UNDEBUG)
  add_test(NAME obstacle_constraints COMMAND obstacle_constraints_test)
  add_executable(params_test tests/params_test.cpp)
  target_link_libraries(params_test PRIVATE planner::optimizer planner::params)
  target_compile_options(params_test PRIVATE -UNDEBUG)
  add_test(NAME params COMMAND params_test "planner/traj_opt/config")
  set_tests_properties(occupancy_optimizer occupancy_gradient params
    PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
  add_executable(diff_optimizer_test
    planner/traj_opt/optimizer/diff_opt/tests/diff_optimizer_test.cpp)
  target_link_libraries(diff_optimizer_test PRIVATE planner::optimizer planner::diff planner::log)
  target_compile_options(diff_optimizer_test PRIVATE -UNDEBUG)
  add_test(NAME diff_optimizer COMMAND diff_optimizer_test
    "${CMAKE_CURRENT_SOURCE_DIR}/planner/traj_opt/config/diff.json")
  add_executable(diff_gradient_test
    planner/traj_opt/optimizer/diff_opt/tests/diff_gradient_test.cpp)
  target_link_libraries(diff_gradient_test PRIVATE planner::diff planner::log)
  add_test(NAME diff_gradient COMMAND diff_gradient_test)
endif()

include(GNUInstallDirs)
install(TARGETS planner_utils planner_map planner_log planner_params planner_omni3d planner_diff planner_optimizer planner_robot_model EXPORT PlannerTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY planner/traj_opt/optimizer/omni_3d_opt/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY planner/traj_opt/optimizer/diff_opt/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES planner/traj_opt/optimizer/base_optimizer.hpp
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/optimizer)
install(FILES planner/traj_opt/plan_input.hpp planner/traj_opt/optimizer_interface.hpp
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY planner/traj_opt/utils/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/utils
  FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY planner/traj_opt/params/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/params
  FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY map/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/map
  FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY robot_model/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot_model
  FILES_MATCHING PATTERN "*.hpp")
install(FILES tools/log.hpp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/tools)
install(EXPORT PlannerTargets NAMESPACE planner:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/planner)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/plannerConfig.cmake
  "include(CMakeFindDependencyMacro)\nfind_dependency(Eigen3 3.3 NO_MODULE)\nfind_dependency(nlohmann_json 3.2)\nfind_dependency(Threads)\nfind_dependency(OpenCV 4 COMPONENTS core imgproc)\ninclude(\"\${CMAKE_CURRENT_LIST_DIR}/PlannerTargets.cmake\")\n")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/plannerConfig.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/planner)
install(FILES LICENSE DESTINATION ${CMAKE_INSTALL_DATADIR}/planner/licenses)
install(FILES tools/g3log/LICENSE DESTINATION ${CMAKE_INSTALL_DATADIR}/planner/licenses RENAME g3log-LICENSE)
install(DIRECTORY planner/traj_opt/config/ DESTINATION ${CMAKE_INSTALL_DATADIR}/planner/config
  FILES_MATCHING PATTERN "*.json")
