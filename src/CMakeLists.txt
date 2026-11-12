cmake_minimum_required(VERSION 3.16)
project(NavApex VERSION 0.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(nlohmann_json 3.2 REQUIRED)
find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)
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
target_link_libraries(planner_map INTERFACE Eigen3::Eigen planner::log)

add_library(planner_esdf STATIC perception/esdf/esdf_map_2d.cpp)
add_library(planner::esdf ALIAS planner_esdf)
set_target_properties(planner_esdf PROPERTIES EXPORT_NAME esdf)
target_link_libraries(planner_esdf PUBLIC planner::map opencv_core PRIVATE opencv_imgproc)
target_compile_options(planner_esdf PRIVATE -Wall -Wextra -Wpedantic)

add_library(planner_perception STATIC
  perception/perception_interface.cpp
  perception/lidar/lidar2d/lidar_2d_processor.cpp)
add_library(planner::perception ALIAS planner_perception)
set_target_properties(planner_perception PROPERTIES EXPORT_NAME perception)
target_link_libraries(planner_perception PUBLIC planner::map planner::robot_model nlohmann_json::nlohmann_json
  PRIVATE opencv_core opencv_imgproc)
target_compile_options(planner_perception PRIVATE -Wall -Wextra -Wpedantic)

add_library(planner_robot_model STATIC robot_model/robot_model.cpp)
add_library(planner::robot_model ALIAS planner_robot_model)
set_target_properties(planner_robot_model PROPERTIES EXPORT_NAME robot_model)
target_include_directories(planner_robot_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<INSTALL_INTERFACE:include>)
target_link_libraries(planner_robot_model PUBLIC planner::map PRIVATE planner::esdf nlohmann_json::nlohmann_json)
target_compile_options(planner_robot_model PRIVATE -Wall -Wextra -Wpedantic)

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/g3log/src/g3log.cpp")
  message(FATAL_ERROR
    "g3log submodule is missing. Run 'git submodule update --init --recursive' from the repository root.")
endif()
set(G3_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tools/g3log/src)
file(GENERATE OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/generated/g3log/generated_definitions.hpp CONTENT
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

add_library(planner_replay STATIC tools/replay.cpp)
add_library(planner::replay ALIAS planner_replay)
set_target_properties(planner_replay PROPERTIES EXPORT_NAME replay)
target_link_libraries(planner_replay PUBLIC planner::map planner::utils nlohmann_json::nlohmann_json
  PRIVATE ZLIB::ZLIB)
target_compile_options(planner_replay PRIVATE -Wall -Wextra -Wpedantic)

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
  PRIVATE planner::log planner::esdf)
target_compile_options(planner_diff PRIVATE -Wall -Wextra -Wpedantic)

# 对外接口封装；只有实现文件依赖具体规划器。
add_library(planner_optimizer STATIC planner/traj_opt/optimizer_interface.cpp)
add_library(planner::optimizer ALIAS planner_optimizer)
set_target_properties(planner_optimizer PROPERTIES EXPORT_NAME optimizer)
target_link_libraries(planner_optimizer PUBLIC planner::params planner::map
  PRIVATE planner::omni3d planner::diff planner::log)
target_compile_options(planner_optimizer PRIVATE -Wall -Wextra -Wpedantic)

add_library(planner_search_params INTERFACE)
add_library(planner::search_params ALIAS planner_search_params)
set_target_properties(planner_search_params PROPERTIES EXPORT_NAME search_params)
target_include_directories(planner_search_params INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<INSTALL_INTERFACE:include>)
target_link_libraries(planner_search_params INTERFACE Eigen3::Eigen nlohmann_json::nlohmann_json)

add_library(planner_astar STATIC planner/path_plan/searcher/astar/src/astar.cpp)
add_library(planner::astar ALIAS planner_astar)
set_target_properties(planner_astar PROPERTIES EXPORT_NAME astar)
target_include_directories(planner_astar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/planner/path_plan/searcher/astar/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(planner_astar PUBLIC planner::search_params planner::map PRIVATE planner::log)
target_compile_options(planner_astar PRIVATE -Wall -Wextra -Wpedantic)

# 对外接口封装；只有实现文件依赖具体搜索器。
add_library(planner_searcher STATIC planner/path_plan/searcher_interface.cpp)
add_library(planner::searcher ALIAS planner_searcher)
set_target_properties(planner_searcher PROPERTIES EXPORT_NAME searcher)
target_link_libraries(planner_searcher PUBLIC planner::search_params planner::map PRIVATE planner::astar)
target_compile_options(planner_searcher PRIVATE -Wall -Wextra -Wpedantic)

add_library(planner_core STATIC
  planner/plan_manage/diff_planner.cpp
  planner/plan_manage/params/planner_params.cpp)
add_library(planner::core ALIAS planner_core)
set_target_properties(planner_core PROPERTIES EXPORT_NAME core)
target_link_libraries(planner_core PUBLIC planner::optimizer planner::searcher planner::esdf planner::robot_model
  PRIVATE planner::log)
target_compile_options(planner_core PRIVATE -Wall -Wextra -Wpedantic)

add_library(planner_controller STATIC planner/controller/trajectory_tracker.cpp)
add_library(planner::controller ALIAS planner_controller)
set_target_properties(planner_controller PROPERTIES EXPORT_NAME controller)
target_link_libraries(planner_controller PUBLIC planner::params planner::map)
target_compile_options(planner_controller PRIVATE -Wall -Wextra -Wpedantic)

include(GNUInstallDirs)
install(TARGETS planner_utils planner_map planner_log planner_replay planner_params planner_omni3d planner_diff planner_optimizer planner_robot_model
  planner_search_params planner_astar planner_searcher planner_esdf planner_core planner_perception planner_controller EXPORT PlannerTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY planner/traj_opt/optimizer/omni_3d_opt/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY planner/traj_opt/optimizer/diff_opt/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES planner/traj_opt/optimizer/base_optimizer.hpp
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/optimizer)
install(FILES planner/traj_opt/optimizer_io.hpp planner/traj_opt/optimizer_interface.hpp
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY planner/path_plan/searcher/astar/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES planner/path_plan/searcher_io.hpp planner/path_plan/searcher_interface.hpp
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/planner/path_plan)
install(FILES planner/path_plan/searcher/base_searcher.hpp
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/planner/path_plan/searcher)
install(DIRECTORY planner/path_plan/params/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/planner/path_plan/params
  FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY planner/traj_opt/utils/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/utils
  FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY planner/traj_opt/params/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/params
  FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY perception/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/perception
  FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY robot_model/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot_model
  FILES_MATCHING PATTERN "*.hpp")
install(FILES tools/log.hpp tools/replay.hpp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/tools)
install(FILES planner/plan_manage/base_planner.hpp planner/plan_manage/diff_planner.hpp
  planner/plan_manage/planner_io.hpp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/planner/plan_manage)
install(DIRECTORY planner/controller/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/planner/controller
  FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY planner/plan_manage/params/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/planner/plan_manage/params
  FILES_MATCHING PATTERN "*.hpp")
install(EXPORT PlannerTargets NAMESPACE planner:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/planner)
file(GENERATE OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/plannerConfig.cmake CONTENT
  "include(CMakeFindDependencyMacro)\nfind_dependency(Eigen3 3.3 NO_MODULE)\nfind_dependency(nlohmann_json 3.2)\nfind_dependency(Threads)\nfind_dependency(ZLIB)\nfind_dependency(OpenCV 4 COMPONENTS core imgproc)\ninclude(\"\${CMAKE_CURRENT_LIST_DIR}/PlannerTargets.cmake\")\n")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/plannerConfig.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/planner)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../LICENSE" DESTINATION ${CMAKE_INSTALL_DATADIR}/planner/licenses)
install(FILES tools/g3log/LICENSE DESTINATION ${CMAKE_INSTALL_DATADIR}/planner/licenses RENAME g3log-LICENSE)
install(DIRECTORY planner/traj_opt/config/ DESTINATION ${CMAKE_INSTALL_DATADIR}/planner/config/traj_opt
  FILES_MATCHING PATTERN "*.json")
install(DIRECTORY planner/plan_manage/config/ DESTINATION ${CMAKE_INSTALL_DATADIR}/planner/config/plan_manage
  FILES_MATCHING PATTERN "*.json")
install(DIRECTORY planner/controller/config/ DESTINATION ${CMAKE_INSTALL_DATADIR}/planner/config/controller
  FILES_MATCHING PATTERN "*.json")
install(DIRECTORY planner/path_plan/config/ DESTINATION ${CMAKE_INSTALL_DATADIR}/planner/config/path_plan
  FILES_MATCHING PATTERN "*.json")
install(DIRECTORY perception/config/ DESTINATION ${CMAKE_INSTALL_DATADIR}/planner/config/perception
  FILES_MATCHING PATTERN "*.json")
install(DIRECTORY robot_model/config/ DESTINATION ${CMAKE_INSTALL_DATADIR}/planner/config/robot_model
  FILES_MATCHING PATTERN "*.json")
