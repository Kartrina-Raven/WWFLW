cmake_minimum_required(VERSION 2.8.3)
project(ego_planner)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  airsim_ros
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env
  path_searching
  traj_opt
  traj_utils
  message_generation
  message_runtime
  cv_bridge
  px4ctrl
  transform_utils
  tf2_ros
)

# Sophus local install
set(SOPHUS_ROOT "/home/kartrina/svslocal" CACHE PATH "Root directory of Sophus installation")
set(SOPHUS_INCLUDE_DIR "${SOPHUS_ROOT}/include")
set(SOPHUS_LIBRARY_DIR "${SOPHUS_ROOT}/lib")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ego_planner
  CATKIN_DEPENDS
    airsim_ros
    roscpp
    std_msgs
    geometry_msgs
    quadrotor_msgs
    plan_env
    path_searching
    traj_opt
    traj_utils
    px4ctrl
    transform_utils
    tf2_ros
    cv_bridge
    airsim_ros
    message_generation
)

include_directories(
  include
  ../px4ctrl/include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${SOPHUS_INCLUDE_DIR}
)

link_directories(${SOPHUS_LIBRARY_DIR})

add_executable(ego_planner_node
  src/ego_planner_node.cpp
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
)

add_dependencies(ego_planner_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ego_planner_node
  ${catkin_LIBRARIES}
  Sophus
)

add_executable(traj_server
  src/traj_server.cpp
)

add_dependencies(traj_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(traj_server
  ${catkin_LIBRARIES}
  Sophus
)

set_target_properties(ego_planner_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${SOPHUS_LIBRARY_DIR}"
)

set_target_properties(traj_server PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${SOPHUS_LIBRARY_DIR}"
)