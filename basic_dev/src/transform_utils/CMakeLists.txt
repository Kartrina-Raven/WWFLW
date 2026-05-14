cmake_minimum_required(VERSION 3.0.2)
project(transform_utils)

############################
# C++ standard
############################
add_compile_options(-std=c++14)

############################
# Find dependencies
############################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  yolov11_ros_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# ===== Sophus local install =====
# 在编译时传：
# -DSOPHUS_ROOT=/path/to/svslocal
set(SOPHUS_ROOT "/home/kartrina/svslocal" CACHE PATH "Root directory of Sophus installation")
set(SOPHUS_INCLUDE_DIR "${SOPHUS_ROOT}/include")
set(SOPHUS_LIBRARY_DIR "${SOPHUS_ROOT}/lib")

############################
# Catkin package
############################

add_message_files(
  FILES
  WayPoints.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES transform_utils
  CATKIN_DEPENDS
    roscpp
    std_msgs
    roslib
    sensor_msgs
    geometry_msgs
    nav_msgs
    cv_bridge
    yolov11_ros_msgs
    message_runtime
)


############################
# Include directories
############################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${SOPHUS_INCLUDE_DIR}
)

############################
# Link directories (for Sophus .so)
############################
link_directories(${SOPHUS_LIBRARY_DIR})

############################
# Build library
############################
add_library(transform_utils
  src/config.cpp
  src/camera_model.cpp
  src/camera_load.cpp
  src/stereo_camera.cpp
)


target_link_libraries(transform_utils
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Sophus
)

############################
# RPATH (so runtime can find libSophus.so)
############################
set_target_properties(transform_utils PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${SOPHUS_LIBRARY_DIR}"
)

############################
# Test executable
############################
add_executable(test_camera_loader
  test/test_camera_loader.cpp
)

add_executable(test_stereo_camera
  test/test_stereo_camera.cpp
)

add_executable(door_center_to_world_node
  test/DoorCenterToWorldNode.cpp
)

add_dependencies(door_center_to_world_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(test_camera_loader
  transform_utils
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Sophus
)

target_link_libraries(test_stereo_camera
  transform_utils
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Sophus
)

target_link_libraries(door_center_to_world_node
  transform_utils
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Sophus
)

set_target_properties(test_camera_loader PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${SOPHUS_LIBRARY_DIR}"
)

set_target_properties(test_stereo_camera PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${SOPHUS_LIBRARY_DIR}"
)

set_target_properties(door_center_to_world_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${SOPHUS_LIBRARY_DIR}"
)