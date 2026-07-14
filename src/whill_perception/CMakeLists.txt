cmake_minimum_required(VERSION 3.16)
project(whill_perception)

# Patchwork++ core needs C++20 (see src/third_party/patchwork_plusplus/
# cpp/CMakeLists.txt). We inherit that for our wrapper.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Pull in the BSD-2-Clause Patchwork++ C++ core as a CMake subdirectory.
# Upstream's ros/ package is skipped via a COLCON_IGNORE we drop after
# vcs import — its package.xml declares GPL-3.0 which does not fit the
# project's permissive-runtime policy. See src/whill_perception/README.md
# and docs/decisions/0011-ground-removal-choice.md.
set(_PATCHWORKPP_CORE_DIR
  ${CMAKE_CURRENT_LIST_DIR}/../third_party/patchwork_plusplus/cpp)
if(NOT EXISTS ${_PATCHWORKPP_CORE_DIR}/CMakeLists.txt)
  message(FATAL_ERROR
    "Patchwork++ C++ core not found at ${_PATCHWORKPP_CORE_DIR}. \n"
    "Run vcs import (whill_lab.repos) from the repo root, then\n"
    "touch src/third_party/patchwork_plusplus/ros/COLCON_IGNORE. \n"
    "See src/whill_perception/README.md.")
endif()
# EXCLUDE_FROM_ALL keeps the core library out of `install` by default —
# we only link a shared object into our node, no separate ROS install
# artefact from the core. USE_SYSTEM_EIGEN3 ON avoids the FetchContent
# path so we do not silently drag a different Eigen into the workspace.
set(USE_SYSTEM_EIGEN3 ON CACHE BOOL "Use system Eigen3 for Patchwork++")
add_subdirectory(${_PATCHWORKPP_CORE_DIR} patchworkpp_cpp_build EXCLUDE_FROM_ALL)

add_executable(patchworkpp_node src/patchworkpp_node.cpp)
# Plain (non-keyword) target_link_libraries so ament_target_dependencies —
# which itself calls the plain form — does not error out on mixed styles.
target_link_libraries(patchworkpp_node
  patchworkpp::ground_seg_cores
  Eigen3::Eigen)
ament_target_dependencies(patchworkpp_node
  rclcpp
  sensor_msgs)

install(TARGETS patchworkpp_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
