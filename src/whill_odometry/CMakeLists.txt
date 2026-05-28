cmake_minimum_required(VERSION 3.8)
project(whill_odometry)

# Match the rest of the workspace: warnings on, no aggressive opts. The
# composable component is small enough that PIC-only build is fine; the
# standalone executable links the same component object.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(whill_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Component library — registered with rclcpp_components so the node can be
# composed into a shared container (planned in M6 bringup) without changing
# its callbacks.
add_library(whill_odometry_component SHARED
  src/odometry_node.cpp
)
target_include_directories(whill_odometry_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(whill_odometry_component
  rclcpp
  rclcpp_components
  nav_msgs
  geometry_msgs
  whill_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
rclcpp_components_register_node(whill_odometry_component
  PLUGIN "whill_odometry::OdometryNode"
  EXECUTABLE odometry_node
)

install(TARGETS whill_odometry_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(whill_odometry_component)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  nav_msgs
  geometry_msgs
  whill_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

ament_package()
