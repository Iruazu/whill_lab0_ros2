cmake_minimum_required(VERSION 3.8)
project(whill_sensors_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/velodyne_self_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
